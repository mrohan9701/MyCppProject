// Copyright 2025 Hexagon Manufacturing Intelligence GmbH
// Lane Keeping Assist (LKA) System Implementation
// Based on VTD RDB Framework

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <chrono>
#include <algorithm>
#include "VtdToolkit/RDBHandler.hh"

#define DEFAULT_PORT        48190
#define DEFAULT_BUFFER      204800

// LKA System Parameters
#define LKA_MIN_SPEED       5.0     // Minimum speed for LKA activation (m/s)
#define LKA_MAX_SPEED       35.0    // Maximum speed for LKA operation (m/s)
#define LKA_MAX_LATERAL_OFFSET  2.0 // Maximum lateral offset before warning (m)
#define LKA_LANE_WIDTH      3.5     // Standard lane width (m)
#define LKA_LOOKAHEAD_TIME  1.5     // Lookahead time for prediction (s)

// PID Controller Parameters
#define PID_KP  0.8     // Proportional gain
#define PID_KI  0.1     // Integral gain  
#define PID_KD  0.3     // Derivative gain
#define PID_MAX_OUTPUT  0.3  // Maximum steering angle output (rad)
#define PID_WINDUP_LIMIT 1.0 // Integral windup limit

// Global variables
char  szServer[128];
int   iPort = DEFAULT_PORT;
int   sClient;

// LKA System State Structure
struct LKAState {
    bool isActive;
    bool isEnabled;
    double currentSpeed;
    double lateralOffset;
    double lateralVelocity;
    double yawRate;
    double roadCurvature;
    double laneWidth;
    std::chrono::high_resolution_clock::time_point lastUpdateTime;
    
    // PID Controller State
    double pidError;
    double pidErrorPrev;
    double pidIntegral;
    double pidOutput;
    
    // Safety monitoring
    bool driverOverride;
    double handsTorque;
    int systemHealthStatus;
    
    LKAState() : isActive(false), isEnabled(false), currentSpeed(0.0), 
                 lateralOffset(0.0), lateralVelocity(0.0), yawRate(0.0),
                 roadCurvature(0.0), laneWidth(LKA_LANE_WIDTH),
                 pidError(0.0), pidErrorPrev(0.0), pidIntegral(0.0), pidOutput(0.0),
                 driverOverride(false), handsTorque(0.0), systemHealthStatus(1) {
        lastUpdateTime = std::chrono::high_resolution_clock::now();
    }
};

LKAState lkaState;

// Function prototypes
void parseRDBMessage(RDB_MSG_t* msg, bool & isImage);
void parseRDBMessageEntry(const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr);
void handleRDBitem(const double & simTime, const unsigned int & simFrame, RDB_OBJECT_STATE_t & item, bool isExtended);
void handleRoadPosition(const double & simTime, const unsigned int & simFrame, RDB_ROAD_POS_t & roadPos);
void handleLaneInfo(const double & simTime, const unsigned int & simFrame, RDB_LANE_INFO_t & laneInfo);
void updateLKASystem(const double & simTime, const unsigned int & simFrame);
double calculatePIDController(double error, double deltaTime);
bool checkLKAActivationConditions();
bool checkDriverOverride();
void sendLKASteeringCommand(int & sendSocket, const double & simTime, const unsigned int & simFrame);
void logLKAStatus(const double & simTime);

// Utility functions
void usage() {
    printf("usage: LKA_client [-p:x] [-s:IP]\n\n");
    printf("       -p:x      Remote port to send to\n");
    printf("       -s:IP     Server's IP address or hostname\n");
    printf("       Lane Keeping Assist System based on VTD RDB\n");
    exit(1);
}

void ValidateArgs(int argc, char **argv) {
    strcpy(szServer, "127.0.0.1");
    
    for(int i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') || (argv[i][0] == '/')) {
            switch (tolower(argv[i][1])) {
                case 'p':
                    if (strlen(argv[i]) > 3)
                        iPort = atoi(&argv[i][3]);
                    break;
                case 's':
                    if (strlen(argv[i]) > 3)
                        strcpy(szServer, &argv[i][3]);
                    break;
                default:
                    usage();
                    break;
            }
        }
    }
}

int main(int argc, char* argv[]) {
    char* szBuffer = new char[DEFAULT_BUFFER];
    int ret;
    struct sockaddr_in server;
    struct hostent *host = NULL;
    
    static bool sVerbose = true;
    
    printf("=== Lane Keeping Assist (LKA) System ===\n");
    printf("Initializing VTD RDB connection...\n");
    
    ValidateArgs(argc, argv);
    
    // Create socket and connect to VTD
    sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sClient == -1) {
        fprintf(stderr, "socket() failed: %s\n", strerror(errno));
        return 1;
    }
    
    int opt = 1;
    setsockopt(sClient, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    
    server.sin_family = AF_INET;
    server.sin_port = htons(iPort);
    server.sin_addr.s_addr = inet_addr(szServer);
    
    if (server.sin_addr.s_addr == INADDR_NONE) {
        host = gethostbyname(szServer);
        if (host == NULL) {
            fprintf(stderr, "Unable to resolve server: %s\n", szServer);
            return 1;
        }
        memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
    }
    
    // Connection loop
    bool bConnected = false;
    while (!bConnected) {
        if (connect(sClient, (struct sockaddr *)&server, sizeof(server)) == -1) {
            fprintf(stderr, "connect() failed: %s\n", strerror(errno));
            sleep(1);
        } else {
            bConnected = true;
        }
    }
    
    printf("Connected to VTD server at %s:%d\n", szServer, iPort);
    printf("LKA System Status: STANDBY\n");
    
    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);
    unsigned int count = 0;
    unsigned char *pData = (unsigned char*) calloc(1, bufferSize);
    
    // Main message processing loop
    for(;;) {
        bool bMsgComplete = false;
        
        while (!bMsgComplete) {
            ret = recv(sClient, szBuffer, DEFAULT_BUFFER, 0);
            
            if (ret == -1) {
                printf("recv() failed: %s\n", strerror(errno));
                break;
            }
            
            if (ret != 0) {
                // Grow buffer if needed
                if ((bytesInBuffer + ret) > bufferSize) {
                    pData = (unsigned char*) realloc(pData, bytesInBuffer + ret);
                    bufferSize = bytesInBuffer + ret;
                }
                
                memcpy(pData + bytesInBuffer, szBuffer, ret);
                bytesInBuffer += ret;
                
                // Process complete messages
                if (bytesInBuffer >= sizeof(RDB_MSG_HDR_t)) {
                    RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*) pData;
                    
                    if (hdr->magicNo != RDB_MAGIC_NO) {
                        printf("Message out of sync; discarding data\n");
                        bytesInBuffer = 0;
                        continue;
                    }
                    
                    while (bytesInBuffer >= (hdr->headerSize + hdr->dataSize)) {
                        unsigned int msgSize = hdr->headerSize + hdr->dataSize;
                        bool isImage = false;
                        
                        // Parse RDB message for LKA data
                        parseRDBMessage((RDB_MSG_t*) pData, isImage);
                        
                        // Remove processed message from buffer
                        memmove(pData, pData + msgSize, bytesInBuffer - msgSize);
                        bytesInBuffer -= msgSize;
                        
                        bMsgComplete = true;
                    }
                }
            }
        }
    }
    
    close(sClient);
    delete[] szBuffer;
    free(pData);
    return 0;
}

void parseRDBMessage(RDB_MSG_t* msg, bool & isImage) {
    if (!msg || !msg->hdr.dataSize) return;
    
    RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*) (((char*) msg) + msg->hdr.headerSize);
    uint32_t remainingBytes = msg->hdr.dataSize;
    
    while (remainingBytes) {
        parseRDBMessageEntry(msg->hdr.simTime, msg->hdr.frameNo, entry);
        
        isImage |= (entry->pkgId == RDB_PKG_ID_IMAGE);
        
        remainingBytes -= (entry->headerSize + entry->dataSize);
        if (remainingBytes) {
            entry = (RDB_MSG_ENTRY_HDR_t*) (((char*) entry) + entry->headerSize + entry->dataSize);
        }
    }
}

void parseRDBMessageEntry(const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr) {
    if (!entryHdr) return;
    
    int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;
    
    // Handle frame synchronization
    if (!noElements) {
        switch (entryHdr->pkgId) {
            case RDB_PKG_ID_START_OF_FRAME:
                // Frame start - prepare for new data
                break;
                
            case RDB_PKG_ID_END_OF_FRAME:
                // Frame end - update LKA system and send commands
                updateLKASystem(simTime, simFrame);
                sendLKASteeringCommand(sClient, simTime, simFrame);
                break;
                
            default:
                break;
        }
        return;
    }
    
    char* dataPtr = (char*) entryHdr;
    dataPtr += entryHdr->headerSize;
    
    // Process relevant RDB packages for LKA
    while (noElements--) {
        switch (entryHdr->pkgId) {
            case RDB_PKG_ID_OBJECT_STATE:
                handleRDBitem(simTime, simFrame, *((RDB_OBJECT_STATE_t*) dataPtr), 
                             entryHdr->flags & RDB_PKG_FLAG_EXTENDED);
                break;
                
            case RDB_PKG_ID_ROAD_POS:
                handleRoadPosition(simTime, simFrame, *((RDB_ROAD_POS_t*) dataPtr));
                break;
                
            case RDB_PKG_ID_LANE_INFO:
                handleLaneInfo(simTime, simFrame, *((RDB_LANE_INFO_t*) dataPtr));
                break;
                
            default:
                break;
        }
        dataPtr += entryHdr->elementSize;
    }
}

void handleRDBitem(const double & simTime, const unsigned int & simFrame, 
                   RDB_OBJECT_STATE_t & item, bool isExtended) {
    // Process ego vehicle state (assuming player ID 1)
    if (item.base.id == 1) {  // Ego vehicle
        lkaState.currentSpeed = sqrt(item.ext.speed.x * item.ext.speed.x + 
                                   item.ext.speed.y * item.ext.speed.y);
        
        if (isExtended) {
            lkaState.lateralVelocity = item.ext.speed.y;
            lkaState.yawRate = item.ext.speed.h;  // Heading rate
        }
        
        // Update timestamp
        lkaState.lastUpdateTime = std::chrono::high_resolution_clock::now();
    }
}

void handleRoadPosition(const double & simTime, const unsigned int & simFrame, RDB_ROAD_POS_t & roadPos) {
    // Extract lateral offset from road position
    lkaState.lateralOffset = roadPos.roadT;  // Lateral offset from lane center
    lkaState.roadCurvature = roadPos.roadId;  // Road curvature information
    
    // Road position data provides critical information for lane keeping
    fprintf(stderr, "Road Position - Lateral Offset: %.3f m, Road ID: %d\n", 
            roadPos.roadT, roadPos.roadId);
}

void handleLaneInfo(const double & simTime, const unsigned int & simFrame, RDB_LANE_INFO_t & laneInfo) {
    // Extract lane width and boundary information
    lkaState.laneWidth = laneInfo.width;
    
    fprintf(stderr, "Lane Info - Width: %.3f m, Type: %d\n", 
            laneInfo.width, laneInfo.type);
}

void updateLKASystem(const double & simTime, const unsigned int & simFrame) {
    // Calculate time delta
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - lkaState.lastUpdateTime);
    double deltaTime = timeDiff.count() / 1000.0;  // Convert to seconds
    
    if (deltaTime < 0.001) deltaTime = 0.001;  // Minimum delta time
    
    // Check activation conditions
    bool shouldActivate = checkLKAActivationConditions();
    bool driverOverride = checkDriverOverride();
    
    // Update system state
    if (shouldActivate && !driverOverride) {
        if (!lkaState.isActive) {
            lkaState.isActive = true;
            printf("LKA ACTIVATED at %.3f s - Speed: %.1f m/s, Offset: %.3f m\n", 
                   simTime, lkaState.currentSpeed, lkaState.lateralOffset);
        }

        printf("LKA ACTIVATED at %.3f s...\n");
         printf("LKA DEACTIVATED at %.3f s\n", simTime);

        
        // Calculate PID control output
        lkaState.pidOutput = calculatePIDController(lkaState.lateralOffset, deltaTime);
        
    } else {
        if (lkaState.isActive) {
            lkaState.isActive = false;
            lkaState.pidIntegral = 0.0;  // Reset integral term
            printf("LKA DEACTIVATED at %.3f s\n", simTime);
        }
        lkaState.pidOutput = 0.0;
    }
    
    // Log system status periodically
    static int logCounter = 0;
    if (++logCounter % 50 == 0) {  // Log every 50 frames (~1 second at 50 Hz)
        logLKAStatus(simTime);
    }
}

double calculatePIDController(double error, double deltaTime) {
    // PID Controller Implementation
    lkaState.pidError = error;  // Lateral offset is the error
    
    // Proportional term
    double proportional = PID_KP * lkaState.pidError;
    
    // Integral term with windup protection
    lkaState.pidIntegral += lkaState.pidError * deltaTime;
    lkaState.pidIntegral = std::max(-PID_WINDUP_LIMIT, 
                                   std::min(PID_WINDUP_LIMIT, lkaState.pidIntegral));
    double integral = PID_KI * lkaState.pidIntegral;
    
    // Derivative term
    double derivative = 0.0;
    if (deltaTime > 0) {
        derivative = PID_KD * (lkaState.pidError - lkaState.pidErrorPrev) / deltaTime;
    }
    
    // Total PID output
    double output = proportional + integral + derivative;
    
    // Limit output to maximum steering angle
    output = std::max(-PID_MAX_OUTPUT, std::min(PID_MAX_OUTPUT, output));
    
    // Store previous error for next iteration
    lkaState.pidErrorPrev = lkaState.pidError;
    
    return output;
}

bool checkLKAActivationConditions() {
    // Speed check
    if (lkaState.currentSpeed < LKA_MIN_SPEED || lkaState.currentSpeed > LKA_MAX_SPEED) {
        return false;
    }
    
    // Lateral offset check - don't activate if too far from center
    if (fabs(lkaState.lateralOffset) > LKA_MAX_LATERAL_OFFSET) {
        return false;
    }
    
    // System health check
    if (lkaState.systemHealthStatus != 1) {
        return false;
    }
    
    // Enable LKA system
    lkaState.isEnabled = true;
    return true;
}

bool checkDriverOverride() {
    // Check for driver steering input (simplified)
    // In real implementation, this would check steering torque sensors
    if (fabs(lkaState.handsTorque) > 3.0) {  // Nm threshold
        lkaState.driverOverride = true;
        return true;
    }
    
    lkaState.driverOverride = false;
    return false;
}

void sendLKASteeringCommand(int & sendSocket, const double & simTime, const unsigned int & simFrame) {
    if (!lkaState.isActive) return;
    
    Framework::RDBHandler myHandler;
    myHandler.initMsg();
    
    RDB_DRIVER_CTRL_t *myDriver = (RDB_DRIVER_CTRL_t*) myHandler.addPackage(
        simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);
    
    if (!myDriver) return;
    
    // Set driver control parameters
    myDriver->playerId = 1;  // Ego vehicle
    myDriver->steeringWheel = lkaState.pidOutput * 15.0;  // Convert to degrees (approx.)
    myDriver->steeringTorque = lkaState.pidOutput * 2.0;  // Steering torque
    myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_STEERING_WHEEL | 
                             RDB_DRIVER_INPUT_VALIDITY_ADD_ON;
    
    // Send command
    int retVal = send(sendSocket, (const char*)(myHandler.getMsg()), 
                     myHandler.getMsgTotalSize(), 0);
    
    if (retVal <= 0) {
        fprintf(stderr, "Failed to send LKA steering command\n");
    }
}

void logLKAStatus(const double & simTime) {
    printf("=== LKA Status at %.3f s ===\n", simTime);
    printf("  Active: %s | Speed: %.1f m/s | Lateral Offset: %.3f m\n",
           lkaState.isActive ? "YES" : "NO", lkaState.currentSpeed, lkaState.lateralOffset);
    printf("  PID Output: %.4f rad | Error: %.3f m | Integral: %.3f\n",
           lkaState.pidOutput, lkaState.pidError, lkaState.pidIntegral);
    printf("  Driver Override: %s | Lane Width: %.1f m\n",
           lkaState.driverOverride ? "YES" : "NO", lkaState.laneWidth);
    printf("================================\n");
}