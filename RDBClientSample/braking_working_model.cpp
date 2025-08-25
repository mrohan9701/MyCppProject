// // Copyright 2024 Hexagon Manufacturing Intelligence GmbH

// /*#include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>
// #include <errno.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <netinet/in.h>
// #include <netinet/tcp.h>
// // #include <arpa/inet.h>
// #include <netdb.h>
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <sys/ioctl.h>
// #include "VtdToolkit/RDBHandler.hh"

// #define DEFAULT_PORT_TC     48190   /* task control port */
// #define DEFAULT_PORT_SENSOR 48195   /* sensor port */
// #define DEFAULT_BUFFER_SIZE 204800
// #define MAX_CONNECTIONS     2       /* max number of TCP connections */

// typedef struct {
//     int            id;             // unique connection ID
//     char           serverAddr[128];// server address
//     int            port;           // server port
//     int            desc;           // socket descriptor
//     unsigned int   bytesInBuffer;  // received byte count in buffer
//     size_t         bufferSize;     // buffer total size
//     unsigned char* pData;          // receive buffer pointer
// } Connection_t;

// // Global connections
// Connection_t sConnection[MAX_CONNECTIONS];

// // Global objects for ego vehicle and nearest detected object
// RDB_OBJECT_STATE_t mOwnObject;
// RDB_OBJECT_STATE_t mNearestObject;

// static const unsigned int scMainConnection   = 0;
// static const unsigned int scSensorConnection = 1;

// static const unsigned int scOwnId = 1;          // own vehicle ID
// static const double scOwnPreferredSpeed = 30.0; // m/s

// // TTC and braking thresholds
// static constexpr double scBrakingTTCThreshold = 2.0;  // seconds
// static constexpr double scMaxBrake = -1.0;            // max braking accel
// double closestObjectDistance = 1e9;

// // Function declarations
// int openPort(int &descriptor, int portNo, const char* serverAddr);
// void initConnections();
// void initConnection(Connection_t &conn);
// void readConnection(Connection_t &conn, bool waitForMessage, bool verbose);
// void parseRDBMessage(Connection_t &conn, RDB_MSG_t* msg);
// void parseRDBMessageEntry(Connection_t &conn, const double &simTime, unsigned int simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr);
// void handleRDBitem(const double &simTime, unsigned int simFrame, const RDB_OBJECT_STATE_t &item, bool isSensor, bool isExtended, bool firstElement);

// void sendDriverCtrl(int &sendSocket, const double &simTime, unsigned int simFrame);
// double computeTTC(const RDB_OBJECT_STATE_t &own, const RDB_OBJECT_STATE_t &obj);
// void usage();
// void ValidateArgs(int argc, char **argv);


// // === Helper: compute Time-To-Collision (TTC) ===
// double computeTTC(const RDB_OBJECT_STATE_t &own, const RDB_OBJECT_STATE_t &obj)
// {
//     double dx = obj.base.pos.x - own.base.pos.x;
//     double relVel = own.ext.speed.x - obj.ext.speed.x;

//     if (relVel <= 0.0)
//         return INFINITY;  // no collision if own speed is less or equal

//     return dx / relVel;
// }

// void usage()
// {
//     printf("usage: client [-t:x] [-s:x] [-T:IP] [-S:IP]\n\n");
//     printf("       -t:x     taskControl port\n");
//     printf("       -s:x     sensor port\n");
//     printf("       -T:IP    taskControl IPv4 address\n");
//     printf("       -S:IP    sensor IPv4 address\n");
//     exit(1);
// }

// void ValidateArgs(int argc, char **argv)
// {
//     for (int i = 1; i < argc; i++)
//     {
//         if ((argv[i][0] == '-') || (argv[i][0] == '/'))
//         {
//             switch (argv[i][1])
//             {
//             case 't': // Remote port taskControl
//                 if (strlen(argv[i]) > 3)
//                     sConnection[0].port = atoi(&argv[i][3]);
//                 break;
//             case 's': // Remote port sensor
//                 if (strlen(argv[i]) > 3)
//                     sConnection[1].port = atoi(&argv[i][3]);
//                 break;
//             case 'T': // TC server
//                 if (strlen(argv[i]) > 3)
//                     strcpy(sConnection[0].serverAddr, &argv[i][3]);
//                 break;
//             case 'S': // sensor server
//                 if (strlen(argv[i]) > 3)
//                     strcpy(sConnection[1].serverAddr, &argv[i][3]);
//                 break;
//             default:
//                 usage();
//                 break;
//             }
//         }
//     }
// }

// int main(int argc, char* argv[])
// {
//     bool sVerbose = false;

//     // Initialize connections and buffers
//     initConnections();

//     // Parse the command line arguments
//     ValidateArgs(argc, argv);

//     // Clear objects initially
//     memset(&mNearestObject, 0, sizeof(RDB_OBJECT_STATE_t));
//     memset(&mOwnObject, 0, sizeof(RDB_OBJECT_STATE_t));

//     // Open Task Control port
//     if (!openPort(sConnection[0].desc, sConnection[0].port, sConnection[0].serverAddr))
//         return -1;

//     // Open Sensor port
//     if (!openPort(sConnection[1].desc, sConnection[1].port, sConnection[1].serverAddr))
//         return -1;

//     // Main processing loop: receive, parse, compute, send
//     for (;;)
//     {
//         readConnection(sConnection[0], false, sVerbose);
//         readConnection(sConnection[1], false, sVerbose);

//         usleep(1000); // 1 ms pause to ease CPU load
//     }

//     return 0;
// }


// void initConnections()
// {
//     memset(sConnection, 0, MAX_CONNECTIONS * sizeof(Connection_t));

//     for (int i = 0; i < MAX_CONNECTIONS; i++)
//     {
//         initConnection(sConnection[i]);
//         sConnection[i].id = i;
//     }

//     sConnection[0].port = DEFAULT_PORT_TC;
//     sConnection[1].port = DEFAULT_PORT_SENSOR;
// }


// void initConnection(Connection_t &conn)
// {
//     strcpy(conn.serverAddr, "127.0.0.1");

//     conn.desc = -1;
//     conn.bufferSize = sizeof(RDB_MSG_t);
//     conn.pData = (unsigned char*)calloc(1, conn.bufferSize);
//     conn.bytesInBuffer = 0;
// }


// int openPort(int &descriptor, int portNo, const char* serverAddr)
// {
//     struct sockaddr_in server;
//     struct hostent *host = nullptr;

//     descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
//     if (descriptor == -1)
//     {
//         fprintf(stderr, "openPort: socket() failed: %s\n", strerror(errno));
//         return 0;
//     }

//     int opt = 1;
//     setsockopt(descriptor, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

//     server.sin_family = AF_INET;
//     server.sin_port = htons(portNo);
//     server.sin_addr.s_addr = inet_addr(serverAddr);

//     if (server.sin_addr.s_addr == INADDR_NONE)
//     {
//         host = gethostbyname(serverAddr);
//         if (!host)
//         {
//             fprintf(stderr, "openPort: unable to resolve server: %s\n", serverAddr);
//             return 0;
//         }
//         memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
//     }

//     // Set socket non-blocking
//     opt = 1;
//     ioctl(descriptor, FIONBIO, &opt);

//     // Non-blocking connect with select()
//     int res = connect(descriptor, (struct sockaddr*)&server, sizeof(server));
//     if (res < 0)
//     {
//         if (errno != EINPROGRESS)
//         {
//             fprintf(stderr, "connect() failed immediately: %s\n", strerror(errno));
//             close(descriptor);
//             return 0;
//         }
//     }

//     fd_set wfds;
//     struct timeval tv;
//     FD_ZERO(&wfds);
//     FD_SET(descriptor, &wfds);

//     tv.tv_sec = 5;  // wait max 5 seconds
//     tv.tv_usec = 0;

//     res = select(descriptor + 1, NULL, &wfds, NULL, &tv);
//     if (res <= 0)
//     {
//         fprintf(stderr, "connect() timeout or error\n");
//         close(descriptor);
//         return 0;
//     }

//     int err = 0;
//     socklen_t len = sizeof(err);
//     if (getsockopt(descriptor, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0)
//     {
//         fprintf(stderr, "connect() error after select: %s\n", strerror(err));
//         close(descriptor);
//         return 0;
//     }

//     // Set back to blocking or keep non-blocking as needed (here we set back to blocking)
//     opt = 0;
//     ioctl(descriptor, FIONBIO, &opt);

//     fprintf(stderr, "Port %d connected to %s\n", portNo, serverAddr);
//     return 1;
// }


// // Receive and buffer data, parse complete messages
// void readConnection(Connection_t &conn, bool waitForMessage, bool verbose)
// {
//     static char* szBuffer = (char*)calloc(1, DEFAULT_BUFFER_SIZE);
//     int ret = -1;

//     bool bMsgComplete = false;

//     if (verbose)
//         fprintf(stderr, "readConnection: start reading connection %d\n", conn.id);

//     do
//     {
//         ret = recv(conn.desc, szBuffer, DEFAULT_BUFFER_SIZE, 0);

//         if (ret > 0)
//         {
//             // Grow buffer if necessary
//             if ((conn.bytesInBuffer + ret) > conn.bufferSize)
//             {
//                 unsigned char* newBuffer = (unsigned char*)realloc(conn.pData, conn.bytesInBuffer + ret);
//                 if (!newBuffer)
//                 {
//                     fprintf(stderr, "readConnection: failed to realloc buffer\n");
//                     exit(1);
//                 }
//                 conn.pData = newBuffer;
//                 conn.bufferSize = conn.bytesInBuffer + ret;
//             }

//             memcpy(conn.pData + conn.bytesInBuffer, szBuffer, ret);
//             conn.bytesInBuffer += ret;

//             // Check for complete message
//             while (conn.bytesInBuffer >= sizeof(RDB_MSG_HDR_t))
//             {
//                 RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*)conn.pData;

//                 if (hdr->magicNo != RDB_MAGIC_NO)
//                 {
//                     fprintf(stderr, "readConnection: magicNo mismatch, discarding buffer\n");
//                     conn.bytesInBuffer = 0;
//                     break;
//                 }

//                 unsigned int fullMsgSize = hdr->headerSize + hdr->dataSize;
//                 if (conn.bytesInBuffer < fullMsgSize)
//                     break; // incomplete message, wait for more data

//                 // Verbose print (optional)
//                 if (verbose)
//                     Framework::RDBHandler::printMessage((RDB_MSG_t*)conn.pData, true);

//                 // Parse complete RDB message
//                 parseRDBMessage(conn, (RDB_MSG_t*)conn.pData);

//                 // Remove processed message from buffer
//                 memmove(conn.pData, conn.pData + fullMsgSize, conn.bytesInBuffer - fullMsgSize);
//                 conn.bytesInBuffer -= fullMsgSize;

//                 bMsgComplete = true;
//             }
//         }
//         else if (ret == 0)
//         {
//             // Connection closed by peer, handle if needed
//             fprintf(stderr, "readConnection: connection %d closed by peer\n", conn.id);
//             close(conn.desc);
//             conn.desc = -1;
//             return;
//         }
//         else // ret < 0
//         {
//             if (errno == EAGAIN || errno == EWOULDBLOCK)
//             {
//                 // No data available for non-blocking socket
//                 break;
//             }
//             else
//             {
//                 fprintf(stderr, "readConnection: recv error on conn %d: %s\n", conn.id, strerror(errno));
//                 close(conn.desc);
//                 conn.desc = -1;
//                 return;
//             }
//         }

//     } while ((ret > 0) && (!waitForMessage || bMsgComplete));

//     if (verbose)
//         fprintf(stderr, "readConnection: finished reading connection %d\n", conn.id);
// }

// // Parse a full RDB message and its entries
// void parseRDBMessage(Connection_t &conn, RDB_MSG_t* msg)
// {
//     if (!msg || msg->hdr.dataSize == 0)
//         return;
   
//     RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)((char*)msg + msg->hdr.headerSize);
//     uint32_t remainingBytes = msg->hdr.dataSize;

//     while (remainingBytes)
//     {
//         parseRDBMessageEntry(conn, msg->hdr.simTime, msg->hdr.frameNo, entry);

//         remainingBytes -= (entry->headerSize + entry->dataSize);

//         if (remainingBytes)
//             entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
//     }
// }

// // Parse individual entries inside an RDB message
// void parseRDBMessageEntry(Connection_t &conn, const double &simTime, unsigned int simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr)
// {
//     if (!entryHdr)
//         return;


//     int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

//     if (!noElements)  // special entries
//     {
//         switch (entryHdr->pkgId)
//         {
//             case RDB_PKG_ID_START_OF_FRAME:
//                 fprintf(stderr, "parseRDBMessageEntry: conn %d: start of frame\n", conn.id);
//                 if (conn.id == scSensorConnection)
//                     memset(&mNearestObject, 0, sizeof(RDB_OBJECT_STATE_t));
//                     closestObjectDistance = 1e9;
//                 break;

//             case RDB_PKG_ID_END_OF_FRAME:
//                 fprintf(stderr, "parseRDBMessageEntry: conn %d: end of frame\n", conn.id);
//                 if (conn.id == scMainConnection)
//                     sendDriverCtrl(conn.desc, simTime, simFrame);
//                 break;

//             default:
//                 return;
//         }
//         return;
//     }

//     char* dataPtr = (char*)entryHdr + entryHdr->headerSize;
//     bool firstElement = true;

//     while (noElements--)
//     {
//         switch (entryHdr->pkgId)
//         {
//         case RDB_PKG_ID_OBJECT_STATE:
//             if ((conn.id == scSensorConnection) && (entryHdr->flags & RDB_PKG_FLAG_EXTENDED))
//             {
//                 handleRDBitem(simTime, simFrame, *(RDB_OBJECT_STATE_t*)dataPtr, true, true, firstElement);
//                 firstElement = false;
//             }
//             else if (conn.id == scMainConnection)
//             {
//                 handleRDBitem(simTime, simFrame, *(RDB_OBJECT_STATE_t*)dataPtr, true, false, firstElement);
//             }
//             break;

//         default:
//             break;
//         }

//         dataPtr += entryHdr->elementSize;
//     }
// }

// // Update global ego and nearest object states and log relevant info
// void handleRDBitem(const double &simTime, unsigned int simFrame, const RDB_OBJECT_STATE_t &item, bool isSensor, bool isExtended, bool firstElement)
// {
//     if (isSensor)
//     {
//         // Ignore the ego vehicle itself
//         if (item.base.id == mOwnObject.base.id)
//             return;

//         // Calculate relative position
//         double dx = item.base.pos.x - mOwnObject.base.pos.x;
//         double dy = item.base.pos.y - mOwnObject.base.pos.y;

//         // Calculate ego heading vector from speed
//         double egoSpeedX = mOwnObject.ext.speed.x;
//         double egoSpeedY = mOwnObject.ext.speed.y;
//         double egoSpeedMag = std::sqrt(egoSpeedX * egoSpeedX + egoSpeedY * egoSpeedY);

//         // If ego is stationary, skip (can't determine direction)
//         if (egoSpeedMag < 0.1)
//             return;

//         double forwardX = egoSpeedX / egoSpeedMag;
//         double forwardY = egoSpeedY / egoSpeedMag;

//         // Project relative position onto ego's heading direction (dot product)
//         double longitudinalDist = dx * forwardX + dy * forwardY;
//         double lateralDist = std::abs(-dx * forwardY + dy * forwardX);  // Perpendicular to heading

//         // Only consider objects ahead and within ego lane (~3.5 meters width)
//         if (longitudinalDist > 0 && lateralDist < 3.5)
//         {
//             if (longitudinalDist < closestObjectDistance)
//             {
//                 closestObjectDistance = longitudinalDist;
//                 mNearestObject = item;
//             }
//         }
//     }
//     else
//     {
//         // Update own ego vehicle state (from main connection)
//         if (item.base.id == 1 /* EGO ID */)
//         {
//             mOwnObject = item;
//         }
//     }

//     // Debug print for each object (optional)
//     fprintf(stderr, "handleRDBitem: simTime=%.3f simFrame=%u\n", simTime, simFrame);
//     fprintf(stderr, "  object: %s id=%d pos=%.3f %.3f %.3f speed=%.3f %.3f %.3f\n",
//             item.base.name, item.base.id,
//             item.base.pos.x, item.base.pos.y, item.base.pos.z,
//             item.ext.speed.x, item.ext.speed.y, item.ext.speed.z);
// }


// // Send driver control commands incorporating ACC and AEB logic
// void sendDriverCtrl(int &sendSocket, const double &simTime, unsigned int simFrame)
// {
//     Framework::RDBHandler myHandler;
//     myHandler.initMsg();

//     RDB_DRIVER_CTRL_t* myDriver = (RDB_DRIVER_CTRL_t*)myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);
//     if (!myDriver)
//         return;
//         fprintf(stderr, "\n[DEBUG] simFrame=%u simTime=%.2f\n", simFrame, simTime);
// fprintf(stderr, "EGO: id=%d x=%.2f y=%.2f speedX=%.2f speedY=%.2f\n",
//         mOwnObject.base.id, mOwnObject.base.pos.x, mOwnObject.base.pos.y,
//         mOwnObject.ext.speed.x, mOwnObject.ext.speed.y);

// if (mNearestObject.base.id != 0) {
//     fprintf(stderr, "NEAREST: id=%d x=%.2f y=%.2f speedX=%.2f\n",
//             mNearestObject.base.id,
//             mNearestObject.base.pos.x, mNearestObject.base.pos.y,
//             mNearestObject.ext.speed.x);
// } else {
//     fprintf(stderr, "NEAREST: None detected this frame\n");
// }

//     bool haveSensorObject = (mNearestObject.base.id > 0);
//     double ownSpeed = sqrt(mOwnObject.ext.speed.x * mOwnObject.ext.speed.x + mOwnObject.ext.speed.y * mOwnObject.ext.speed.y);

//     double accelTgtDist = 0.0;
//     double accelTgtSpeed = (scOwnPreferredSpeed - ownSpeed) / 2.0;  // ACC: speed tracking

//     if (haveSensorObject)
//     {
//         if (mNearestObject.ext.speed.x < -1.0e-3)
//             accelTgtSpeed = 2.0 * mNearestObject.ext.speed.x / 5.0;
//         else
//             accelTgtSpeed = 0.0;

//         double tgtDist = ownSpeed * 2.0;
//         if (tgtDist < 10.0)
//             tgtDist = 10.0;

//         accelTgtDist = (mNearestObject.base.pos.x - tgtDist) / 10.0;
//     }

//     // --- AEB logic ---
//     double ttc = computeTTC(mOwnObject, mNearestObject);
//     bool shouldBrake = (haveSensorObject && (ttc < scBrakingTTCThreshold));

//     if (shouldBrake)
//     {
//         fprintf(stderr, "AEB activated! TTC = %.2fs (threshold %.2fs) - applying brake\n", ttc, scBrakingTTCThreshold);
//         myDriver->accelTgt = scMaxBrake;
//     }
//     else
//     {
//         myDriver->accelTgt = accelTgtDist + accelTgtSpeed;
//     }

//     myDriver->playerId = scOwnId;
//     myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_ADD_ON;

//     int retVal = send(sendSocket, (const char*)myHandler.getMsg(), myHandler.getMsgTotalSize(), 0);

//     if (retVal <= 0)
//         fprintf(stderr, "sendDriverCtrl: failed to send driver control\n");
//     else
//         fprintf(stderr, "sendDriverCtrl: driver control sent (accel=%.3lf)\n", myDriver->accelTgt);
// }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "VtdToolkit/RDBHandler.hh"

#define DEFAULT_PORT        48190
#define DEFAULT_BUFFER      204800
#define EGO_ID              1

#ifndef SIM_DT
#define SIM_DT 0.05f
#endif

// Flag just for SIM_MODE logs
bool aebApplied = false;

struct VehState {
    bool has = false;
    int id = -1;
    char name[32] = {0};
    int category = 0;
    int type = 0;
    float x=0, y=0, z=0;
    float h=0, p=0, r=0;
    float vx=0, vy=0, vz=0;
    float ax=0, ay=0, az=0;
};

static inline void worldToEgoLocal(float x, float y, float ego_x, float ego_y, float ego_h,
                                   float& x_local, float& y_local)
{
    float dx = x - ego_x;
    float dy = y - ego_y;
    float ch = cosf(ego_h);
    float sh = sinf(ego_h);
    x_local =  ch*dx + sh*dy;
    y_local = -sh*dx + ch*dy;
}

// ---------- NEW: send driver control to VTD ----------
static void sendDriverCtrl(int sock, double simTime, unsigned int simFrame, 
                           float accelTgt = 0.0f, float brakeTgt = 0.0f, float steerTgt = 0.0f,
                           bool stopNow = false ) 
{
    Framework::RDBHandler handler;
    handler.initMsg();

    RDB_DRIVER_CTRL_t* drv =
        (RDB_DRIVER_CTRL_t*)handler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL, 1, sizeof(RDB_DRIVER_CTRL_t));
    if (!drv) return;

    memset(drv, 0, sizeof(RDB_DRIVER_CTRL_t));
    drv->playerId = EGO_ID;

    if (stopNow) {
        drv->accelTgt   = 0.0f;   // no throttle
        drv->brakePedal = 1.0f; 
        drv->speedTgt = 0;
        drv->steeringTgt = steerTgt;  // full brake
        fprintf(stderr, "[CTRL] Full brake applied (accelTgt=%.2f, brakePedal=%.2f)\n", accelTgt, brakeTgt);
    } else {
        drv->accelTgt   = accelTgt;  // normal throttle
        drv->brakePedal = brakeTgt;  // normal brake
        //drv->speedTgt = speedTgt;
        fprintf(stderr, "[CTRL] Normal control (accelTgt=%.2f, brakePedal=%.2f)\n", accelTgt, brakeTgt);
    }

    drv->steeringTgt = steerTgt;
    drv->gear        = 1;

    // mark which inputs are valid
    drv->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING |
                         RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL |
                         RDB_DRIVER_INPUT_VALIDITY_BRAKE | RDB_DRIVER_INPUT_VALIDITY_GEAR ;

    int ret = send(sock, (const char*)handler.getMsg(), handler.getMsgTotalSize(), 0);
    if (ret <= 0) perror("sendDriverCtrl() send failed");
}

// (Kept for SIM_MODE only; not used in live RDB control)
void autonomousEmergencyBraking(VehState& ego, const VehState& frontObj,
                               float warningDistance, float emergencyDistance,
                               float maxDecelRate)
{
    aebApplied = false;
    if (!frontObj.has) return;

    float x_local, y_local;
    worldToEgoLocal(frontObj.x, frontObj.y, ego.x, ego.y, ego.h, x_local, y_local);
    float dist = hypotf(x_local, y_local);
    float rel_vx = frontObj.vx - ego.vx;
    if (rel_vx <= 0.0f) return;

    float ttc = dist / rel_vx;
    printf("TTC: %.3f s, Distance: %.3f m, Ego Vx: %.3f m/s\n", ttc, dist, ego.vx);

    const float ttc_warning = 3.0f;
    const float ttc_emergency = 1.0f;

    if (ttc <= ttc_emergency && dist <= emergencyDistance) {
        if (ego.vx > 0.0f) {
            printf("AEB Emergency brake! Speed reduced from %.3f to 0.000 m/s\n", ego.vx);
            ego.vx = 0.0f;
            aebApplied = true;
        }
    } else if (ttc <= ttc_warning && dist <= warningDistance) {
        float decelFactor = maxDecelRate * (ttc_warning - ttc) / ttc_warning;
        if (decelFactor < 0.0f) decelFactor = 0.0f;
        if (decelFactor > 1.0f) decelFactor = 1.0f;
        float newVx = ego.vx * (1.0f - decelFactor);
        if (newVx < 0.0f) newVx = 0.0f;
        if (newVx < ego.vx) {
            printf("AEB Warning brake: Speed reduced from %.3f to %.3f m/s\n", ego.vx, newVx);
            ego.vx = newVx;
            aebApplied = true;
        }
    }
}
float laneFollowingSteering(float y_local, float laneCenterY,
                            float heading, float laneHeading,
                            float kp = 0.5f, float kd = 0.2f)
{
    float y_error = y_local - laneCenterY;
    float heading_error = heading - laneHeading;

    // simple PD controller
    float steerCmd = -kp * y_error - kd * heading_error;
    return steerCmd;
}
#ifdef SIM_MODE
// Simple local sim (no RDB). Not used for real braking in VTD.
void runSimulation()
{
    float ego_x = 0.0f, ego_y = 0.0f, ego_h = 0.0f, ego_vx = 20.0f;
    float front_x = 50.0f, front_y = 0.0f, front_vx = 0.0f;

    const float dt = SIM_DT;
    const float warningDist = 30.0f, emergencyDist = 10.0f, maxDecelRate = 0.7f;

    int frameNo = 0; float simTime = 0.0f;

    while (frameNo < 500) {
        ego_x += ego_vx * dt;

        VehState ego{}, front{};
        ego.has = true; ego.id = EGO_ID; strncpy(ego.name, "EgoVehicle", sizeof(ego.name)-1);
        ego.x = ego_x; ego.y = ego_y; ego.h = ego_h; ego.vx = ego_vx;

        front.has = true; front.id = 2; strncpy(front.name, "FrontObject", sizeof(front.name)-1);
        front.x = front_x; front.y = front_y; front.vx = front_vx;

        float x_local, y_local; worldToEgoLocal(front.x, front.y, ego.x, ego.y, ego.h, x_local, y_local);
        float dist = hypotf(x_local, y_local); float rel_v = fabsf(front.vx - ego.vx);

        printf("\n=== FRAME %d  t=%.3f ===\n", frameNo, simTime);
        printf("EGO: X=%.2f Vx=%.2f m/s\n", ego.x, ego.vx);
        printf("Front: X=%.2f Vx=%.2f m/s  Dist=%.2f m  RelV=%.2f m/s\n", front.x, front.vx, dist, rel_v);

        autonomousEmergencyBraking(ego, front, warningDist, emergencyDist, maxDecelRate);

        if (aebApplied) printf("AEB is actively controlling ego vehicle speed this frame.\n");
        else            printf("No AEB braking active this frame.\n");

        ego_vx = ego.vx;
        if (ego_vx <= 0.01f && dist < 12.0f) { printf("Simulation stop: ego stopped near object.\n"); break; }

        frameNo++; simTime += dt; usleep((int)(dt * 1e6));
    }
}
#endif

int main(int argc, char* argv[])
{
#ifdef SIM_MODE
    printf("=== Running in SIM_MODE (no VTD connection, using built-in simulation) ===\n");
    runSimulation();
    return 0;
#else
    const char* serverIP = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : DEFAULT_PORT;
    printf("=== Running in LIVE RDB MODE ===\n");
    printf("Connecting to VTD RDB server at %s:%d ...\n", serverIP, port);
    // Detection gate for “in front”
    const float lane_window_m   = 5.0f;    // |y_local| must be <= this
    const float fwd_max_range_m = 20.0f;  // ahead within this range
    const float kp_steer = 1.5f;
    const float kd_steer= 0.5f;
    const float laneCenterY = 0.0f;
    // float heading_error = ego.h - laneHeading; // if you have it
    // float steerCmd = -kp_steer * y_error - kd_steer * heading_error;




    int sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sClient < 0) { perror("socket() failed"); return 1; }

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port   = htons(port);
    if (inet_pton(AF_INET, serverIP, &server.sin_addr) != 1) { perror("inet_pton failed"); return 1; }

    if (connect(sClient, (struct sockaddr*)&server, sizeof(server)) < 0) { perror("connect() failed"); return 1; }
    printf("Connected to RDB server %s:%d\n", serverIP, port);

    char* szBuffer = new char[DEFAULT_BUFFER];
    unsigned char* pData = (unsigned char*)calloc(1, sizeof(RDB_MSG_HDR_t));
    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);

    while (true)
    {
        int ret = recv(sClient, szBuffer, DEFAULT_BUFFER, 0);
        if (ret <= 0) { perror("recv() failed or closed"); break; }

        if (bytesInBuffer + ret > bufferSize) {
            pData = (unsigned char*)realloc(pData, bytesInBuffer + ret);
            bufferSize = bytesInBuffer + ret;
        }
        memcpy(pData + bytesInBuffer, szBuffer, ret);
        bytesInBuffer += ret;

        while (bytesInBuffer >= sizeof(RDB_MSG_HDR_t)) {
            RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*)pData;
            if (hdr->magicNo != RDB_MAGIC_NO) { bytesInBuffer = 0; break; }

            unsigned int msgSize = hdr->headerSize + hdr->dataSize;
            if (bytesInBuffer < msgSize) break;

            VehState ego{};
            VehState bestFront{};
            float bestFrontDist = 1e9f;

            // 1st pass: find ego
            RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)((char*)hdr + hdr->headerSize);
            uint32_t remaining = hdr->dataSize;

            while (remaining) {
                if (entry->pkgId == RDB_PKG_ID_OBJECT_STATE) {
                    int noElements = entry->elementSize ? (entry->dataSize / entry->elementSize) : 0;
                    char* dataPtr = (char*)entry + entry->headerSize;

                    for (int i = 0; i < noElements; i++) {
                        RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)dataPtr;

                        if (obj->base.id == EGO_ID) {
                            ego.has = true; ego.id = obj->base.id;
                            strncpy(ego.name, obj->base.name, sizeof(ego.name) - 1);
                            ego.category = obj->base.category; ego.type = obj->base.type;
                            ego.x = obj->base.pos.x; ego.y = obj->base.pos.y; ego.z = obj->base.pos.z;
                            ego.h = obj->base.pos.h; ego.p = obj->base.pos.p; ego.r = obj->base.pos.r;
                            ego.vx = obj->ext.speed.x; ego.vy = obj->ext.speed.y; ego.vz = obj->ext.speed.z;
                            ego.ax = obj->ext.accel.x; ego.ay = obj->ext.accel.y; ego.az = obj->ext.accel.z;
                        }
                        dataPtr += entry->elementSize;
                    }
                }
                remaining -= (entry->headerSize + entry->dataSize);
                if (remaining) entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
            }

            // 2nd pass: find nearest “in-front” target
            if (ego.has) {
                entry = (RDB_MSG_ENTRY_HDR_t*)((char*)hdr + hdr->headerSize);
                remaining = hdr->dataSize;

                while (remaining) {
                    if (entry->pkgId == RDB_PKG_ID_OBJECT_STATE) {
                        int noElements = entry->elementSize ? (entry->dataSize / entry->elementSize) : 0;
                        char* dataPtr = (char*)entry + entry->headerSize;

                        for (int i = 0; i < noElements; i++) {
                            RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)dataPtr;
                            if (obj->base.id != EGO_ID) {
                                float x_local, y_local;
                                worldToEgoLocal(obj->base.pos.x, obj->base.pos.y, ego.x, ego.y, ego.h, x_local, y_local);

                                if (x_local > 0.0f &&
                                    fabsf(y_local) <= lane_window_m &&
                                    x_local <= fwd_max_range_m)
                                {
                                    float dist = hypotf(x_local, y_local);
                                    if (dist < bestFrontDist) {
                                        bestFrontDist = dist;
                                        bestFront.has = true;
                                        bestFront.id = obj->base.id;
                                        strncpy(bestFront.name, obj->base.name, sizeof(bestFront.name) - 1);
                                        bestFront.category = obj->base.category; bestFront.type = obj->base.type;
                                        bestFront.x = obj->base.pos.x; bestFront.y = obj->base.pos.y; bestFront.z = obj->base.pos.z;
                                        bestFront.h = obj->base.pos.h; bestFront.p = obj->base.pos.p; bestFront.r = obj->base.pos.r;
                                        bestFront.vx = obj->ext.speed.x; bestFront.vy = obj->ext.speed.y; bestFront.vz = obj->ext.speed.z;
                                        bestFront.ax = obj->ext.accel.x; bestFront.ay = obj->ext.accel.y; bestFront.az = obj->ext.accel.z;
                                    }
                                }
                            }
                            dataPtr += entry->elementSize;
                        }
                    }
                    remaining -= (entry->headerSize + entry->dataSize);
                    if (remaining) entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
                }
            }

            // Logs & CONTROL
            if (ego.has) {
                printf("\n=== FRAME %u  t=%.3f ===\n", hdr->frameNo, hdr->simTime);
                printf("EGO (ID=%d, Name=%s)\n", ego.id, ego.name);
                printf("  Pos:  X=%.3f Y=%.3f Z=%.3f (m)\n", ego.x, ego.y, ego.z);
                printf("  Ori:  H=%.3f P=%.3f R=%.3f (rad)\n", ego.h, ego.p, ego.r);
                printf("  Vel:  Vx=%.3f Vy=%.3f Vz=%.3f (m/s)\n", ego.vx, ego.vy, ego.vz);

                float accelCmd = 0.0f; // default: no accel/brake
                float brakeCmd = 0.0f;
                float steerCmd = laneFollowingSteering(ego.y, laneCenterY, ego.h, 0.0f, 1.5f, 0.5f);
                //float speedCmd = 5.0f;
                if (bestFront.has) {
                        autonomousEmergencyBraking(ego, bestFront, 30.0f, 10.0f, 0.7f);

              
                    float x_local, y_local;
                    worldToEgoLocal(bestFront.x, bestFront.y, ego.x, ego.y, ego.h, x_local, y_local);
                    float dist_xy = hypotf(x_local, y_local);
                    printf("Front Object (ID=%d, Name=%s)  dist=%.2f m, x_fwd=%.2f, y_left=%.2f\n",
                           bestFront.id, bestFront.name[0] ? bestFront.name : "N/A", dist_xy, x_local, y_local);

                    // ----------- SIMPLE RULE: if anything in front, FULL BRAKE -----------
                     accelCmd = -10.0f;
                    brakeCmd = 1.0f;
                    printf("Command: FULL BRAKE (accelTgt=%.2f) (brakeCmd=%.2f\n", accelCmd, brakeCmd);
                     } else {
                            float targetSpeed = 60.0f / 3.6f; // 50 km/h in m/s

                            if (ego.vx < targetSpeed) {
                                accelCmd = 1.0f;   // gentle acceleration
                                brakeCmd = 0.0f;
                                printf("CRUISE: accelerating (vx=%.2f m/s, target=%.2f)\n", ego.vx, targetSpeed);
                            } else {
                                accelCmd = 0.0f;   // stop accelerating, hold speed
                                brakeCmd = 0.0f;
                                printf("CRUISE: holding speed (vx=%.2f m/s)\n", ego.vx);
                            }  
                }
                 const float kp_steer = 0.5f;  // proportional gain
                 float y_error = ego.y - laneCenterY;
                float steerTgt = -kp_steer * y_error;
                 printf("Lane following: y_error=%.3f → steerCmd=%.3f\n", y_error, steerTgt);

                // >>> Send control to VTD <<<
                sendDriverCtrl(sClient, hdr->simTime, hdr->frameNo, accelCmd,brakeCmd,steerCmd);
            }

            memmove(pData, pData + msgSize, bytesInBuffer - msgSize);
            bytesInBuffer -= msgSize;
        }
    }

    close(sClient);
    delete[] szBuffer;
    free(pData);
    return 0;
#endif
}

