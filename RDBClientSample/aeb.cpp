// Copyright 2024 Hexagon Manufacturing Intelligence GmbH

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
#include <sys/ioctl.h>
#include "VtdToolkit/RDBHandler.hh"

#define DEFAULT_PORT_TC     48190   /* task control port */
#define DEFAULT_PORT_SENSOR 48195   /* sensor port */
#define DEFAULT_BUFFER_SIZE 204800
#define MAX_CONNECTIONS     2       /* max number of TCP connections */

typedef struct {
    int            id;             // unique connection ID
    char           serverAddr[128];// server address
    int            port;           // server port
    int            desc;           // socket descriptor
    unsigned int   bytesInBuffer;  // received byte count in buffer
    size_t         bufferSize;     // buffer total size
    unsigned char* pData;          // receive buffer pointer
} Connection_t;

// Global connections
Connection_t sConnection[MAX_CONNECTIONS];

// Global objects for ego vehicle and nearest detected object
RDB_OBJECT_STATE_t mOwnObject;
RDB_OBJECT_STATE_t mNearestObject;

static const unsigned int scMainConnection   = 0;
static const unsigned int scSensorConnection = 1;

static const unsigned int scOwnId = 1;          // own vehicle ID
static const double scOwnPreferredSpeed = 30.0; // m/s

// TTC and braking thresholds
static constexpr double scBrakingTTCThreshold = 2.0;  // seconds
static constexpr double scMaxBrake = -1.0;            // max braking accel

// Function declarations
int openPort(int &descriptor, int portNo, const char* serverAddr);
void initConnections();
void initConnection(Connection_t &conn);
void readConnection(Connection_t &conn, bool waitForMessage, bool verbose);
void parseRDBMessage(Connection_t &conn, RDB_MSG_t* msg);
void parseRDBMessageEntry(Connection_t &conn, const double &simTime, unsigned int simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr);
void handleRDBitem(const double &simTime, unsigned int simFrame, RDB_OBJECT_STATE_t &item, bool isExtended, bool isSensor, bool firstElement);
void sendDriverCtrl(int &sendSocket, const double &simTime, unsigned int simFrame);
double computeTTC(const RDB_OBJECT_STATE_t &own, const RDB_OBJECT_STATE_t &obj);
void usage();
void ValidateArgs(int argc, char **argv);


// === Helper: compute Time-To-Collision (TTC) ===
double computeTTC(const RDB_OBJECT_STATE_t &own, const RDB_OBJECT_STATE_t &obj)
{
    double dx = obj.base.pos.x - own.base.pos.x;
    double relVel = own.ext.speed.x - obj.ext.speed.x;

    if (relVel <= 0.0)
        return INFINITY;  // no collision if own speed is less or equal

    return dx / relVel;
}

void usage()
{
    printf("usage: client [-t:x] [-s:x] [-T:IP] [-S:IP]\n\n");
    printf("       -t:x     taskControl port\n");
    printf("       -s:x     sensor port\n");
    printf("       -T:IP    taskControl IPv4 address\n");
    printf("       -S:IP    sensor IPv4 address\n");
    exit(1);
}

void ValidateArgs(int argc, char **argv)
{
    for (int i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (argv[i][1])
            {
            case 't': // Remote port taskControl
                if (strlen(argv[i]) > 3)
                    sConnection[0].port = atoi(&argv[i][3]);
                break;
            case 's': // Remote port sensor
                if (strlen(argv[i]) > 3)
                    sConnection[1].port = atoi(&argv[i][3]);
                break;
            case 'T': // TC server
                if (strlen(argv[i]) > 3)
                    strcpy(sConnection[0].serverAddr, &argv[i][3]);
                break;
            case 'S': // sensor server
                if (strlen(argv[i]) > 3)
                    strcpy(sConnection[1].serverAddr, &argv[i][3]);
                break;
            default:
                usage();
                break;
            }
        }
    }
}

int main(int argc, char* argv[])
{
    bool sVerbose = false;

    // Initialize connections and buffers
    initConnections();

    // Parse the command line arguments
    ValidateArgs(argc, argv);

    // Clear objects initially
    memset(&mNearestObject, 0, sizeof(RDB_OBJECT_STATE_t));
    memset(&mOwnObject, 0, sizeof(RDB_OBJECT_STATE_t));

    // Open Task Control port
    if (!openPort(sConnection[0].desc, sConnection[0].port, sConnection[0].serverAddr))
        return -1;

    // Open Sensor port
    if (!openPort(sConnection[1].desc, sConnection[1].port, sConnection[1].serverAddr))
        return -1;

    // Main processing loop: receive, parse, compute, send
    for (;;)
    {
        readConnection(sConnection[0], false, sVerbose);
        readConnection(sConnection[1], false, sVerbose);

        usleep(1000); // 1 ms pause to ease CPU load
    }

    return 0;
}


void initConnections()
{
    memset(sConnection, 0, MAX_CONNECTIONS * sizeof(Connection_t));

    for (int i = 0; i < MAX_CONNECTIONS; i++)
    {
        initConnection(sConnection[i]);
        sConnection[i].id = i;
    }

    sConnection[0].port = DEFAULT_PORT_TC;
    sConnection[1].port = DEFAULT_PORT_SENSOR;
}


void initConnection(Connection_t &conn)
{
    strcpy(conn.serverAddr, "127.0.0.1");

    conn.desc = -1;
    conn.bufferSize = sizeof(RDB_MSG_t);
    conn.pData = (unsigned char*)calloc(1, conn.bufferSize);
    conn.bytesInBuffer = 0;
}


int openPort(int &descriptor, int portNo, const char* serverAddr)
{
    struct sockaddr_in server;
    struct hostent *host = nullptr;

    descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (descriptor == -1)
    {
        fprintf(stderr, "openPort: socket() failed: %s\n", strerror(errno));
        return 0;
    }

    int opt = 1;
    setsockopt(descriptor, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    server.sin_family = AF_INET;
    server.sin_port = htons(portNo);
    server.sin_addr.s_addr = inet_addr(serverAddr);

    if (server.sin_addr.s_addr == INADDR_NONE)
    {
        host = gethostbyname(serverAddr);
        if (!host)
        {
            fprintf(stderr, "openPort: unable to resolve server: %s\n", serverAddr);
            return 0;
        }
        memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
    }

    // Set socket non-blocking
    opt = 1;
    ioctl(descriptor, FIONBIO, &opt);

    // Non-blocking connect with select()
    int res = connect(descriptor, (struct sockaddr*)&server, sizeof(server));
    if (res < 0)
    {
        if (errno != EINPROGRESS)
        {
            fprintf(stderr, "connect() failed immediately: %s\n", strerror(errno));
            close(descriptor);
            return 0;
        }
    }

    fd_set wfds;
    struct timeval tv;
    FD_ZERO(&wfds);
    FD_SET(descriptor, &wfds);

    tv.tv_sec = 5;  // wait max 5 seconds
    tv.tv_usec = 0;

    res = select(descriptor + 1, NULL, &wfds, NULL, &tv);
    if (res <= 0)
    {
        fprintf(stderr, "connect() timeout or error\n");
        close(descriptor);
        return 0;
    }

    int err = 0;
    socklen_t len = sizeof(err);
    if (getsockopt(descriptor, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0)
    {
        fprintf(stderr, "connect() error after select: %s\n", strerror(err));
        close(descriptor);
        return 0;
    }

    // Set back to blocking or keep non-blocking as needed (here we set back to blocking)
    opt = 0;
    ioctl(descriptor, FIONBIO, &opt);

    fprintf(stderr, "Port %d connected to %s\n", portNo, serverAddr);
    return 1;
}


// Receive and buffer data, parse complete messages
void readConnection(Connection_t &conn, bool waitForMessage, bool verbose)
{
    static char* szBuffer = (char*)calloc(1, DEFAULT_BUFFER_SIZE);
    int ret = -1;

    bool bMsgComplete = false;

    if (verbose)
        fprintf(stderr, "readConnection: start reading connection %d\n", conn.id);

    do
    {
        ret = recv(conn.desc, szBuffer, DEFAULT_BUFFER_SIZE, 0);

        if (ret > 0)
        {
            // Grow buffer if necessary
            if ((conn.bytesInBuffer + ret) > conn.bufferSize)
            {
                unsigned char* newBuffer = (unsigned char*)realloc(conn.pData, conn.bytesInBuffer + ret);
                if (!newBuffer)
                {
                    fprintf(stderr, "readConnection: failed to realloc buffer\n");
                    exit(1);
                }
                conn.pData = newBuffer;
                conn.bufferSize = conn.bytesInBuffer + ret;
            }

            memcpy(conn.pData + conn.bytesInBuffer, szBuffer, ret);
            conn.bytesInBuffer += ret;

            // Check for complete message
            while (conn.bytesInBuffer >= sizeof(RDB_MSG_HDR_t))
            {
                RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*)conn.pData;

                if (hdr->magicNo != RDB_MAGIC_NO)
                {
                    fprintf(stderr, "readConnection: magicNo mismatch, discarding buffer\n");
                    conn.bytesInBuffer = 0;
                    break;
                }

                unsigned int fullMsgSize = hdr->headerSize + hdr->dataSize;
                if (conn.bytesInBuffer < fullMsgSize)
                    break; // incomplete message, wait for more data

                // Verbose print (optional)
                if (verbose)
                    Framework::RDBHandler::printMessage((RDB_MSG_t*)conn.pData, true);

                // Parse complete RDB message
                parseRDBMessage(conn, (RDB_MSG_t*)conn.pData);

                // Remove processed message from buffer
                memmove(conn.pData, conn.pData + fullMsgSize, conn.bytesInBuffer - fullMsgSize);
                conn.bytesInBuffer -= fullMsgSize;

                bMsgComplete = true;
            }
        }
        else if (ret == 0)
        {
            // Connection closed by peer, handle if needed
            fprintf(stderr, "readConnection: connection %d closed by peer\n", conn.id);
            close(conn.desc);
            conn.desc = -1;
            return;
        }
        else // ret < 0
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // No data available for non-blocking socket
                break;
            }
            else
            {
                fprintf(stderr, "readConnection: recv error on conn %d: %s\n", conn.id, strerror(errno));
                close(conn.desc);
                conn.desc = -1;
                return;
            }
        }

    } while ((ret > 0) && (!waitForMessage || bMsgComplete));

    if (verbose)
        fprintf(stderr, "readConnection: finished reading connection %d\n", conn.id);
}

// Parse a full RDB message and its entries
void parseRDBMessage(Connection_t &conn, RDB_MSG_t* msg)
{
    if (!msg || msg->hdr.dataSize == 0)
        return;

    RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)((char*)msg + msg->hdr.headerSize);
    uint32_t remainingBytes = msg->hdr.dataSize;

    while (remainingBytes)
    {
        parseRDBMessageEntry(conn, msg->hdr.simTime, msg->hdr.frameNo, entry);

        remainingBytes -= (entry->headerSize + entry->dataSize);

        if (remainingBytes)
            entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
    }
}

// Parse individual entries inside an RDB message
/*void parseRDBMessageEntry(Connection_t &conn, const double &simTime, unsigned int simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr)
{
    if (!entryHdr)
        return;
    if (entryHdr->pkgId == RDB_PKG_ID_OBJECT_STATE) {
    char* dataPtr = (char*)entryHdr + entryHdr->headerSize;
    int n = entryHdr->dataSize / entryHdr->elementSize;
    for (int i = 0; i < n; i++) {
        RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)(dataPtr + i * entryHdr->elementSize);
        fprintf(stderr, "[DEBUG] OBJECT_STATE: id=%d name=%s pos=(%.2f, %.2f, %.2f)\n",
                obj->base.id, obj->base.name,
                obj->base.pos.x, obj->base.pos.y, obj->base.pos.z);
    }
}

    int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

    if (!noElements)  // special entries
    {
        switch (entryHdr->pkgId)
        {
            case RDB_PKG_ID_START_OF_FRAME:
                fprintf(stderr, "parseRDBMessageEntry: conn %d: start of frame\n", conn.id);
                if (conn.id == scSensorConnection)
                    memset(&mNearestObject, 0, sizeof(RDB_OBJECT_STATE_t));
                break;

            case RDB_PKG_ID_END_OF_FRAME:
                fprintf(stderr, "parseRDBMessageEntry: conn %d: end of frame\n", conn.id);
                if (conn.id == scMainConnection)
                    sendDriverCtrl(conn.desc, simTime, simFrame);
                break;

            default:
                return;
        }
        return;
    }

    char* dataPtr = (char*)entryHdr + entryHdr->headerSize;
    bool firstElement = true;

    while (noElements--)
    {
        switch (entryHdr->pkgId)
        {
        case RDB_PKG_ID_OBJECT_STATE:
            if ((conn.id == scSensorConnection) && (entryHdr->flags & RDB_PKG_FLAG_EXTENDED))
            {
                handleRDBitem(simTime, simFrame, *(RDB_OBJECT_STATE_t*)dataPtr, true, true, firstElement);
                firstElement = false;
            }
            else if (conn.id == scMainConnection)
            {
                handleRDBitem(simTime, simFrame, *(RDB_OBJECT_STATE_t*)dataPtr, true, false, firstElement);
            }
            break;

        default:
            break;
        }

        dataPtr += entryHdr->elementSize;
    }
}*/
void parseRDBMessageEntry(Connection_t &conn,
                          const double &simTime,
                          unsigned int simFrame,
                          RDB_MSG_ENTRY_HDR_t* entryHdr)
{
    if (!entryHdr)
        return;

    int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

    // --- Handle special entries (start/end of frame etc.) ---
    if (!noElements)
    {
        switch (entryHdr->pkgId)
        {
            case RDB_PKG_ID_START_OF_FRAME:
                fprintf(stderr, "parseRDBMessageEntry: conn %d: start of frame\n", conn.id);
                if (conn.id == scSensorConnection)
                    memset(&mNearestObject, 0, sizeof(RDB_OBJECT_STATE_t));
                break;

            case RDB_PKG_ID_END_OF_FRAME:
                fprintf(stderr, "parseRDBMessageEntry: conn %d: end of frame\n", conn.id);
                if (conn.id == scMainConnection)
                    sendDriverCtrl(conn.desc, simTime, simFrame);
                break;

            default:
                return;
        }
        return;
    }

    // --- Iterate over all elements ---
    char* dataPtr = (char*)entryHdr + entryHdr->headerSize;
    while (noElements--)
    {
        switch (entryHdr->pkgId)
        {
            case RDB_PKG_ID_OBJECT_STATE:
            {
                RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)dataPtr;

                // Debug print
                fprintf(stderr, "[DEBUG] OBJECT_STATE: id=%d name=%s pos=(%.2f, %.2f, %.2f)\n",
                        obj->base.id, obj->base.name,
                        obj->base.pos.x, obj->base.pos.y, obj->base.pos.z);

                // Ego vs traffic
                if (obj->base.id == scOwnId) {
                    memcpy(&mOwnObject, obj, sizeof(RDB_OBJECT_STATE_t));
                } else {
                    double dy = fabs(obj->base.pos.y - mOwnObject.base.pos.y);
                    double dx = obj->base.pos.x - mOwnObject.base.pos.x;

                    // Check if vehicle is in front and roughly in lane
                    if (dy < 1.5 && dx > 0.0) {
                        if (mNearestObject.base.id == 0 ||
                            dx < (mNearestObject.base.pos.x - mOwnObject.base.pos.x)) {
                            memcpy(&mNearestObject, obj, sizeof(RDB_OBJECT_STATE_t));
                        }
                    }
                }
                break;
            }

            // other pkgIds can go here if you want to extend later
            default:
                break;
        }

        dataPtr += entryHdr->elementSize;
    }
}



// Update global ego and nearest object states and log relevant info
void handleRDBitem(const double &simTime, unsigned int simFrame, RDB_OBJECT_STATE_t &item, bool isExtended, bool isSensor, bool firstElement)
{
    double lateralThreshold =1.5; 
    if (item.base.id == scOwnId){

    
        memcpy(&mOwnObject, &item, sizeof(RDB_OBJECT_STATE_t));
    }
    /*else if (isSensor && firstElement){       
        double dy = fabs(item.base.pos.y - mOwnObject.base.pos.y);
        if (dy < lateralThreshold) {
            memcpy(&mNearestObject, &item, sizeof(RDB_OBJECT_STATE_t));
    }*/ //this part check only first element so commented out
      else if (isSensor) {
    double dy = fabs(item.base.pos.y - mOwnObject.base.pos.y);
    if (dy < lateralThreshold) {
        // Keep the nearest in longitudinal (x) direction
        if (mNearestObject.base.id == 0 ||
            item.base.pos.x < mNearestObject.base.pos.x) {
            memcpy(&mNearestObject, &item, sizeof(RDB_OBJECT_STATE_t));
        }
    }
}

    fprintf(stderr, "handleRDBitem: simTime=%.3lf simFrame=%u\n", simTime, simFrame);
    fprintf(stderr, "  object: %s id=%d pos=%.3lf %.3lf %.3lf speed=%.3lf %.3lf %.3lf\n",
            item.base.name, item.base.id,
            item.base.pos.x, item.base.pos.y, item.base.pos.z,
            item.ext.speed.x, item.ext.speed.y, item.ext.speed.z);
}


// Send driver control commands incorporating ACC and AEB logic
void sendDriverCtrl(int &sendSocket, const double &simTime, unsigned int simFrame)
{
    Framework::RDBHandler myHandler;
    myHandler.initMsg();

    RDB_DRIVER_CTRL_t* myDriver = (RDB_DRIVER_CTRL_t*)myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);
    if (!myDriver)
        return;

    bool haveSensorObject = (mNearestObject.base.id > 0);
    double ownSpeed = sqrt(mOwnObject.ext.speed.x * mOwnObject.ext.speed.x + mOwnObject.ext.speed.y * mOwnObject.ext.speed.y);

    double accelTgtDist = 0.0;
    double accelTgtSpeed = (scOwnPreferredSpeed - ownSpeed) / 2.0;  // ACC: speed tracking

    if (haveSensorObject)
    {
        if (mNearestObject.ext.speed.x < -1.0e-3)
            accelTgtSpeed = 2.0 * mNearestObject.ext.speed.x / 5.0;
        else
            accelTgtSpeed = 0.0;

        double tgtDist = ownSpeed * 2.0;
        if (tgtDist < 10.0)
            tgtDist = 10.0;

        accelTgtDist = (mNearestObject.base.pos.x - tgtDist) / 10.0;
    }

    // --- AEB logic ---
    double ttc = computeTTC(mOwnObject, mNearestObject);
    bool shouldBrake = (haveSensorObject && (ttc < scBrakingTTCThreshold));

    if (shouldBrake)
    {
        fprintf(stderr, "AEB activated! TTC = %.2fs (threshold %.2fs) - applying brake\n", ttc, scBrakingTTCThreshold);
        myDriver->accelTgt = scMaxBrake;
    }
    else
    {
        myDriver->accelTgt = accelTgtDist + accelTgtSpeed;
    }

    myDriver->playerId = scOwnId;
    myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_ADD_ON;

    int retVal = send(sendSocket, (const char*)myHandler.getMsg(), myHandler.getMsgTotalSize(), 0);

    if (retVal <= 0)
        fprintf(stderr, "sendDriverCtrl: failed to send driver control\n");
    else
        fprintf(stderr, "sendDriverCtrl: driver control sent (accel=%.3lf)\n", myDriver->accelTgt);
}
// void sendDriverCtrl(int &sendSocket, const double &simTime, unsigned int simFrame)
// {
//     Framework::RDBHandler myHandler;
//     myHandler.initMsg();

//     RDB_DRIVER_CTRL_t* myDriver = (RDB_DRIVER_CTRL_t*)myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);
//     if (!myDriver)
//         return;

//     bool haveSensorObject = (mNearestObject.base.id > 0);

//     // --- AEB logic only ---
//     double ttc = computeTTC(mOwnObject, mNearestObject);
//     bool shouldBrake = (haveSensorObject && (ttc < scBrakingTTCThreshold));

//     if (shouldBrake)
//     {
//         fprintf(stderr,
//                 "AEB activated! TTC = %.2fs (threshold %.2fs) - applying brake\n",
//                 ttc, scBrakingTTCThreshold);
//         myDriver->accelTgt = scMaxBrake; // full brake
//     }
//     else
//     {
//         myDriver->accelTgt = 0.0; // no acceleration or braking if no threat
//     }

//     myDriver->playerId = scOwnId;
//     myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_ADD_ON;

//     int retVal = send(sendSocket, (const char*)myHandler.getMsg(), myHandler.getMsgTotalSize(), 0);

//     if (retVal <= 0)
//         fprintf(stderr, "sendDriverCtrl: failed to send driver control\n");
//     else
//         fprintf(stderr, "sendDriverCtrl: driver control sent (accel=%.3lf)\n", myDriver->accelTgt);
// }


