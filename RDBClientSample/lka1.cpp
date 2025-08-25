// LKA-only VTD client - updated for VTD 2025.1 lane info struct

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

#define DEFAULT_PORT_TC     48190
#define DEFAULT_PORT_SENSOR 48195
#define DEFAULT_BUFFER_SIZE 204800
#define MAX_CONNECTIONS     2

typedef struct
{
    int           id;
    char          serverAddr[128];
    int           port;
    int           desc;
    unsigned int  bytesInBuffer;
    size_t        bufferSize;
    unsigned char *pData;
} Connection_t;

Connection_t sConnection[MAX_CONNECTIONS];

// Globals for LKA
double mLaneOffset = 0.0;       // Lateral offset, not available from current struct, keep zero
double mLaneHeadingError = 0.0; // Use horizontal curvature as proxy for heading error

static const unsigned int scMainConnection   = 0; // TaskControl
static const unsigned int scSensorConnection = 1; // Sensor data
void parseRDBMessage(Connection_t &conn, RDB_MSG_t* msg);
void parseRDBMessageEntry(Connection_t &conn, const double &simTime, const unsigned int &simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr);
void sendDriverCtrl(int &sendSocket, const double &simTime, const unsigned int &simFrame);

// Then your existing code (including usage(), ValidateArgs(), main(), etc.)



void usage()
{
    printf("usage: client [-t:x] [-s:x] [-T:IP] [-S:IP]\n");
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
                case 't':
                    if (strlen(argv[i]) > 3) sConnection[0].port = atoi(&argv[i][3]);
                    break;
                case 's':
                    if (strlen(argv[i]) > 3) sConnection[1].port = atoi(&argv[i][3]);
                    break;
                case 'T':
                    if (strlen(argv[i]) > 3) strcpy(sConnection[0].serverAddr, &argv[i][3]);
                    break;
                case 'S':
                    if (strlen(argv[i]) > 3) strcpy(sConnection[1].serverAddr, &argv[i][3]);
                    break;
                default:
                    usage();
            }
        }
    }
}

int openPort(int & descriptor, int portNo, const char* serverAddr)
{
    struct sockaddr_in server;
    struct hostent *host = NULL;

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

    opt = 1;
    ioctl(descriptor, FIONBIO, &opt);

    bool bConnected = false;
    while (!bConnected)
    {
        if (connect(descriptor, (struct sockaddr*)&server, sizeof(server)) == -1)
        {
            fprintf(stderr, "connect() failed: %s\n", strerror(errno));
            sleep(1);
        }
        else
            bConnected = true;
    }

    fprintf(stderr, "port %d connected!\n", portNo);
    return 1;
}

void initConnection(Connection_t &conn)
{
    strcpy(conn.serverAddr, "127.0.0.1");
    conn.desc = -1;
    conn.bufferSize = sizeof(RDB_MSG_t);
    conn.pData = (unsigned char*) calloc(1, conn.bufferSize);
    conn.bytesInBuffer = 0;
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

void parseRDBMessage(Connection_t &conn, RDB_MSG_t* msg)
{
    if (!msg || !msg->hdr.dataSize) return;

    RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*) ((char*)msg + msg->hdr.headerSize);
    uint32_t remainingBytes = msg->hdr.dataSize;

    while (remainingBytes)
    {
        parseRDBMessageEntry(conn, msg->hdr.simTime, msg->hdr.frameNo, entry);
        remainingBytes -= (entry->headerSize + entry->dataSize);
        if (remainingBytes)
            entry = (RDB_MSG_ENTRY_HDR_t*) ((char*)entry + entry->headerSize + entry->dataSize);
    }
}

void parseRDBMessageEntry(Connection_t &conn, const double &simTime, const unsigned int &simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr)
{
    if (!entryHdr) return;
    int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

    if (!noElements)
    {
        switch (entryHdr->pkgId)
        {
            case RDB_PKG_ID_END_OF_FRAME:
                if (conn.id == scMainConnection)
                    sendDriverCtrl(conn.desc, simTime, simFrame);
                break;
        }
        return;
    }

    char* dataPtr = (char*) entryHdr + entryHdr->headerSize;
    bool firstElement = true;

    while (noElements--)
    {
        switch (entryHdr->pkgId)
        {
            case RDB_PKG_ID_LANE_INFO:
            {
                if (conn.id == scSensorConnection && firstElement)
                {
                    RDB_LANE_INFO_t* laneInfo = (RDB_LANE_INFO_t*) dataPtr;

                    // Use horizontal curvature as heading error proxy
                    mLaneHeadingError = laneInfo->curvHor;

                    // Lateral offset is not provided here; default to zero or combine with other data if available
                    mLaneOffset = 0.0;

                    fprintf(stderr, "LaneID=%d, curvHor=%.6f, width=%.2f\n",
                            laneInfo->id, laneInfo->curvHor, laneInfo->width);
                }
                firstElement = false;
                break;
            }
        }
        dataPtr += entryHdr->elementSize;
    }
}

oid sendDriverCtrl(int sock, double simTime, uint32_t frameNo, float accel, float brake) {
    char buffer[sizeof(RDB_MSG_HDR_t) + sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_DRIVER_CTRL_t)];
    memset(buffer, 0, sizeof(buffer));

    RDB_MSG_HDR_t* msg = (RDB_MSG_HDR_t*) buffer;
    msg->magicNo   = RDB_MAGIC_NO;
    msg->version   = RDB_VERSION;
    msg->simTime   = simTime;
    msg->frameNo   = frameNo;
    msg->headerSize= sizeof(RDB_MSG_HDR_t);
    msg->dataSize  = sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_DRIVER_CTRL_t);

    RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)(buffer + msg->headerSize);
    entry->headerSize   = sizeof(RDB_MSG_ENTRY_HDR_t);
    entry->dataSize     = sizeof(RDB_DRIVER_CTRL_t);
    entry->pkgId        = RDB_PKG_ID_DRIVER_CTRL;
    entry->elementSize  = sizeof(RDB_DRIVER_CTRL_t);
    entry->elementCount = 1;

    RDB_DRIVER_CTRL_t* ctrl = (RDB_DRIVER_CTRL_t*)(buffer + msg->headerSize + entry->headerSize);
    ctrl->playerId = egoId;   // <-- use your ego player ID
    ctrl->accelTgt = accel;
    ctrl->brakeTgt = brake;

    send(sock, buffer, sizeof(buffer), 0);
}
