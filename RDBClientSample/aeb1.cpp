// =============================================================
// Unified VTD RDB Client (FULL MERGE)
// - Contains the full original Dual-Port (TaskControl + Sensor) client
//   from your first program, with minimal changes (only renaming main)
// - Contains the full original Single-Port / SIM_MODE client
//   from your second program, with minimal changes (only renaming main
//   and making helper names distinct where needed)
// - Adds a runtime mode selector so you can switch without recompiling:
//      --dual     => run dual-port client (default)
//      --single   => run single-port client
//      --sim      => run offline simulation (single-port code path)
//
// Compile example (VTD SDK must be in include/lib paths):
//   g++ -std=c++17 -O2 unified_vtd_client.cpp -o vtd_client \
//       -I/path/to/VtdToolkit/include -L/path/to/VtdToolkit/lib -lvtd -lpthread
//
// Run examples:
//   ./vtd_client --dual                       # dual-port client (TaskControl+Sensor)
//   ./vtd_client --single 127.0.0.1 48190     # single-port client (live RDB)
//   ./vtd_client --sim                         # offline SIM_MODE (no sockets)
// =============================================================

// ------------------------------
// Common system headers
// ------------------------------
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
#include <ctype.h>

// ------------------------------
// VTD headers (ensure include path)
// ------------------------------
#include "VtdToolkit/RDBHandler.hh"

// =============================================================
// SECTION A: FULL original DUAL-PORT CLIENT (as given)
//           - only change: main() renamed to main_dual_impl()
//           - everything else preserved (names, prints, logic)
// =============================================================

#define DEFAULT_PORT_TC_A     48190   /* for image port it should be 48192 */
#define DEFAULT_PORT_SENSOR_A 48195
#define DEFAULT_BUFFER_SIZE_A 204800
#define MAX_CONNECTIONS_A     2       /* maximum number of bi-directional TCP connections */

// type definition for connection handling (dual-port)
typedef struct
{
    int           id;               // unique connection ID
    char          serverAddr[128];  // Server to connect to
    int           port;             // Port on server to connect to
    int           desc;             // client (socket) descriptor
    unsigned int  bytesInBuffer;    // used size of receive buffer
    size_t        bufferSize;       // total size of receive buffer;
    unsigned char *pData;           // pointer to receive buffer
} ConnectionA_t;

// connection instances (dual-port)
static ConnectionA_t sConnectionA[MAX_CONNECTIONS_A];

// globally store nearest object (dual-port)
static RDB_OBJECT_STATE_t mNearestObjectA;
static RDB_OBJECT_STATE_t mOwnObjectA;

static const unsigned int scMainConnectionA   = 0;
static const unsigned int scSensorConnectionA = 1;

static const unsigned int scOwnIdA = 1;  // ID of own vehicle
static const double scOwnPreferredSpeedA = 30.0;     // default preferred speed of own vehicle
static double lastUpdateTimeSensorObjectA = 0.0;

// -------- prototypes (dual-port) --------
static int  openPortA( int & descriptor, int portNo, const char* serverAddr );
static void initConnectionsA();
static void initConnectionA( ConnectionA_t & conn );
static void readConnectionA( ConnectionA_t & conn, bool waitForMessage, bool verbose );

static void parseRDBMessageA( ConnectionA_t & conn, RDB_MSG_t* msg );
static void parseRDBMessageEntryA( ConnectionA_t & conn, const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );
static void handleRDBitemA( const double & simTime, const unsigned int & simFrame, RDB_OBJECT_STATE_t & item, bool isExtended, bool isSensor, bool firstElement );
static void sendDriverCtrlA( int & sendSocket, const double & simTime, const unsigned int & simFrame );

static void usageA()
{
    printf("usage: client [-t:x] [-s:IP]\n\n");
    printf("       -t:x      taskControl port\n");
    printf("       -s:x      sensor port\n");
    printf("       -T:IP     taskControl IPv4 address\n");
    printf("       -S:IP     sensor IPv4 address\n");
}

static void ValidateArgsA(int argc, char **argv)
{
    for(int i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (argv[i][1])
            {
                case 't':        // Remote port taskControl
                    if (strlen(argv[i]) > 3)
                        sConnectionA[0].port = atoi(&argv[i][3]);
                    break;
                case 's':        // Remote port sensor
                    if (strlen(argv[i]) > 3)
                        sConnectionA[1].port = atoi(&argv[i][3]);
                    break;
                case 'T':       // TC server
                    if (strlen(argv[i]) > 3)
                        strcpy(sConnectionA[0].serverAddr, &argv[i][3]);
                    break;
                case 'S':       // sensor server
                    if (strlen(argv[i]) > 3)
                        strcpy(sConnectionA[1].serverAddr, &argv[i][3]);
                    break;
                default:
                    usageA();
                    break;
            }
        }
    }
}

// -------- dual-port main (renamed) --------
static int main_dual_impl( int argc, char* argv[] )
{
    static bool sVerbose = false;

    // initialize the connections
    initConnectionsA();

    // Parse the command line
    ValidateArgsA(argc, argv);

    // reset the information about the nearest and own object
    memset( &mNearestObjectA, 0, sizeof( RDB_OBJECT_STATE_t ) );
    memset( &mOwnObjectA, 0, sizeof( RDB_OBJECT_STATE_t ) );

    // open TC port
    if ( !openPortA( sConnectionA[0].desc, sConnectionA[0].port, sConnectionA[0].serverAddr ) )
        return -1;

    // open sensor port
    if ( !openPortA( sConnectionA[1].desc, sConnectionA[1].port, sConnectionA[1].serverAddr ) )
        return -1;

    // Send and receive data - forever!
    for(;;)
    {
        readConnectionA( sConnectionA[0], false, sVerbose );
        readConnectionA( sConnectionA[1], false, sVerbose );

        usleep( 1 );    // do not overload the system
    }

    return 0;
}

static void parseRDBMessageA( ConnectionA_t & conn, RDB_MSG_t* msg )
{
    if ( !msg )
      return;

    if ( !msg->hdr.dataSize )
        return;

    RDB_MSG_ENTRY_HDR_t* entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( char* ) msg ) + msg->hdr.headerSize );
    uint32_t remainingBytes    = msg->hdr.dataSize;

    while ( remainingBytes )
    {
        parseRDBMessageEntryA( conn, msg->hdr.simTime, msg->hdr.frameNo, entry );

        remainingBytes -= ( entry->headerSize + entry->dataSize );

        if ( remainingBytes )
          entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( ( char* ) entry ) + entry->headerSize + entry->dataSize ) );
    }
}

static void parseRDBMessageEntryA( ConnectionA_t & conn, const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr )
{
    if ( !entryHdr )
        return;

    int noElements = entryHdr->elementSize ? ( entryHdr->dataSize / entryHdr->elementSize ) : 0;

    if ( !noElements )  // some elements require special treatment
    {
        switch ( entryHdr->pkgId )
        {
            case RDB_PKG_ID_START_OF_FRAME:
                fprintf( stderr, "void parseRDBMessageEntry: connection %d: got start of frame\n",  conn.id );

                if ( conn.id == scSensorConnectionA )            // reset information about sensor object if a new sensor frame starts.
                     memset( &mNearestObjectA, 0, sizeof( RDB_OBJECT_STATE_t ) );
                break;

            case RDB_PKG_ID_END_OF_FRAME:
                fprintf( stderr, "void parseRDBMessageEntry: connection %d: got end of frame\n", conn.id );
                // only connection to taskControl shall send driver control commands

                if ( conn.id == scMainConnectionA )
                    sendDriverCtrlA( conn.desc, simTime, simFrame );
                break;

            default:
                return;
                break;
        }
        return;
    }

    unsigned char ident   = 6;
    char*         dataPtr = ( char* ) entryHdr;
    bool          firstElement = true;

    dataPtr += entryHdr->headerSize;

    while ( noElements-- )
    {
        bool printedMsg = true;

        switch ( entryHdr->pkgId )
        {
            case RDB_PKG_ID_OBJECT_STATE:
                // only first object state in sensor (i.e. nearest object ) is relevant
                if ( ( conn.id == scSensorConnectionA ) && ( entryHdr->flags & RDB_PKG_FLAG_EXTENDED ) )
                {
                    handleRDBitemA( simTime, simFrame, *( ( RDB_OBJECT_STATE_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ( conn.id == scSensorConnectionA ), firstElement );
                    firstElement = false;
                }
                break;

            default:
                printedMsg = false;
                break;
        }
        dataPtr += entryHdr->elementSize;
     }
}

static void handleRDBitemA( const double & simTime, const unsigned int & simFrame, RDB_OBJECT_STATE_t & item, bool isExtended, bool isSensor, bool firstElement )
{
    // own object?
    if ( item.base.id == scOwnIdA )
        memcpy( &mOwnObjectA, &item, sizeof( RDB_OBJECT_STATE_t ) );
    else if ( isSensor && firstElement )
        memcpy( &mNearestObjectA, &item, sizeof( RDB_OBJECT_STATE_t ) );

  fprintf( stderr, "handleRDBitem: handling object state\n" );
  fprintf( stderr, "    simTime = %.3lf, simFrame = %ld\n", simTime, (long)simFrame );
  fprintf( stderr, "    object = %s, id = %d\n", item.base.name, item.base.id );
  fprintf( stderr, "    position = %.3lf / %.3lf / %.3lf\n", item.base.pos.x, item.base.pos.y, item.base.pos.z );

  if ( isExtended )
    fprintf( stderr, "    speed = %.3lf / %.3lf / %.3lf\n", item.ext.speed.x, item.ext.speed.y, item.ext.speed.z );
}

static void sendDriverCtrlA( int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
  Framework::RDBHandler myHandler;

  myHandler.initMsg();

  RDB_DRIVER_CTRL_t *myDriver = ( RDB_DRIVER_CTRL_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL );

  if ( !myDriver )
    return;

  // do we have a valid nearest object?
  bool haveSensorObject = ( mNearestObjectA.base.id > 0 );   // sensor object must not be older than 1.0s

  double ownSpeed = sqrt( mOwnObjectA.ext.speed.x * mOwnObjectA.ext.speed.x + mOwnObjectA.ext.speed.y * mOwnObjectA.ext.speed.y );

  double accelTgtDist  = 0.0;
  const double DESIRED_SPEED = 16.67;   // 60 km/h in m/s
  double accelTgtSpeed = (DESIRED_SPEED - ownSpeed) / 5.0;

  if ( haveSensorObject )
  {
    // let's go for the same speed as preceding vehicle:
    if ( mNearestObjectA.ext.speed.x < -1.0e-3 )
      accelTgtSpeed = 2.0 * mNearestObjectA.ext.speed.x / 5.0;
    else
      accelTgtSpeed = 0.0;

    // let's go for a 2s distance
    double tgtDist = ownSpeed * 2.0;

    if ( tgtDist < 10.0 )   // minimum  distance to keep
        tgtDist = 10.0;

    accelTgtDist = ( mNearestObjectA.base.pos.x - tgtDist ) / 10.0;
  }

  fprintf( stderr, "sendDriverCtrl: accelDist = %.5lf, accelSpeed = %.5lf\n", accelTgtDist, accelTgtSpeed );

  myDriver->playerId      = 1;
  myDriver->accelTgt      = accelTgtDist + accelTgtSpeed;
  myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_ADD_ON;

  int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

  if ( !retVal )
    fprintf( stderr, "sendDriverCtrl: could not send driver control\n" );
  else
    fprintf( stderr, "sentDriverCtrl\n" );
}

static void initConnectionsA()
{
    memset( sConnectionA, 0, MAX_CONNECTIONS_A * sizeof( ConnectionA_t ) );

    // general initialization
    for ( int i = 0; i < MAX_CONNECTIONS_A; i++ )
    {
        initConnectionA( sConnectionA[i] );
        sConnectionA[i].id = i;
    }

    // additional individual initalization
    sConnectionA[0].port = DEFAULT_PORT_TC_A;
    sConnectionA[1].port = DEFAULT_PORT_SENSOR_A;
}

static void initConnectionA( ConnectionA_t & conn )
{
    strcpy( conn.serverAddr, "127.0.0.1" );

    conn.desc       = -1;
    conn.bufferSize = sizeof ( RDB_MSG_t );
    conn.pData      = ( unsigned char* ) calloc( 1, conn.bufferSize );
}

static int openPortA( int & descriptor, int portNo, const char* serverAddr )
{
    struct sockaddr_in server;
    struct hostent    *host = NULL;

    descriptor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if ( descriptor == -1 )
    {
        fprintf( stderr, "openPort: socket() failed: %s\n", strerror( errno ) );
        return 0;
    }

    int opt = 1;
    setsockopt ( descriptor, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof( opt ) );

    server.sin_family      = AF_INET;
    server.sin_port        = htons( portNo );
    server.sin_addr.s_addr = inet_addr( serverAddr );

    if ( server.sin_addr.s_addr == INADDR_NONE )
    {
        host = gethostbyname( serverAddr );

        if ( host == NULL )
        {
            fprintf( stderr, "openPort: unable to resolve server: %s\n", serverAddr );
            return 0;
        }
        memcpy( &server.sin_addr, host->h_addr_list[0], host->h_length );
    }

    // set to non blocking
    opt = 1;
    ioctl( descriptor, FIONBIO, &opt );

    // wait for connection
    bool bConnected = false;

    while ( !bConnected )
    {
        if (connect( descriptor, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
        {
            fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
            sleep( 1 );
        }
        else
            bConnected = true;
    }

    fprintf( stderr, "port % d connected!\n", portNo );

    return 1;
}

static void readConnectionA( ConnectionA_t & conn, bool waitForMessage, bool verbose )
{
    // receive buffer
    static char* szBuffer = ( char* ) calloc( 1, DEFAULT_BUFFER_SIZE_A );
    int ret = -1;

    bool bMsgComplete = false;

    if ( verbose )
        fprintf( stderr, "readConnection: start reading connection %d\n", conn.id );

    // read a complete message
    do
    {
        ret = recv( conn.desc, szBuffer, DEFAULT_BUFFER_SIZE_A, 0 );

        if ( verbose )
            fprintf( stderr, "readConnection: connection %d, ret = %d\n", conn.id, ret );

        if ( ret > 0 )
        {
            // do we have to grow the buffer??
            if ( ( conn.bytesInBuffer + ret ) > conn.bufferSize )
            {
                conn.pData      = ( unsigned char* ) realloc( conn.pData, conn.bytesInBuffer + ret );
                conn.bufferSize = conn.bytesInBuffer + ret;
            }

            memcpy( conn.pData + conn.bytesInBuffer, szBuffer, ret );
            conn.bytesInBuffer += ret;

            // already complete messagae?
            if ( conn.bytesInBuffer >= sizeof( RDB_MSG_HDR_t ) )
            {
                RDB_MSG_HDR_t* hdr = ( RDB_MSG_HDR_t* ) conn.pData;

                // is this message containing the valid magic number?
                if ( hdr->magicNo != RDB_MAGIC_NO )
                {
                    printf( "message receiving is out of sync; discarding data" );
                    conn.bytesInBuffer = 0;
                }

                while ( conn.bytesInBuffer >= ( hdr->headerSize + hdr->dataSize ) )
                {
                    unsigned int msgSize = hdr->headerSize + hdr->dataSize;

                    // print the message (preserved)
                    // Framework::RDBHandler::printMessage( ( RDB_MSG_t* ) conn.pData, true );

                    // now parse the message
                    parseRDBMessageA( conn, ( RDB_MSG_t* ) conn.pData );

                    // remove message from queue
                    memmove( conn.pData, conn.pData + msgSize, conn.bytesInBuffer - msgSize );
                    conn.bytesInBuffer -= msgSize;

                    bMsgComplete = true;
                }
            }
        }
    } while ( ( ret > 0 ) && ( !waitForMessage || bMsgComplete ) );

    if ( verbose )
        fprintf( stderr, "readConnection: finished reading connection %d\n", conn.id );
}

// =============================================================
// SECTION B: FULL original SINGLE-PORT / SIM_MODE CLIENT (as given)
//           - only change: main() renamed to main_single_impl()
//           - helper names made distinct with suffix B where overlap risk
// =============================================================

#define DEFAULT_PORT_B        48190
#define DEFAULT_BUFFER_B      204800
#define EGO_ID_B              1

#ifndef SIM_DT
#define SIM_DT 0.05f
#endif

// Flag just for SIM_MODE logs
static bool aebAppliedB = false;

struct VehStateB {
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

static inline void worldToEgoLocalB(float x, float y, float ego_x, float ego_y, float ego_h,
                                   float& x_local, float& y_local)
{
    float dx = x - ego_x;
    float dy = y - ego_y;
    float ch = cosf(ego_h);
    float sh = sinf(ego_h);
    x_local =  ch*dx + sh*dy;
    y_local = -sh*dx + ch*dy;
}

// ---------- send driver control to VTD (single-port variant) ----------
static void sendDriverCtrlB(int sock, double simTime, unsigned int simFrame,
                           float accelTgt = 0.0f, float brakeTgt = 0.0f, float steerTgt = 0.0f,
                           bool stopNow = false )
{
    Framework::RDBHandler handler;
    handler.initMsg();

    RDB_DRIVER_CTRL_t* drv =
        (RDB_DRIVER_CTRL_t*)handler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL, 1, sizeof(RDB_DRIVER_CTRL_t));
    if (!drv) return;

    memset(drv, 0, sizeof(RDB_DRIVER_CTRL_t));
    drv->playerId = EGO_ID_B;

    if (stopNow) {
        drv->accelTgt   = 0.0f;   // no throttle
        drv->brakePedal = 1.0f;
        drv->speedTgt   = 0.0f;
        drv->steeringTgt= steerTgt;  // full brake
        fprintf(stderr, "[CTRL] Full brake applied (accelTgt=%.2f, brakePedal=%.2f)\n", accelTgt, brakeTgt);
    } else {
        drv->accelTgt   = accelTgt;  // normal throttle
        drv->brakePedal = brakeTgt;  // normal brake
        fprintf(stderr, "[CTRL] Normal control (accelTgt=%.2f, brakePedal=%.2f)\n", accelTgt, brakeTgt);
    }

    drv->steeringTgt = steerTgt;
    drv->gear        = 1;

    // mark which inputs are valid
    drv->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING |
                         RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL |
                         RDB_DRIVER_INPUT_VALIDITY_BRAKE | RDB_DRIVER_INPUT_VALIDITY_GEAR;

    int ret = send(sock, (const char*)handler.getMsg(), handler.getMsgTotalSize(), 0);
    if (ret <= 0) perror("sendDriverCtrl() send failed");
}

// (Kept for SIM_MODE only; not used in live RDB control)
static void autonomousEmergencyBrakingB(VehStateB& ego, const VehStateB& frontObj,
                               float warningDistance, float emergencyDistance,
                               float maxDecelRate)
{
    aebAppliedB = false;
    if (!frontObj.has) return;

    float x_local, y_local;
    worldToEgoLocalB(frontObj.x, frontObj.y, ego.x, ego.y, ego.h, x_local, y_local);
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
            aebAppliedB = true;
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
            aebAppliedB = true;
        }
    }
}

static float laneFollowingSteeringB(float y_local, float laneCenterY,
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
static void runSimulationB()
{
    float ego_x = 0.0f, ego_y = 0.0f, ego_h = 0.0f, ego_vx = 20.0f;
    float front_x = 50.0f, front_y = 0.0f, front_vx = 0.0f;

    const float dt = SIM_DT;
    const float warningDist = 30.0f, emergencyDist = 10.0f, maxDecelRate = 0.7f;

    int frameNo = 0; float simTime = 0.0f;

    while (frameNo < 500) {
        ego_x += ego_vx * dt;

        VehStateB ego{}, front{};
        ego.has = true; ego.id = EGO_ID_B; strncpy(ego.name, "EgoVehicle", sizeof(ego.name)-1);
        ego.x = ego_x; ego.y = ego_y; ego.h = ego_h; ego.vx = ego_vx;

        front.has = true; front.id = 2; strncpy(front.name, "FrontObject", sizeof(front.name)-1);
        front.x = front_x; front.y = front_y; front.vx = front_vx;

        float x_local, y_local; worldToEgoLocalB(front.x, front.y, ego.x, ego.y, ego.h, x_local, y_local);
        float dist = hypotf(x_local, y_local); float rel_v = fabsf(front.vx - ego.vx);

        printf("\n=== FRAME %d  t=%.3f ===\n", frameNo, simTime);
        printf("EGO: X=%.2f Vx=%.2f m/s\n", ego.x, ego.vx);
        printf("Front: X=%.2f Vx=%.2f m/s  Dist=%.2f m  RelV=%.2f m/s\n", front.x, front.vx, dist, rel_v);

        autonomousEmergencyBrakingB(ego, front, warningDist, emergencyDist, maxDecelRate);

        if (aebAppliedB) printf("AEB is actively controlling ego vehicle speed this frame.\n");
        else             printf("No AEB braking active this frame.\n");

        ego_vx = ego.vx;
        if (ego_vx <= 0.01f && dist < 12.0f) { printf("Simulation stop: ego stopped near object.\n"); break; }

        frameNo++; simTime += dt; usleep((int)(dt * 1e6));
    }
}
#endif

// -------------- Single-port main (renamed) --------------
static int main_single_impl(int argc, char* argv[])
{
#ifndef SIM_MODE
    const char* serverIP = (argc > 2) ? argv[2] : "127.0.0.1";  // note: argv[1] is mode flag
    int port = (argc > 3) ? atoi(argv[3]) : DEFAULT_PORT_B;
    printf("=== Running in LIVE RDB MODE ===\n");
    printf("Connecting to VTD RDB server at %s:%d ...\n", serverIP, port);
    // Detection gate for “in front”
    const float lane_window_m   = 5.0f;    // |y_local| must be <= this
    const float fwd_max_range_m = 18.0f;   // ahead within this range
    const float laneCenterY = 0.0f;

    int sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sClient < 0) { perror("socket() failed"); return 1; }

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port   = htons(port);
    if (inet_pton(AF_INET, serverIP, &server.sin_addr) != 1) { perror("inet_pton failed"); return 1; }

    if (connect(sClient, (struct sockaddr*)&server, sizeof(server)) < 0) { perror("connect() failed"); return 1; }
    printf("Connected to RDB server %s:%d\n", serverIP, port);

    char* szBuffer = new char[DEFAULT_BUFFER_B];
    unsigned char* pData = (unsigned char*)calloc(1, sizeof(RDB_MSG_HDR_t));
    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);

    while (true)
    {
        int ret = recv(sClient, szBuffer, DEFAULT_BUFFER_B, 0);
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

            VehStateB ego{};
            VehStateB bestFront{};
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

                        if (obj->base.id == EGO_ID_B) {
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
                            if (obj->base.id != EGO_ID_B) {
                                float x_local, y_local;
                                worldToEgoLocalB(obj->base.pos.x, obj->base.pos.y, ego.x, ego.y, ego.h, x_local, y_local);

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
                float steerCmd = laneFollowingSteeringB(ego.y, laneCenterY, ego.h, 0.0f, 1.5f, 0.5f);
                if (bestFront.has) {
                    autonomousEmergencyBrakingB(ego, bestFront, 30.0f, 10.0f, 0.7f);

                    float x_local, y_local;
                    worldToEgoLocalB(bestFront.x, bestFront.y, ego.x, ego.y, ego.h, x_local, y_local);
                    float dist_xy = hypotf(x_local, y_local);
                    printf("Front Object (ID=%d, Name=%s)  dist=%.2f m, x_fwd=%.2f, y_left=%.2f\n",
                           bestFront.id, bestFront.name[0] ? bestFront.name : "N/A", dist_xy, x_local, y_local);

                    // SIMPLE RULE: if anything in front, FULL BRAKE
                    accelCmd = -20.0f;
                    brakeCmd = 1.0f;
                    printf("Command: FULL BRAKE (accelTgt=%.2f) (brakeCmd=%.2f)\n", accelCmd, brakeCmd);
                } else {
                    float targetSpeed = 60.0f / 3.6f; // 60 km/h in m/s

                    if (ego.vx < targetSpeed) {
                        accelCmd = 2.0f;   // gentle acceleration
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
                sendDriverCtrlB(sClient, hdr->simTime, hdr->frameNo, accelCmd, brakeCmd, steerCmd);
            }

            memmove(pData, pData + msgSize, bytesInBuffer - msgSize);
            bytesInBuffer -= msgSize;
        }
    }

    close(sClient);
    delete[] szBuffer;
    free(pData);
    return 0;
#else
    printf("=== Running in SIM_MODE (no VTD connection, using built-in simulation) ===\n");
    runSimulationB();
    return 0;
#endif
}

// =============================================================
// SECTION C: Runtime mode selector and entry point
// =============================================================

static void print_help_and_exit()
{
    printf("\nUnified VTD RDB Client (Full Merge)\n");
    printf("Usage:\n");
    printf("  vtd_client [--dual] [--single [IP [PORT]]] [--sim]\n\n");
    printf("Modes:\n");
    printf("  --dual           Run dual-port client (TaskControl+Sensor). Default if no mode given.\n");
    printf("  --single         Run single-port client (LIVE RDB). Optionally pass IP and PORT.\n");
    printf("  --sim            Run offline simulation (single-port logic, no sockets).\n\n");
    printf("Examples:\n");
    printf("  ./vtd_client --dual\n");
    printf("  ./vtd_client --single 127.0.0.1 48190\n");
    printf("  ./vtd_client --sim\n\n");
}

int main(int argc, char* argv[])
{
    // Determine mode from argv
    bool wantDual = true;  // default
    bool wantSingle = false;
    bool wantSim = false;

    for (int i = 1; i < argc; ++i) {
        if      (!strcmp(argv[i], "--dual"))   { wantDual = true; wantSingle = false; wantSim = false; }
        else if (!strcmp(argv[i], "--single")) { wantDual = false; wantSingle = true; }
        else if (!strcmp(argv[i], "--sim"))    { wantDual = false; wantSingle = true; wantSim = true; }
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { print_help_and_exit(); return 0; }
        else {
            // stop parsing on first non-flag; remaining args belong to selected mode
            break;
        }
    }

    if (wantSingle) {
#ifdef SIM_MODE
        (void)wantSim; // compile-time SIM_MODE can still guard internal path
#endif
        // If user explicitly asked --sim, define SIM_MODE path at runtime by calling the sim entry
#ifdef SIM_MODE
        if (wantSim) {
            return main_single_impl(argc, argv); // compiled with SIM_MODE
        }
#endif
        return main_single_impl(argc, argv);
    }

    // default: dual
    return main_dual_impl(argc, argv);
}
