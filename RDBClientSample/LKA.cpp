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
#include "VtdToolkit/RDBHandler.hh"
#include <algorithm>
 
 
 
 
 
#define DEFAULT_PORT        48190   /* for image port it should be 48192 */
#define DEFAULT_BUFFER      204800
 
char  szServer[128];             // Server to connect to
int   iPort     = DEFAULT_PORT;  // Port on server to connect to
int   sClient;
 
// Controller gain constants (tune as needed for smoother response)
const double K_LAT = 2.5;
const double K_HEAD = 1.5;
 
// In Utils.h
 
 
 
// function prototypes
void parseRDBMessage( RDB_MSG_t* msg, bool & isImage );
void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );
void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_OBJECT_STATE_t & item, bool isExtended );
void sendDriverCtrl( int & sendSocket, const double & simTime, const unsigned int & simFrame );
void handleRoadMarking(const RDB_ROADMARK_t& roadmark);
void sendLKAControl(int sendSocket, const double & simTime, const unsigned int & simFrame, double steeringTgt);
 
//
// Function: usage:
//
// Description:
//    Print usage information and exit
//
 
// // Core Lane Centering Control
// void laneCenteringControl(const RDB_OBJECT_STATE_t &ego, double &steerOut) {
//     // Centerline is assumed y=0; positive y: drift right, negative: drift left.
//     double lateralOffset = ego.base.pos.y;
 
//     // Heading should be relative to road tangent (assuming desired = 0)
//     double headingError = ego.base.pos.h;
 
//     // Unwrap heading error to [-pi, pi] if necessary
//     while (headingError > M_PI) headingError -= 2*M_PI;
//     while (headingError < -M_PI) headingError += 2*M_PI;
 
//     // PD control law
//     steerOut = -(K_LAT * lateralOffset + K_HEAD * headingError);
 
//     // Clamp steering command for safety
//     steerOut = std::clamp(steerOut, -1.5, 1.5);
 
 
//     fprintf(stderr, "[LKA] Offset=%.3f, HeadingErr=%.3f, SteerCmd=%.3f\n",
//             lateralOffset, headingError, steerOut);
// }
 
 
void usage()
{
    printf("usage: client [-p:x] [-s:IP]\n\n");
    printf("       -p:x      Remote port to send to\n");
    printf("       -s:IP     Server's IP address or hostname\n");
    exit(1);
}
 
//
// Function: ValidateArgs
//
// Description:
//    Parse the command line arguments, and set some global flags
//    to indicate what actions to perform
//
void ValidateArgs(int argc, char **argv)
{
    int i;
    
    strcpy( szServer, "127.0.0.1" );
 
    for(i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (tolower(argv[i][1]))
            {
                case 'p':        // Remote port
                    if (strlen(argv[i]) > 3)
                        iPort = atoi(&argv[i][3]);
                    break;
                case 's':       // Server
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
 
struct LaneMarkingInfo {
    double laneWidth = 3.5;  // default width in meters
    std::string leftType = "unknown";
    std::string rightType = "unknown";
};
LaneMarkingInfo gLaneInfo;
 
 
int main(int argc, char* argv[])
{
    char* szBuffer = new char[DEFAULT_BUFFER];  // allocate on heap
    int           ret;
 
    struct sockaddr_in server;
    struct hostent    *host = NULL;
 
    static bool sSendData    = false;
    static bool sVerbose     = true;
    static bool sSendTrigger = false;
 
    // Parse the command line
    //
    ValidateArgs(argc, argv);
    
    //
    // Create the socket, and attempt to connect to the server
    //
    sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    
    if ( sClient == -1 )
    {
        fprintf( stderr, "socket() failed: %s\n", strerror( errno ) );
        return 1;
    }
    
    int opt = 1;
    setsockopt ( sClient, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof( opt ) );
 
    server.sin_family      = AF_INET;
    server.sin_port        = htons(iPort);
    server.sin_addr.s_addr = inet_addr(szServer);
    
    //
    // If the supplied server address wasn't in the form
    // "aaa.bbb.ccc.ddd" it's a hostname, so try to resolve it
    //
    if ( server.sin_addr.s_addr == INADDR_NONE )
    {
        host = gethostbyname(szServer);
        if ( host == NULL )
        {
            fprintf( stderr, "Unable to resolve server: %s\n", szServer );
            return 1;
        }
        memcpy( &server.sin_addr, host->h_addr_list[0], host->h_length );
    }
    // wait for connection
    bool bConnected = false;
 
    while ( !bConnected )
    {
        if (connect( sClient, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
        {
            fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
            sleep( 1 );
        }
        else
            bConnected = true;
    }
 
    fprintf( stderr, "connected!\n" );
    
    unsigned int  bytesInBuffer = 0;
    size_t        bufferSize    = sizeof( RDB_MSG_HDR_t );
    unsigned int  count         = 0;
    unsigned char *pData        = ( unsigned char* ) calloc( 1, bufferSize );
 
    // Send and receive data - forever!
    //
    for(;;)
    {
        bool bMsgComplete = false;
 
        // read a complete message
        while ( !bMsgComplete )
        {
            //if ( sSendTrigger && !( count++ % 1000 ) )
            //  sendTrigger( sClient, 0, 0 );
 
            ret = recv( sClient, szBuffer, DEFAULT_BUFFER, 0 );
 
            if ( ret == -1 )
            {
                printf( "recv() failed: %s\n", strerror( errno ) );
                break;
            }
 
            if ( ret != 0 )
            {
                // do we have to grow the buffer??
                if ( ( bytesInBuffer + ret ) > bufferSize )
                {
                    pData      = ( unsigned char* ) realloc( pData, bytesInBuffer + ret );
                    bufferSize = bytesInBuffer + ret;
                }
 
                memcpy( pData + bytesInBuffer, szBuffer, ret );
                bytesInBuffer += ret;
 
                // already complete messagae?
                if ( bytesInBuffer >= sizeof( RDB_MSG_HDR_t ) )
                {
                    RDB_MSG_HDR_t* hdr = ( RDB_MSG_HDR_t* ) pData;
 
                    // is this message containing the valid magic number?
                    if ( hdr->magicNo != RDB_MAGIC_NO )
                    {
                        printf( "message receiving is out of sync; discarding data" );
                        bytesInBuffer = 0;
                    }
 
                    while ( bytesInBuffer >= ( hdr->headerSize + hdr->dataSize ) )
                    {
                        unsigned int msgSize = hdr->headerSize + hdr->dataSize;
                        bool         isImage = false;
                        
                        // print the message
                        //if ( sVerbose )
                        //    Framework::RDBHandler::printMessage( ( RDB_MSG_t* ) pData, true );
                        
                        // now parse the message    
                        parseRDBMessage( ( RDB_MSG_t* ) pData, isImage );
 
                        // remove message from queue
                        memmove( pData, pData + msgSize, bytesInBuffer - msgSize );
                        bytesInBuffer -= msgSize;
 
                        bMsgComplete = true;
                    }
                }
            }
        }
        // do some other stuff before returning to network reading
    }
    ::close(sClient);
 
    return 0;
}
 
void parseRDBMessage( RDB_MSG_t* msg, bool & isImage )
{
    if ( !msg )
      return;
 
    if ( !msg->hdr.dataSize )
        return;
    
    RDB_MSG_ENTRY_HDR_t* entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( char* ) msg ) + msg->hdr.headerSize );
    uint32_t remainingBytes    = msg->hdr.dataSize;
        
    while ( remainingBytes )
    {
        parseRDBMessageEntry( msg->hdr.simTime, msg->hdr.frameNo, entry );
 
        isImage |= ( entry->pkgId == RDB_PKG_ID_IMAGE );
 
        remainingBytes -= ( entry->headerSize + entry->dataSize );
        
        if ( remainingBytes )
          entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( ( char* ) entry ) + entry->headerSize + entry->dataSize ) );
    }
}
 
void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr )
{
    if ( !entryHdr )
        return;
    
    int noElements = entryHdr->elementSize ? ( entryHdr->dataSize / entryHdr->elementSize ) : 0;
    
    if ( !noElements )  // some elements require special treatment
    {
        switch ( entryHdr->pkgId )
        {
            case RDB_PKG_ID_START_OF_FRAME:
                fprintf( stderr, "void parseRDBMessageEntry: got start of frame\n" );
                break;
                
            case RDB_PKG_ID_END_OF_FRAME:
                fprintf( stderr, "void parseRDBMessageEntry: got end of frame\n" );
 
        sendDriverCtrl( sClient, simTime, simFrame );
                break;
                
            default:
                return;
                break;
        }
        return;
    }
 
    unsigned char ident   = 6;
    char*         dataPtr = ( char* ) entryHdr;
        
    dataPtr += entryHdr->headerSize;
        
    while ( noElements-- )
    {
        bool printedMsg = true;
            
        switch ( entryHdr->pkgId )
        {
 
         case RDB_PKG_ID_ROADMARK:
         {
            RDB_ROADMARK_t* roadMark = reinterpret_cast<RDB_ROADMARK_t*>(dataPtr);
            handleRoadMarking(*roadMark);
            break;
         }
 
/*
            case RDB_PKG_ID_COORD_SYSTEM:
                print( *( ( RDB_COORD_SYSTEM_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_COORD:
                print( *( ( RDB_COORD_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_ROAD_POS:
                print( *( ( RDB_ROAD_POS_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_LANE_INFO:
                print( *( ( RDB_LANE_INFO_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_ROADMARK:
                print( *( ( RDB_ROADMARK_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_OBJECT_CFG:
                print( *( ( RDB_OBJECT_CFG_t* ) dataPtr ), ident );
                break;
*/                    
            case RDB_PKG_ID_OBJECT_STATE:
                handleRDBitem( simTime, simFrame, *( ( RDB_OBJECT_STATE_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED );
                break;
                    
/*            case RDB_PKG_ID_VEHICLE_SYSTEMS:
                print( *( ( RDB_VEHICLE_SYSTEMS_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_VEHICLE_SETUP:
                print( *( ( RDB_VEHICLE_SETUP_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_ENGINE:
                print( *( ( RDB_ENGINE_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident );
                break;
                    
            case RDB_PKG_ID_DRIVETRAIN:
                print( *( ( RDB_DRIVETRAIN_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident );
                break;
                    
            case RDB_PKG_ID_WHEEL:
                print( *( ( RDB_WHEEL_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident );
                break;
 
            case RDB_PKG_ID_PED_ANIMATION:
                print( *( ( RDB_PED_ANIMATION_t* ) dataPtr ), ident );
                break;
 
            case RDB_PKG_ID_SENSOR_STATE:
                print( *( ( RDB_SENSOR_STATE_t* ) dataPtr ), ident );
                break;
 
            case RDB_PKG_ID_SENSOR_OBJECT:
                print( *( ( RDB_SENSOR_OBJECT_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_CAMERA:
                print( *( ( RDB_CAMERA_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_CONTACT_POINT:
                print( *( ( RDB_CONTACT_POINT_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_TRAFFIC_SIGN:
                print( *( ( RDB_TRAFFIC_SIGN_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_ROAD_STATE:
                print( *( ( RDB_ROAD_STATE_t* ) dataPtr ), ident );
                break;
*/                    
            case RDB_PKG_ID_IMAGE:
            case RDB_PKG_ID_LIGHT_MAP:
                //handleRDBitem( simTime, simFrame, *( ( RDB_IMAGE_t* ) dataPtr ) );
                break;
/*                    
            case RDB_PKG_ID_LIGHT_SOURCE:
                print( *( ( RDB_LIGHT_SOURCE_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident );
                break;
                    
            case RDB_PKG_ID_ENVIRONMENT:
                print( *( ( RDB_ENVIRONMENT_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_TRIGGER:
                print( *( ( RDB_TRIGGER_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_DRIVER_CTRL:
                print( *( ( RDB_DRIVER_CTRL_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_TRAFFIC_LIGHT:
                print( *( ( RDB_TRAFFIC_LIGHT_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident );
                break;
                    sendDriverCtrl
            case RDB_PKG_ID_SYNC:
                print( *( ( RDB_SYNC_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_DRIVER_PERCEPTION:
                print( *( ( RDB_DRIVER_PERCEPTION_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_TONE_MAPPING:
                print( *( ( RDB_FUNCTION_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_ROAD_QUERY:
                print( *( ( RDB_ROAD_QUERY_t* ) dataPtr ), ident );
                break;
                    
            case RDB_PKG_ID_TRAJECTORY:
                print( *( ( RDB_TRAJECTORY_t* ) dataPtr ), ident );
                break;
 
            case RDB_PKG_ID_DYN_2_STEER:
                print( *( ( RDB_DYN_2_STEER_t* ) dataPtr ), ident );
                break;
 
            case RDB_PKG_ID_STEER_2_DYN:
                print( *( ( RDB_STEER_2_DYN_t* ) dataPtr ), ident );
                break;
 
            case RDB_PKG_ID_PROXY:
                print( *( ( RDB_PROXY_t* ) dataPtr ), ident );
                break;
*/
            default:
                printedMsg = false;
                break;
        }
        dataPtr += entryHdr->elementSize;
            
/*        if ( noElements && printedMsg )
            fprintf( stderr, "\n" );
            */
     }
}
 
double g_laneWidth = 3.5;  // default fallback
bool   g_isSolidLine = false;
 
void handleRoadMarking(const RDB_ROADMARK_t& roadmark)
{
    // Filter to only consider the ego lane (optional: based on roadMark.id or pos.startLateral)
    g_laneWidth = roadmark.width;
 
    // Check if it's a solid line
    g_isSolidLine = (roadmark.type == RDB_ROADMARK_TYPE_SOLID);
 
    // Optional: print for debugging
    fprintf(stderr, "[ROADMARK] lateral=%.2f, type=%u, subType=%u, width=%.2f\n",
            roadmark.lateralDist,
            roadmark.type,
            roadmark.color,
            roadmark.width);
}
 
double clamp(double val, double minVal, double maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}
 
 
// void computeLKAControl(const RDB_OBJECT_STATE_t& egoState, double& steeringOut, double dt)
// {
//     double lateralOffset = egoState.base.pos.y;
//     double headingError = egoState.base.pos.h;
 
//     const double kLat = 1.5;
//     const double kHead = 2.0;
 
//     if (g_isSolidLine && std::abs(lateralOffset) > (g_laneWidth / 2.0 - 0.1)) {
//         fprintf(stderr, "[LKA] Approaching solid line! Correcting more aggressively.\n");
//         steeringOut = -2.0 * lateralOffset;
//     } else {
//         steeringOut = -(kLat * lateralOffset + kHead * headingError);
//     }
 
//     steeringOut = clamp(steeringOut, -1.5, 1.5);
 
//     fprintf(stderr, "[LKA] Offset=%.3f, YawErr=%.3f, LaneWidth=%.2f, Solid=%d -> steer=%.3f\n",
//             lateralOffset, headingError, g_laneWidth, g_isSolidLine, steeringOut);
// }
 
// void computeLKAControl(const RDB_OBJECT_STATE_t& egoState, double& steeringOut, double dt)
// {
//     double lateralOffset = egoState.base.pos.y;
//     double headingError = egoState.base.pos.h;
 
//     // PD control gains (can be tuned)
//     const double kLat = 2.0;   // lateral position gain
//     const double kHead = 1.5;  // heading gain
 
//     // Correction always applied to bring back to center
//     steeringOut = -(kLat * lateralOffset + kHead * headingError);
 
//     // Extra safety clamp if near solid line
//     if (g_isSolidLine && std::abs(lateralOffset) > (g_laneWidth / 2.0 - 0.1)) {
//         fprintf(stderr, "[LKA] Warning: Approaching solid line! Applying stronger correction.\n");
//         steeringOut *= 1.5;  // boost correction when near boundary
//     }
 
//     steeringOut = clamp(steeringOut, -1.5, 1.5);
 
//     fprintf(stderr, "[LKA] Offset=%.3f, YawErr=%.3f, LaneWidth=%.2f, Solid=%d -> steer=%.3f\n",
//             lateralOffset, headingError, g_laneWidth, g_isSolidLine, steeringOut);
// }
 
void computeLKAControl(const RDB_OBJECT_STATE_t& egoState, double& steeringOut, double dt)
{
    // World position of ego
    double posX = egoState.base.pos.x;
    double posY = egoState.base.pos.y;
    double yaw  = egoState.base.pos.h;  // heading in radians

    // Assume road center is not always y=0. We'll simulate road curvature.
    // You can replace this with real road curvature from RDB_ROAD_POS_t

    // === Simulated curved road model ===
    double curvature = 0.0015; // Popoki.comsitive = left curve, Negative = right curve
    double desiredYaw = atan(curvature * posX); // Desired heading tangent to curve
    double desiredY   = curvature * posX * posX / 2.0; // y of centerline at x

    double lateralOffset = posY - desiredY;
    double headingError = yaw - desiredYaw;

    // Normalize heading error
    while (headingError > M_PI) headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;

    // === PD Control ===
    const double kLat = 2.0;    // lateral position gain
    const double kHead = 1.5;   // heading gain

    steeringOut = -(kLat * lateralOffset + kHead * headingError);

    // Boost correction near solid lane lines
    if (g_isSolidLine && std::abs(lateralOffset) > (g_laneWidth / 2.0 - 0.1)) {
        fprintf(stderr, "[LKA] Near solid line! Boosting correction.\n");
        steeringOut *= 1.3;
    }

    steeringOut = clamp(steeringOut, -1.5, 1.5);

    // Debug log
    fprintf(stderr,
            "[LKA] Pos(%.2f, %.2f), Yaw=%.3f, DesiredYaw=%.3f, Offset=%.3f, Err=%.3f -> Steer=%.3f\n",
            posX, posY, yaw, desiredYaw, lateralOffset, headingError, steeringOut);
}

 
 
void handleRDBitem(const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t &item, bool isExtended) {
    if (item.base.id != 1) // Only control the ego vehicle
        return;
 
    double steeringCmd = 0.0;
 
    // Compute steering to bring vehicle to center of lane
    computeLKAControl(item, steeringCmd, 0.01);  // assume dt = 0.01 for now

    
    // Send driver control with updated steeringCmd
    Framework::RDBHandler myHandler;
    myHandler.initMsg();
    RDB_DRIVER_CTRL_t *myDriver = (RDB_DRIVER_CTRL_t*) myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);
 
    if (!myDriver)
        return;
 
    myDriver->playerId      = 1;
    myDriver->steeringTgt   = steeringCmd;
    myDriver->accelTgt      = 1.0; // Adjust as needed for your application
    myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING;
 
    int retVal = send(sClient, (const char*) (myHandler.getMsg()), myHandler.getMsgTotalSize(), 0);
 
    if (retVal <= 0)
        fprintf(stderr, "sendDriverCtrl (LKA): could not send control\n");
}
 
 
 
void sendLKAControl(int sendSocket, const double & simTime, const unsigned int & simFrame, double steeringTgt)
 
{
    Framework::RDBHandler myHandler;
 
    myHandler.initMsg();
 
    RDB_DRIVER_CTRL_t *myDriver = ( RDB_DRIVER_CTRL_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL );
 
    if ( !myDriver )
        return;
 
    myDriver->playerId      = 1;
    myDriver->accelTgt      = 0.5;  // Constant low accel for testing
    myDriver->steeringTgt   = steeringTgt;
    myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING;
 
    int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );
 
    if ( retVal <= 0 )
        fprintf(stderr, "[LKA] sendLKAControl: failed to send control\n");
}
void sendDriverCtrl(int &sendSocket, const double &simTime, const unsigned int &simFrame)
{
    Framework::RDBHandler myHandler;
    myHandler.initMsg();
 
    RDB_DRIVER_CTRL_t *myDriver = (RDB_DRIVER_CTRL_t *) myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);
 
    if (!myDriver)
        return;
 
    myDriver->playerId = 1;
    myDriver->accelTgt = 0.0;           // No acceleration input
    myDriver->steeringTgt = 0.0;        // Neutral steering
    myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING;
 
    int retVal = send(sendSocket, (const char *) (myHandler.getMsg()), myHandler.getMsgTotalSize(), 0);
 
    if (retVal <= 0)
        fprintf(stderr, "[DriverCtrl] sendDriverCtrl: failed to send control\n");
}
 
 