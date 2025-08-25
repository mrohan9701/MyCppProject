// Copyright 2025 Hexagon Manufacturing Intelligence GmbH

// ExampleVehDynInteg.cpp
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#define DEFAULT_PORT        48190   /* for image port it should be 48192 */
#define DEFAULT_BUFFER      204800

char                      szServer[128];        // Server to connect to
int                       iPort = DEFAULT_PORT; // Port on server to connect to
static int                sClient = -1;         // client socket
static RDB_OBJECT_STATE_t sOwnObjectState[2];   // own object's stateconst float SAFE_STOP_DISTANCE = 10.0f;     // Distance at which ego should start braking (meters)
const float SAFE_ACCEL_DISTANCE = 20.0f;    // Distance beyond which ego starts accelerating
const float MAX_ACCEL = 0.5f;                // Max throttle value for acceleration
const float MAX_BRAKE = 1.0f;                // Max brake pedal value




// function prototypes
void sendOwnObjectState( RDB_OBJECT_STATE_t* state, int & sendSocket, const double & simTime, const unsigned int & simFrame );
void computeDynamics( unsigned int playerId, RDB_OBJECT_STATE_t* state, const double & dt, const double & simTime, const unsigned int & simFrame );
int getNoReadyRead( int descriptor );

//
// Function: usage:
//
// Description:
//    Print usage information and exit
//
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

int main(int argc, char* argv[])
{
    char*              szBuffer = new char[DEFAULT_BUFFER];  // allocate on heap
    int                ret;
    struct sockaddr_in server;
    struct hostent    *host     = NULL;
    static bool        sVerbose = false;
    double             simTime  = 0.0;
    unsigned int       simFrame = 0;

    // Parse the command line
    //
    ValidateArgs(argc, argv);
    
    // initialize some variables
    memset( &sOwnObjectState[0], 0, sizeof( RDB_OBJECT_STATE_t ) );
    memset( &sOwnObjectState[1], 0, sizeof( RDB_OBJECT_STATE_t ) );
    
    //
    // Create the socket, and attempt to connect to the server
    //
    sClient = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
    
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

    // set to non-blocking
    opt = 1;
    ioctl( sClient, FIONBIO, &opt );

	// wait for connection
	bool bConnected = false;

    while ( !bConnected )
    {
        if ( connect( sClient, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
        {
            fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
            sleep( 1 );
        }
        else
            bConnected = true;
    }

    fprintf( stderr, "connected!\n" );
    
    // Main loop, send and receive data - forever!
    //
    for(;;)
    {
        // make sure this is non-bloekcing
        double dt = 0.010;
    
        // first read all network data (do not CLOG the ports)
        do
        {
            int nrReady = getNoReadyRead( sClient ); // MUST BE ZERO in order to be non-blocking!!!
 
            if ( nrReady < 0 )
            {
                printf( "recv() failed: %s\n", strerror( errno ) );
                break;
            }
            // read data
            ret = recv( sClient, szBuffer, DEFAULT_BUFFER, 0 );
            
        } while ( ret > 0 );

        // do some other stuff before returning to network reading
        computeDynamics( 1, &sOwnObjectState[0], dt, simTime, simFrame );
        computeDynamics( 2, &sOwnObjectState[1], dt, simTime, simFrame );
        
        simTime += dt;
        simFrame++;
        
        // don't use the processor excessively
        usleep( 10000 );
    }
    ::close(sClient);

    return 0;
}


void computeDynamics( unsigned int playerId, RDB_OBJECT_STATE_t* state, const double & dt, const double & simTime, const unsigned int & simFrame )
{
    if ( !state )
        return;
        
    double speedX = 5.0 * playerId;    // m/s
    double speedY = 0.0;    // m/s
    double speedZ = 0.0;    // m/s
    
    state->base.id       = playerId;
    state->base.category = RDB_OBJECT_CATEGORY_PLAYER;
    state->base.type     = RDB_OBJECT_TYPE_PLAYER_CAR;
    
    sprintf( state->base.name, "%s_%d", "Ego", playerId );
 
    // dimensions of own vehicle
    state->base.geo.dimX = 4.60;
    state->base.geo.dimY = 1.86;
    state->base.geo.dimZ = 1.60;
 
    // offset between reference point and center of geometry
    state->base.geo.offX = 0.80;
    state->base.geo.offY = 0.00;
    state->base.geo.offZ = 0.30;
 
    state->base.pos.x     += dt * speedX;
    state->base.pos.y     += dt * speedY;
    state->base.pos.z     += dt * speedZ;
    state->base.pos.h     = 0.0;
    state->base.pos.p     = 0.0;
    state->base.pos.r     = 0.0;
    state->base.pos.type  = RDB_COORD_TYPE_RELATIVE_START;
    state->base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
 
    state->ext.speed.x     = speedX;
    state->ext.speed.y     = speedY;
    state->ext.speed.z     = speedZ;
    state->ext.speed.h     = 0.0;
    state->ext.speed.p     = 0.0;
    state->ext.speed.r     = 0.0;
    state->ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
 
    state->ext.accel.x     = 0.0;
    state->ext.accel.y     = 0.0;
    state->ext.accel.z     = 0.0;
    state->ext.accel.flags = RDB_COORD_FLAG_POINT_VALID;
 
    state->base.visMask    =  RDB_OBJECT_VIS_FLAG_TRAFFIC | RDB_OBJECT_VIS_FLAG_RECORDER;
    
    // ok, I have a new object state, so let's send the data
    sendOwnObjectState( state, sClient, simTime, simFrame );
}

void sendOwnObjectState( RDB_OBJECT_STATE_t* state, int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
    if ( !state )
        return;
        
    Framework::RDBHandler myHandler;

    // start a new message
    myHandler.initMsg();

    // begin with an SOF identifier
    myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_START_OF_FRAME );

    // add extended package for the object state
    RDB_OBJECT_STATE_t *objState = ( RDB_OBJECT_STATE_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_OBJECT_STATE, 1, true );

    if ( !objState )
    {
        fprintf( stderr, "sendOwnObjectState: could not create object state\n" );
        return;
    }
        
    // copy contents of internally held object state to output structure
    memcpy( objState, state, sizeof( RDB_OBJECT_STATE_t ) );

    fprintf( stderr, "sendOwnObjectState: sending pos x/y/z = %.3lf/%.3lf/%.3lf,\n", objState->base.pos.x, objState->base.pos.y, objState->base.pos.z );
        
    // terminate with an EOF identifier
    myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_END_OF_FRAME );

    int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

    if ( !retVal )
        fprintf( stderr, "sendOwnObjectState: could not send object state\n" );
}

int getNoReadyRead( int descriptor )
{
    fd_set         fs;
    struct timeval time;
    
    // still no valid descriptor?
    if ( descriptor < 0 )
        return 0;

    FD_ZERO( &fs );
    FD_SET( descriptor, &fs );
    
    memset( &time, 0, sizeof( struct timeval ) );

    int retVal = 0;
    
    retVal = select( descriptor + 1, &fs, (fd_set*) 0, (fd_set*) 0, &time);
    
    if ( retVal > 0 )
        return FD_ISSET( descriptor, &fs );
    
    return retVal;
}
int main(int argc, char* argv[])
{
    char*              szBuffer = new char[DEFAULT_BUFFER];  // allocate on heap
    int                ret;
    struct sockaddr_in server;
    struct hostent    *host     = NULL;
    static bool        sVerbose = false;
    double             simTime  = 0.0;
    unsigned int       simFrame = 0;

    // Parse the command line
    //
    ValidateArgs(argc, argv);
    
    // initialize some variables
    memset( &sOwnObjectState[0], 0, sizeof( RDB_OBJECT_STATE_t ) );
    memset( &sOwnObjectState[1], 0, sizeof( RDB_OBJECT_STATE_t ) );
    
    //
    // Create the socket, and attempt to connect to the server
    //
    sClient = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
    
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

    // set to non-blocking
    opt = 1;
    ioctl( sClient, FIONBIO, &opt );

	// wait for connection
	bool bConnected = false;

    while ( !bConnected )
    {
        if ( connect( sClient, (struct sockaddr *)&server, sizeof( server ) ) == -1 )
        {
            fprintf( stderr, "connect() failed: %s\n", strerror( errno ) );
            sleep( 1 );
        }
        else
            bConnected = true;
    }

    fprintf( stderr, "connected!\n" );
    
    // Main loop, send and receive data - forever!
    //
    for(;;)
    {
        // make sure this is non-bloekcing
        double dt = 0.010;
    
        // first read all network data (do not CLOG the ports)
        do
        {
            int nrReady = getNoReadyRead( sClient ); // MUST BE ZERO in order to be non-blocking!!!
 
            if ( nrReady < 0 )
            {
                printf( "recv() failed: %s\n", strerror( errno ) );
                break;
            }
            // read data
            ret = recv( sClient, szBuffer, DEFAULT_BUFFER, 0 );
            
        } while ( ret > 0 );

        // do some other stuff before returning to network reading
        computeDynamics( 1, &sOwnObjectState[0], dt, simTime, simFrame );
        computeDynamics( 2, &sOwnObjectState[1], dt, simTime, simFrame );
        
        simTime += dt;
        simFrame++;
        
        // don't use the processor excessively
        usleep( 10000 );
    }
    ::close(sClient);

    return 0;
}


void computeDynamics( unsigned int playerId, RDB_OBJECT_STATE_t* state, const double & dt, const double & simTime, const unsigned int & simFrame )
{
    if ( !state )
        return;
        
    double speedX = 5.0 * playerId;    // m/s
    double speedY = 0.0;    // m/s
    double speedZ = 0.0;    // m/s
    
    state->base.id       = playerId;
    state->base.category = RDB_OBJECT_CATEGORY_PLAYER;
    state->base.type     = RDB_OBJECT_TYPE_PLAYER_CAR;
    
    sprintf( state->base.name, "%s_%d", "Ego", playerId );
 
    // dimensions of own vehicle
    state->base.geo.dimX = 4.60;
    state->base.geo.dimY = 1.86;
    state->base.geo.dimZ = 1.60;
 
    // offset between reference point and center of geometry
    state->base.geo.offX = 0.80;
    state->base.geo.offY = 0.00;
    state->base.geo.offZ = 0.30;
 
    state->base.pos.x     += dt * speedX;
    state->base.pos.y     += dt * speedY;
    state->base.pos.z     += dt * speedZ;
    state->base.pos.h     = 0.0;
    state->base.pos.p     = 0.0;
    state->base.pos.r     = 0.0;
    state->base.pos.type  = RDB_COORD_TYPE_RELATIVE_START;
    state->base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
 
    state->ext.speed.x     = speedX;
    state->ext.speed.y     = speedY;
    state->ext.speed.z     = speedZ;
    state->ext.speed.h     = 0.0;
    state->ext.speed.p     = 0.0;
    state->ext.speed.r     = 0.0;
    state->ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
 
    state->ext.accel.x     = 0.0;
    state->ext.accel.y     = 0.0;
    state->ext.accel.z     = 0.0;
    state->ext.accel.flags = RDB_COORD_FLAG_POINT_VALID;
 
    state->base.visMask    =  RDB_OBJECT_VIS_FLAG_TRAFFIC | RDB_OBJECT_VIS_FLAG_RECORDER;
    
    // ok, I have a new object state, so let's send the data
    sendOwnObjectState( state, sClient, simTime, simFrame );
}

void sendOwnObjectState( RDB_OBJECT_STATE_t* state, int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
    if ( !state )
        return;
        
    Framework::RDBHandler myHandler;

    // start a new message
    myHandler.initMsg();

    // begin with an SOF identifier
    myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_START_OF_FRAME );

    // add extended package for the object state
    RDB_OBJECT_STATE_t *objState = ( RDB_OBJECT_STATE_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_OBJECT_STATE, 1, true );

    if ( !objState )
    {
        fprintf( stderr, "sendOwnObjectState: could not create object state\n" );
        return;
    }
        
    // copy contents of internally held object state to output structure
    memcpy( objState, state, sizeof( RDB_OBJECT_STATE_t ) );

    fprintf( stderr, "sendOwnObjectState: sending pos x/y/z = %.3lf/%.3lf/%.3lf,\n", objState->base.pos.x, objState->base.pos.y, objState->base.pos.z );
        
    // terminate with an EOF identifier
    myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_END_OF_FRAME );

    int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

    if ( !retVal )
        fprintf( stderr, "sendOwnObjectState: could not send object state\n" );
}

int getNoReadyRead( int descriptor )
{
    fd_set         fs;
    struct timeval time;
    
    // still no valid descriptor?
    if ( descriptor < 0 )
        return 0;

    FD_ZERO( &fs );
    FD_SET( descriptor, &fs );
    
    memset( &time, 0, sizeof( struct timeval ) );

    int retVal = 0;
    
    retVal = select( descriptor + 1, &fs, (fd_set*) 0, (fd_set*) 0, &time);
    
    if ( retVal > 0 )
        return FD_ISSET( descriptor, &fs );
    
    return retVal;
}
