// Copyright 2025 Hexagon Manufacturing Intelligence GmbH

// ExampleConsole.cpp : Defines the entry point for the console application.
//

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

#define DEFAULT_PORT        48190   /* for image port it should be 48192 */
#define DEFAULT_BUFFER      204800

char  szServer[128];             // Server to connect to
int   iPort     = DEFAULT_PORT;  // Port on server to connect to
int   sClient;

// function prototypes
void parseRDBMessage( RDB_MSG_t* msg, bool & isImage );
void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );
void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_PROXY_t* item );
void sendProxyPkg( int & sendSocket, const double & simTime, const unsigned int & simFrame );


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
                        if ( sVerbose )
                            Framework::RDBHandler::printMessage( ( RDB_MSG_t* ) pData, true );
                        
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

		        sendProxyPkg( sClient, simTime, simFrame );
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
            case RDB_PKG_ID_OBJECT_STATE:
                handleRDBitem( simTime, simFrame, *( ( RDB_OBJECT_STATE_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED );
                break;
                    
            case RDB_PKG_ID_VEHICLE_SYSTEMS:
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

*/
            case RDB_PKG_ID_PROXY:
                handleRDBitem( simTime, simFrame, ( RDB_PROXY_t* ) dataPtr );
                break;
                
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

void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_PROXY_t* item )
{
  if ( !item )
    return;
    
  fprintf( stderr, "handleRDBitem: handling proxy package\n" );
  fprintf( stderr, "    simTime = %.3lf\n", simTime );
  fprintf( stderr, "    protocol = %d\n", item->protocol );
  fprintf( stderr, "    pkgId    = %d\n", item->pkgId    );
  fprintf( stderr, "    dataSize = %d\n", item->dataSize );
  
  unsigned char* myDataPtr = ( unsigned char * ) ( ( ( char* ) item ) + sizeof( RDB_PROXY_t ) );
  
  fprintf( stderr, "    testData = %d\n", myDataPtr[0] );
}

void sendProxyPkg( int & sendSocket, const double & simTime, const unsigned int & simFrame )
{
  Framework::RDBHandler myHandler;

  myHandler.initMsg();
  
  size_t trailingDataSize = 50; // 50 bytes

  RDB_PROXY_t *myProxy = ( RDB_PROXY_t* ) myHandler.addPackage( simTime, simFrame, RDB_PKG_ID_PROXY, 1, false, trailingDataSize );

  if ( !myProxy )
    return;

  myProxy->protocol = simFrame % 65000;
  myProxy->pkgId    = 555;
  myProxy->dataSize = trailingDataSize;
  
  unsigned char* myDataPtr = ( unsigned char * ) ( ( ( char* ) myProxy ) + sizeof( RDB_PROXY_t ) );
  
  myDataPtr[0] = ( unsigned char ) simFrame % 255;

  int retVal = send( sendSocket, ( const char* ) ( myHandler.getMsg() ), myHandler.getMsgTotalSize(), 0 );

  if ( retVal )
    fprintf( stderr, "sendProxyPkg: sent package\n" );
  else
    fprintf( stderr, "sendProxyPkg: could not send package\n" );

}


