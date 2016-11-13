#include "OptiTrackParser.h"

#include <iostream>

const uint32_t OptiTrackParser::NatNetVersion[] = {2,2,0,0};

OptiTrackParser::OptiTrackParser(std::string ipAddress, uint32_t port) {
    this->ipAddress = ipAddress;
    this->port = port;
    this->socketId = getSocketId(ipAddress, port);
}

OptiTrackParser::~OptiTrackParser(){
    // TODO Auto-generated destructor stub
}

int32_t OptiTrackParser::getSocketId(std::string ipAddress, uint32_t port){

    int32_t socketId = socket( AF_INET, SOCK_DGRAM, 0 );
    struct sockaddr_in localAddr;
    struct ip_mreq mreq;

    int option_value = 1;
    if( setsockopt( socketId, SOL_SOCKET, SO_REUSEADDR, (void*)&option_value, sizeof( int ) ) == 1 ){
        printf("Set Sock Error!\n");
    }

    memset ( &localAddr, 0, sizeof ( localAddr ) );
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl( INADDR_ANY );
    localAddr.sin_port = htons( port );

    if ( bind( socketId, (struct sockaddr *)&localAddr, sizeof( localAddr ) ) == -1 ){
        printf("Bind Error\n");
    }

    mreq.imr_multiaddr.s_addr = inet_addr( ipAddress.c_str() );
    mreq.imr_interface = localAddr.sin_addr;


    setsockopt(socketId, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));

    //int flags = fcntl( socketId, F_GETFL , 0 );
    //if(fcntl(socketId, F_SETFL, flags | O_NONBLOCK)==-1){
    //	printf("failed to enable non-blocking");
    //}

    return socketId;
}

int OptiTrackParser::processNextMessageSL(Frame& frm){

    uint8_t buffer[20000];
    struct sockaddr_in remote_addr;
    int addr_len = sizeof(struct sockaddr);

    uint16_t msgId, numBytes;

    int status = recvfrom( this->socketId, buffer, 20000, 0, (struct sockaddr *)&remote_addr, (socklen_t*)&addr_len );

    if( status > 0 ) {
        memcpy(&msgId,buffer,sizeof(uint16_t));
        memcpy(&numBytes,buffer+sizeof(uint16_t),sizeof(uint16_t));

        if(status > 0){
            if(msgId == 7){      // FRAME OF MOCAP DATA packet
                frm.parse(buffer+sizeof(uint16_t)*2);
                return 1;
            }else if(msgId == 5){ // Data Descriptions
                parseDescription(buffer+sizeof(uint16_t)*2,numBytes);
            }else{
                printf("Unrecognized Message Type.\n");
            }
        }
    }

    return 0;
}


uint8_t* OptiTrackParser::copyBytes(void* to, uint8_t* from, uint32_t numBytes){
    memcpy(to, from, numBytes);
    return (from + numBytes);
}


void OptiTrackParser::parseDescription(uint8_t* description, uint16_t numBytes){

    uint8_t* ptr = description;

    int i,j,k;
    int major = OptiTrackParser::NatNetVersion[0];
    int minor = OptiTrackParser::NatNetVersion[1];

    // number of datasets
    int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
    printf("Dataset Count : %d\n", nDatasets);

    for(i=0; i < nDatasets; i++)
    {
        printf("Dataset %d\n", i);

        int type = 0; memcpy(&type, ptr, 4); ptr += 4;
        printf("%d Type : %d\n", i, type);

        if(type == 0)   // markerset
        {
            // name
            std::string szName( reinterpret_cast< char const* >(ptr) );
            ptr += szName.size()+1;
            printf("Markerset Name: %s\n", szName.c_str());

            // marker data
            int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
            printf("Marker Count : %d\n", nMarkers);

            for(j=0; j < nMarkers; j++)
            {
                std::string szName( reinterpret_cast< char const* >(ptr) );
                ptr += szName.size()+1;
                printf("Marker Name: %s\n", szName.c_str());
            }
        }
        else if(type ==1)   // rigid body
        {
            if(major >= 2)
            {
                // name
                std::string szName( reinterpret_cast< char const* >(ptr) );
                ptr += szName.size()+1;
                printf("Name: %s\n", szName.c_str());

            }

            int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
            printf("ID : %d\n", ID);

            int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
            printf("Parent ID : %d\n", parentID);

            float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
            printf("X Offset : %3.2f\n", xoffset);

            float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
            printf("Y Offset : %3.2f\n", yoffset);

            float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
            printf("Z Offset : %3.2f\n", zoffset);

        }
        else if(type ==2)   // skeleton
        {
            std::string szName( reinterpret_cast< char const* >(ptr) );
            ptr += szName.size()+1;
            printf("Name: %s\n", szName.c_str());

            int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
            printf("ID : %d\n", ID);

            int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
            printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

            for(i=0; i< nRigidBodies; i++)
            {
                if(major >= 2)
                {
                    // RB name
                    std::string szName( reinterpret_cast< char const* >(ptr) );
                    ptr += szName.size()+1;
                    printf("Rigid Body Name: %s\n", szName.c_str());
                }

                int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
                printf("RigidBody ID : %d\n", ID);

                int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
                printf("Parent ID : %d\n", parentID);

                float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
                printf("X Offset : %3.2f\n", xoffset);

                float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
                printf("Y Offset : %3.2f\n", yoffset);

                float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
                printf("Z Offset : %3.2f\n", zoffset);
            }
        }

    }   // next dataset

    printf("End Packet\n-------------\n");
}



