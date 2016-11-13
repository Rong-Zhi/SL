#ifndef OPTITRACKPARSER_H_
#define OPTITRACKPARSER_H_ 1

#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>


#include <netdb.h>
#include <stdio.h>
#include <errno.h>

#include <stdint.h>



class OptiTrackParser {

    private:

    static uint8_t* copyBytes(void* to, uint8_t* from, uint32_t numBytes);

    public:
    class Marker {
        private:
        uint8_t isIdSet;
        uint8_t isSizeSet;
        public:
        Marker() {
            isIdSet = 0;
            isSizeSet = 0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
            id = 0;
            markerSize = 0.0;
        };

        size_t parse(uint8_t* bytes){
            uint8_t* ptr = bytes;
            ptr = copyBytes(&x, ptr, sizeof(float));
            ptr = copyBytes(&y, ptr, sizeof(float));
            ptr = copyBytes(&z, ptr, sizeof(float));
            return ptr-bytes;
        };

        size_t parseId(uint8_t* bytes){
            copyBytes(&id, bytes, sizeof(int32_t));
            isIdSet = 1;
            return sizeof(int32_t);
        };

        size_t parseSize(uint8_t* bytes){
            copyBytes(&markerSize, bytes, sizeof(float));
            isSizeSet = 1;
            return sizeof(float);
        };

        void print(std::string indent = "", std::string prefix = ""){
            char buffer[16] = {' '};
            if(isIdSet){
                sprintf(buffer,"%s%d ",buffer,id);
            }
            if(isSizeSet){
                sprintf(buffer,"%s%.4f ",buffer,markerSize);
            }
            printf("%s%sMarker:%s[ % .4f % .4f % .4f ]\n",indent.c_str(),prefix.c_str(),buffer,x,y,z);
        };

        float x;
        float y;
        float z;
        int32_t id;
        float markerSize;
    };

    class MarkerSet {
        public:
        MarkerSet(){
            matched = 0;
        };

        size_t parse(uint8_t* bytes){
            uint8_t* ptr = bytes;
            name.assign( reinterpret_cast< char const* >(ptr) );
            ptr += name.size()+1;
            uint32_t numMarkers = 0;
            ptr = copyBytes(&numMarkers, ptr, sizeof(uint32_t));
            for(uint32_t idxMarker=0; idxMarker < numMarkers; idxMarker++){
                Marker marker;
                ptr += marker.parse(ptr);
                markers.push_back(marker);
            }

            return ptr-bytes;
        }

        void setMatched(uint8_t matched){
            this->matched = matched;
        }

        void print(std::string indent = "", std::string prefix = ""){
            printf("%s%sMarkerSet: '%s'\n",indent.c_str(),prefix.c_str(),name.c_str());
            char buffer[15];
            for(uint32_t idx = 0; idx < markers.size(); idx++){
                sprintf(buffer,"%03u / %03lu ",idx+1,markers.size());
                markers[idx].print(indent+"\t",buffer);
            }
        };
        std::string name;
        std::vector<Marker> markers;
        uint8_t matched;
    };

    class RigidBody {
        public:
        size_t parse(uint8_t* bytes){
            uint8_t* ptr = bytes;

            ptr = copyBytes(&id, ptr, sizeof(int32_t));
            ptr = copyBytes(&x, ptr, sizeof(float));
            ptr = copyBytes(&y, ptr, sizeof(float));
            ptr = copyBytes(&z, ptr, sizeof(float));
            ptr = copyBytes(&qx, ptr, sizeof(float));
            ptr = copyBytes(&qy, ptr, sizeof(float));
            ptr = copyBytes(&qz, ptr, sizeof(float));
            ptr = copyBytes(&qw, ptr, sizeof(float));

            uint32_t numMarkers;
            ptr = copyBytes(&numMarkers, ptr, sizeof(uint32_t));

            for(uint32_t idxMarker=0; idxMarker < numMarkers; idxMarker++){
                Marker marker;
                ptr += marker.parse(ptr);
                markers.push_back(marker);
            }

            if(NatNetVersion[0] >= 2){

                for(uint32_t idxMarker=0; idxMarker < numMarkers; idxMarker++){
                    ptr += markers[idxMarker].parseId(ptr);
                }

                for(uint32_t idxMarker=0; idxMarker < numMarkers; idxMarker++){
                    ptr += markers[idxMarker].parseSize(ptr);
                }

                ptr = copyBytes(&meanMarkerError, ptr, sizeof(float));
            }else{
                meanMarkerError = 0.0;
            }

            return ptr-bytes;
        };

        void setName(std::string name){
            this->name = name;
        }

        void print(std::string indent = "", std::string prefix = ""){
            printf("%s%sRigidBody: %d '%s'\n",indent.c_str(),prefix.c_str(),id,name.c_str());
            printf("%s\tposition: [ % .4f % .4f % .4f ]\n",indent.c_str(),x,y,z);
            printf("%s\torientation: [ % .4f % .4f % .4f % .4f ]\n",indent.c_str(),qw,qx,qy,qz);

            char buffer[16];
            for(uint32_t idxMarker=0; idxMarker < markers.size(); idxMarker++){
                sprintf(buffer,"%03u / %03lu ",idxMarker+1,markers.size());
                markers[idxMarker].print(indent+"\t",buffer);
            }
            if(NatNetVersion[0] >= 2){
                printf("%s\tMean Marker Error: %.4f\n",indent.c_str(),meanMarkerError);
            }
        };
        int32_t id;
        float x;
        float y;
        float z;
        float qw;
        float qx;
        float qy;
        float qz;
        std::vector<Marker> markers;
        float meanMarkerError;
        std::string name;
    };

    class Skeleton {
        public:
        size_t parse(uint8_t* bytes){
            uint8_t* ptr = bytes;

            ptr = copyBytes(&id, ptr, sizeof(int32_t));

            uint32_t numRigidBodies = 0;
            ptr = copyBytes(&numRigidBodies, ptr, sizeof(uint32_t));
            for (uint32_t idxRigidBody=0; idxRigidBody < numRigidBodies; idxRigidBody++) {
                RigidBody rigidBody;
                ptr += rigidBody.parse(ptr);
                rigidBodies.push_back(rigidBody);
            }

            return ptr-bytes;
        };

        void print(std::string indent = "", std::string prefix = ""){
            printf("%s%sSkeleton: %d\n",indent.c_str(),prefix.c_str(),id);
            char buffer[16];
            for(uint32_t idxRigidBody=0; idxRigidBody < rigidBodies.size(); idxRigidBody++){
                printf("\n");
                sprintf(buffer,"%03u / %03lu ",idxRigidBody+1,rigidBodies.size());
                rigidBodies[idxRigidBody].print(indent+"\t",buffer);
            }
        };

        int32_t id;
        std::vector<RigidBody> rigidBodies;
    };

    class Frame {
        public:
        size_t parse(uint8_t* bytes){
            uint8_t* ptr = bytes;

            ptr = copyBytes(&id, ptr, sizeof(int32_t));

            uint32_t numMarkerSets = 0;
            ptr = copyBytes(&numMarkerSets, ptr, sizeof(uint32_t));

            for (uint32_t idxMarkerSet=0; idxMarkerSet < numMarkerSets; idxMarkerSet++){
                MarkerSet markerSet;
                ptr += markerSet.parse(ptr);
                markerSets.push_back(markerSet);
            }
            uint32_t numUnidentifiedMarkers = 0;
            ptr = copyBytes(&numUnidentifiedMarkers, ptr, sizeof(uint32_t));

            for(uint32_t idxUnidentifiedMarker=0; idxUnidentifiedMarker < numUnidentifiedMarkers; idxUnidentifiedMarker++){
                Marker marker;
                ptr += marker.parse(ptr);
                unidentifiedMarkers.push_back(marker);
            }

            uint32_t numRigidBodies = 0;
            ptr = copyBytes(&numRigidBodies, ptr, sizeof(uint32_t));
            for (uint32_t idxRigidBody=0; idxRigidBody < numRigidBodies; idxRigidBody++) {
                RigidBody rigidBody;
                ptr += rigidBody.parse(ptr);
                rigidBody.setName(markerSets[idxRigidBody].name);
                rigidBodies.push_back(rigidBody);
            }

            if( (NatNetVersion[0] > 2) || ( (NatNetVersion[0] == 2) && (NatNetVersion[1] > 0) ) ){
                uint32_t numSkeletons = 0;
                ptr = copyBytes(&numSkeletons, ptr, sizeof(uint32_t));
                for (uint32_t idxSkeleton=0; idxSkeleton < numSkeletons; idxSkeleton++) {
                    Skeleton skeleton;
                    ptr += skeleton.parse(ptr);
                    skeletons.push_back(skeleton);
                }
            }

            ptr = copyBytes(&latency, ptr, sizeof(float));

            int32_t endOfData;
            ptr = copyBytes(&endOfData, ptr, sizeof(int32_t));
            return ptr-bytes;
        };

        void print(std::string indent = "", std::string prefix = ""){
            printf("%s%sFrame: %d\n",indent.c_str(),prefix.c_str(),id);
            char buffer[16];

            printf("\n\n%s\tMarkerSets:\n",indent.c_str());
            for(uint32_t idxMarkerSet=0; idxMarkerSet < markerSets.size(); idxMarkerSet++){
                printf("\n");
                sprintf(buffer,"%03u / %03lu ",idxMarkerSet+1,markerSets.size());
                markerSets[idxMarkerSet].print(indent+"\t",buffer);
            }

            printf("\n\n%s\tUnidentified Markers:\n",indent.c_str());
            for(uint32_t idxMarker = 0; idxMarker < unidentifiedMarkers.size(); idxMarker++){
                sprintf(buffer,"%03u / %03lu ",idxMarker+1,unidentifiedMarkers.size());
                unidentifiedMarkers[idxMarker].print(indent+"\t",buffer);
            }

            printf("\n\n%s\tRigid Bodies:\n",indent.c_str());
            for(uint32_t idxRigidBody=0; idxRigidBody < rigidBodies.size(); idxRigidBody++){
                printf("\n");
                sprintf(buffer,"%03u / %03lu ",idxRigidBody+1,rigidBodies.size());
                rigidBodies[idxRigidBody].print(indent+"\t",buffer);
            }

            printf("\n\n%s\tSkeletons:\n",indent.c_str());
            for(uint32_t idxSkeleton=0; idxSkeleton < skeletons.size(); idxSkeleton++){
                printf("\n");
                sprintf(buffer,"%03u / %03lu ",idxSkeleton+1,skeletons.size());
                skeletons[idxSkeleton].print(indent+"\t",buffer);
            }

            printf("\n\n%sLatency: %.4f",indent.c_str(),latency);
        };
        int32_t id;
        std::vector<MarkerSet> markerSets;
        std::vector<Marker> unidentifiedMarkers;
        std::vector<RigidBody> rigidBodies;
        std::vector<Skeleton> skeletons;
        float latency;
    };


    protected:

    int32_t socketId;
    std::string ipAddress;
    uint32_t port;

    const static uint32_t NatNetVersion[];

    int32_t getSocketId(std::string ipAddress, uint32_t port);
    void parseDescription(uint8_t* description, uint16_t numBytes);

    void processNextMessageDummy();

    void start();

    public:

    int processNextMessageSL(Frame& frm);

    OptiTrackParser(std::string ipAddress, uint32_t port);
    virtual ~OptiTrackParser();
};


#endif /* OPTITRACKPARSER_H_ */

