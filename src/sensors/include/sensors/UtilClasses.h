
#pragma once
#ifndef UTILCLASSES_H
#define UTILCLASSES_H
//#include <sys/socket.h>
//#include <linux/if.h>
//#include <unistd.h>
#include <limits.h>
#include <vector>

//#include <dirent.h>
//#include <sys/ioctl.h>
#include <sys/types.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

//#include <bits/stdc++.h>

//#include <termios.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
//#include <unistd.h>
//#include <netdb.h>
#include <vector>
#include <streambuf>
#include <assert.h>
#include <ctime>
// #include "zip.h"
//#include "tinyxml2.h"
#include "math.h"
//#include <sys/mman.h>
// #include "parson.h"
// #include "json.h"
//#include "Cryptography.h"

#define PI                                  3.14159265358979323846
#define BINARY_BOUND                        "res/Bound.bin"
#define BUNDLED_LOG_FILE                    "res/FlightLogBundle.zip"
#define LOG_FILE                            "res/FlightLog_"
#define LOG_NAME                            "FlightLog_"
#define SYSTEM_INFO                         "res/SystemInfo.dat"
#define DIRECTORY                           "res/"
#define PERMISSION_ARTEFACT                 "res/artefact.xml"
#define APP_NAME                            "bfcom4"
#define APP_SIGNATURE                       "res/RFMManagerSignature.txt"
#define APP_NAME_UPDATE                     "_bfcom4"
#define APP_SIGNATURE_UPDATE                "res/_RFMManagerSignature.txt"
#define PUBLIC_KEY_CERTIFICATE_FILE         "res/ideaForgePublicCertificate.pem"


using namespace std;

class ArtefactData
{
public:
    std::string permissionArtefactID = "";
    std::string pilotPinHash = "";
    std::string operatorId = "";
    std::string uaplNo = "";
    std::string uinNo = "";
    std::vector <float> gpsCoordinates;


    // unsigned int lastUpdated = 0;
    // unsigned int ttl = 0;
    // unsigned int txnId = 0;
    // unsigned int permissionartefactId = 0;
    
    // unsigned int operatorId = 0;
    // unsigned int pilotNo = 0;
    // unsigned int pilotNoValidTo = 0;
    
    // std::string shortDesc = "";
    // unsigned int frequency = 0;
    // float payLoadWeight = 0;
    // std::string payloadDetails = "";
    
    // unsigned int flightStartTime = 0;
    // unsigned int flightStartEnd = 0;
    // unsigned int frequenciesUsed = 0;
    // std::string DigitalSignature = "";
};

class UtilPoint
{
    public : 
    float lat,lon;
};


class FlightLogData
{
	public:
	typedef struct CurrentUavSensorData
	{
		//float lat, lng, alt;
		float hdop, numSattelites, heading, groundSpeed, altAGL, altMSL, longitude, latitude;
	}CurrentUavSensorData;
	CurrentUavSensorData sensorData[2];
	float groundSpeed;
    float altitudeAGL, altitudeMSL, latitude, longitude; //pressure sensor's AGL
    uint8_t mode;
    uint32_t date, time;
    int16_t Az, Ay, Ax; // Acceleration
    
	int16_t Bz, By, Bx; // Magnetometer
    int16_t R, Q, P; 

    int16_t dYaw;  // r // Change in Yaw(Psi) (rad/sec)
    int16_t dPitch;// q // Change in Pitch(Theta) (rad/sec)
    int16_t dRoll; // p // Change in Roll(Phi) (rad/sec)

    float float_dYaw;  
    float float_dPitch;
    float float_dRoll; 
};

class Polygon
{
    public :
    //class Point polygonVertices[5];
    vector <class UtilPoint> polygonVertices;
    vector<class UtilPoint> polygonVerticesLog;
    class UtilPoint uavPos;
    int crossing,num_vertices,num;

    int PIP_LOG()
    {
/*
        for (int i = 0; i < num_vertices; i++)
        {
            if (((polygonVerticesLog[i].lon <= uavPos.lon) && (polygonVerticesLog[i + 1].lon > uavPos.lon)) || ((polygonVerticesLog[i].lon > uavPos.lon) && (polygonVerticesLog[i + 1].lon <= uavPos.lon)))
            {
                float vt = (float)(uavPos.lon - polygonVerticesLog[i].lon) / (polygonVerticesLog[i + 1].lon - polygonVerticesLog[i].lon);
                if (uavPos.lat < polygonVerticesLog[i].lat + vt * (polygonVerticesLog[i + 1].lat - polygonVerticesLog[i].lat))
                {
                    ++crossing;
                }
            }
        }
        num = crossing;
        crossing = 0;
        return (num & 1);
*/
	return 0;
    }

    int PIP () 
    {
/*
        for(int i=0;i<num_vertices;i++)
        {
            if (((polygonVertices[i].lon <= uavPos.lon) && (polygonVertices[i+1].lon > uavPos.lon)) || ((polygonVertices[i].lon > uavPos.lon) && (polygonVertices[i+1].lon <=  uavPos.lon)))
            {
                float vt = (float)(uavPos.lon  - polygonVertices[i].lon) / (polygonVertices[i+1].lon - polygonVertices[i].lon);
                if (uavPos.lat <  polygonVertices[i].lat + vt * (polygonVertices[i+1].lat - polygonVertices[i].lat))
                {
                    ++crossing;
                }
            } 
        }
        num=crossing;crossing=0;
        return (num&1);
*/
	return 0;
    }
};


// class MessageBroadCast
// {
//     #define PORT 11001
//     #define PACKET_SIZE 1024
//     unsigned char tx_packet[PACKET_SIZE];
//     struct sockaddr_in transmit_address;
// 	int socket_transmit;
//     std::string ipAddress = "";
// public:
//     MessageBroadCast(std::string ipAddress)
//     {
//         socket_transmit = socket(AF_INET, SOCK_DGRAM, 0); this->ipAddress = ipAddress;
// 	    if (socket_transmit < 0) std::cout << "Socket Create Error" << std::endl;
//         int broadcast = 1;
// 	    if(setsockopt(socket_transmit, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast) < 0) std::cout << "Setsockopt Error - SO_BROADCAST" << std::endl;
// 	    memset((char *)&transmit_address, 0, sizeof(transmit_address));
//         transmit_address.sin_family = AF_INET;
//         transmit_address.sin_addr.s_addr = inet_addr(ipAddress.c_str());
// 	    transmit_address.sin_port = htons(PORT);
// 	    std::cout << " MessageBroadCast Initailized" << std::endl;
//     }
//     void sendMessages(std::string message)
//     {
//         unsigned char* packetPointer;
//         char *messageData = new char[message.length()]; strcpy(messageData, message.c_str());
//         memset((char *)tx_packet, 0, PACKET_SIZE); packetPointer = tx_packet;
//         memcpy(packetPointer, new int (109), sizeof(int)); packetPointer += sizeof(int);
//         memcpy(packetPointer, new int (message.length()), sizeof(int)); packetPointer += sizeof(int);
//         memcpy(packetPointer, messageData, message.length()); packetPointer += message.length();
//         memcpy(&tx_packet[PACKET_SIZE-4], new int (109), sizeof(int)); packetPointer += sizeof(int);
//         int ret = sendto(socket_transmit, tx_packet, PACKET_SIZE, 0, (struct sockaddr *) &transmit_address, sizeof transmit_address);
// 	    if(ret < 0) printf("Socket Send Error %d\n", ret);
//     }
// };



static int16_t ReverseInt16(const int16_t inInt16 )
{
    int16_t retVal;
    char *floatToConvert = ( char* ) & inInt16;
    char *returnInt16 = ( char* ) & retVal;
    returnInt16[0] = floatToConvert[1];
    returnInt16[1] = floatToConvert[0];
    return retVal;
}

static float ReverseFloat( const float inFloat )
{   
    float retVal;
    char *floatToConvert = ( char* ) & inFloat;
    char *returnFloat = ( char* ) & retVal;
    returnFloat[0] = floatToConvert[3];
    returnFloat[1] = floatToConvert[2];
    returnFloat[2] = floatToConvert[1];
    returnFloat[3] = floatToConvert[0];
    return retVal;    
}

static int ReverseInt( const int inInt )
{
    int retVal;
    char *intToConvert = ( char* ) & inInt;
    char *returnInt = ( char* ) & retVal;
	returnInt[0] = intToConvert[3];
	returnInt[1] = intToConvert[2];
	returnInt[2] = intToConvert[1];
	returnInt[3] = intToConvert[0];
    return retVal;
}

static uint32_t ReverseInt32( const uint32_t inInt )
{
    uint32_t retVal;
    char *intToConvert = ( char* ) & inInt;
    char *returnInt = ( char* ) & retVal;
	returnInt[0] = intToConvert[3];
	returnInt[1] = intToConvert[2];
	returnInt[2] = intToConvert[1];
	returnInt[3] = intToConvert[0];
    return retVal;
}




//static ArtefactData ReadArtefact()
//{
//    cout << " :Read Artefct" << endl;
//    ArtefactData artefactData;
//    artefactData.gpsCoordinates.clear();
//    if (access( PERMISSION_ARTEFACT, F_OK) != -1)
//    {
//        try
//        {
//            tinyxml2::XMLDocument xml_doc;
//            tinyxml2::XMLError eResult = xml_doc.LoadFile(PERMISSION_ARTEFACT);
//            tinyxml2::XMLElement* root = xml_doc.FirstChildElement("UAPermission")->FirstChildElement("Permission")->FirstChildElement("FlightDetails")->FirstChildElement("FlightParameters")->FirstChildElement("Coordinates");
//            for(tinyxml2::XMLElement* child = root->FirstChildElement(); child!=NULL; child=child->NextSiblingElement())
//            {
//                const char* coordinateLat = child->Attribute("latitude");
//                const char* coordinateLon = child->Attribute("longitude");
//                string latitude(coordinateLat); string longitude(coordinateLon);
//                artefactData.gpsCoordinates.push_back(strtof(coordinateLat,NULL));
//                artefactData.gpsCoordinates.push_back(strtof(coordinateLon,NULL));
//            }
//            // root = xml_doc.FirstChildElement("UAPermission")->FirstChildElement("Permission")->FirstChildElement("FlightDetails")->FirstChildElement("FlightParameters");
//            // const char* maxAltitude = root->Attribute("maxAltitude"); 
//            // artefactData.gpsCoordinates.push_back(strtof(maxAltitude,NULL));
//            artefactData.gpsCoordinates.push_back(90.0);
//            root = xml_doc.FirstChildElement("UAPermission")->FirstChildElement("Permission")->FirstChildElement("FlightDetails")->FirstChildElement("UADetails");
//            const char* uinNo = root->Attribute("uinNo"); artefactData.uinNo = uinNo;
//            // root = xml_doc.FirstChildElement("UAPermission")->FirstChildElement("Permission")->FirstChildElement("Owner")->FirstChildElement("Pilot");
//            // const char* pilotPinHash = root->Attribute("pilotPinHash"); 
//            artefactData.pilotPinHash = "k+4QZ+Sr2niaDmPs1tN6wIsXdjgLKIBzKq9xJk1nRe0=";
//            // root = xml_doc.FirstChildElement("UAPermission")->FirstChildElement("Permission");
//            // const char* paId = root->Attribute("permissionArtefactID"); 
//            artefactData.permissionArtefactID = "123ABC";
//        }
//        catch(const std::exception& e)
//        {
//            std::cerr << e.what() << '\n';
//        }
//    }
//    else {clog << "Permission Artefact Not Present" << endl;}
//    return artefactData;
//}

// // METHOD FOR CHECKSUM VERIFICATION OF A FILE
// static bool CheckSumVerification(string fileName, int receivedCheckSum)
// {
//     OpenSSL_add_all_algorithms();
//     SSLHandler sslHandlerObject;
//     std::string checkSumString = sslHandlerObject.generateFileChecksum(fileName);
//     int checkSumSum = 0;
//     for(int i = 0; i < checkSumString.length(); i++) checkSumSum += int(checkSumString[i]);
//     if (checkSumSum == receivedCheckSum) return true;
//     else return false;
// }


// METHOD TO READ FILES FROM A DIRECTORY
//static vector<std::string> ReadDirectory(const char * dir)
//{
    //vector<std::string> fileName;
    //DIR* dirp = opendir(dir);
    //struct dirent *dp;
    //while((dp=readdir(dirp)) != NULL)
    //{
    //    fileName.push_back(dp->d_name);
    //}
    //closedir(dirp);
    //return fileName;
//}
// METHOD FOR GENERATING MAC ID
//static vector <unsigned char> GenerateMacId(bool UAV)
//{
    //unsigned char* mac = new unsigned char[12];
    //struct ifreq s;
    //int fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    //strcpy(s.ifr_name, UAV ? "eth0" : "enp8s0");       // interface VirtualBox : enp0s3, Laptop : enp8s0:, UAV : eth0
    //if(0 == ioctl(fd, SIOCGIFHWADDR, &s))
    //    for(int i = 0; i < 6; ++i)
    //        sprintf( (char*)&mac[i*2], "%02X", (unsigned char) s.ifr_addr.sa_data[i]);
    //vector <unsigned char> returnMac (&mac[0], &mac[12]);
    //delete[] mac;
    //return returnMac;
//}
// METHOD FOR GENERATING SSID
//static vector<unsigned char> GenerateSSID(bool UAV)
//{
//    if(!UAV)
//    {
//        string simulatedSsid = "DeViCeMoDeLiD";
//        vector<unsigned char> simulatedSSID(simulatedSsid.begin(),simulatedSsid.end());
//        return simulatedSSID;
//    }
//	const int MAX_COMMAND_LINE_LENGTH = 100;
//	char commandLine[ MAX_COMMAND_LINE_LENGTH ] = { 0 };
//	FILE *commandFile = popen( "uci get wireless.@wifi-iface[0].ssid", "r" );
//
//	if( commandFile != NULL )
//	{
//		fgets( commandLine, MAX_COMMAND_LINE_LENGTH, commandFile );
//		fclose( commandFile );
//	}
//
//	const int DEVICE_ID_LENGTH = 6;
//	const unsigned char BROADCAST_DESTINATION_DEVICE_ID[ DEVICE_ID_LENGTH ] =
//		{ '0', '0', '0', '0', '0', '0' };
//	const int commandLineLength = strlen( commandLine );
//	const int deviceSSIDStartIndex = commandLineLength - ( DEVICE_ID_LENGTH +
//		( ( ( commandLineLength != 0 ) && ( commandLine[ commandLineLength - 1 ] == '\n' ) ) ?
//			1 : 0 ) );
//
//	if( ( deviceSSIDStartIndex < 0 ) ||
//		( memcmp( commandLine + deviceSSIDStartIndex,
//			BROADCAST_DESTINATION_DEVICE_ID, DEVICE_ID_LENGTH ) == 0 ) )
//	{
//		throw std::runtime_error( "Invalid device ID." );
//	}
//    string generatedSSID = string( commandLine + deviceSSIDStartIndex, DEVICE_ID_LENGTH );
//    vector<unsigned char> SSID(generatedSSID.begin(),generatedSSID.end());
//	return SSID;
//}

// METHOD FOR CALCULATING DEGREE
static double CalcDegreeDist(double latitude,double longitude)
{
    double degreeDist=0.0;
    if( (latitude!=0) || (longitude!=0) )
    {
        double theta=longitude*(PI/180);
        double lat1= latitude*(PI/180);
        degreeDist = acos(cos(lat1)*cos(theta))*(180/PI);
    }
    return degreeDist;
}

//static size_t FindSize(const char *fileName)
//{
//	struct stat st;
//	
//	if(stat(fileName,&st)==0)
//	{
//		return (st.st_size);
//	}
//	else
//	{
//		return -1;
//	}
//}

//static char *GetFileChar(const char * fileName)
//{
//    char *fileBuffer;
//    long int fileSize = FindSize(fileName);
//    int fd = open(fileName,O_RDONLY);
//    fileBuffer = (char*)mmap(0,fileSize,PROT_READ,MAP_SHARED,fd,0);
//    return fileBuffer;
//}


#endif

