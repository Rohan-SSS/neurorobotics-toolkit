/*
* NOT FOR HYBRID : modes are different ! 
*/
#pragma once
#ifndef TELEMETRYVALIDATOR_H
#define TELEMETRYVALIDATOR_H
#include <ctime>
#include <time.h>
#include <unistd.h> //TODO : Check replacement of this header
//#include <io.h>
#include <stdint.h>
#include "stdio.h"
#include "sensors/UtilClasses.h"
#include <mutex>
#include <string.h>
//#include "json.h"
//#include "parson.h"

using namespace std;

class TelemetryValidator 
{
    private :
    long startTime,endTime,currentTime,startSystemTime,currentSystemTime;
    float altitude,time,setAlt;
    unsigned char *ptr;unsigned char chksm = 0x20;
    float altitudeLog,timeLog;
    class Polygon polygon;
    class FlightLogData FlightDataGCS;class FlightLogData FlightDataUAV;
    unsigned char rxR_buffer[512];unsigned char rxf_buffer [512]; unsigned char rx_buffer [512];
    bool isPermArtPresent=true;bool isWPATpacketValid=true;
    unsigned char rxf_state=0,rxf_sub_state=0,rxf_packet_bytes=0,rxf_chksum=0,rxf_chksum_recv=0,rxf_packet_type=0;
    unsigned char rx_state=0, packet_type=0, packet_bytes=0, sub_state=0, chksum=0, chksum_recv=0,rc_count=0;
    double PacketsReceived,BytesReceived;unsigned char id_received=0, communication_id=0;
    unsigned short rxf_id; long gpsCount=0; // changed to unsigned short __uint16_
    unsigned char RX_PACKET_FN;unsigned const char PROTOCOL_VERSION =0;
    bool isConnected = false,FirstPacketReceived = false;int16_t Alt;
    float homeAlt=0,homeLat=0,homeLong=0;
    bool result_PIP= true,result_ALT = true,result_TIME = true,logged = false;
    struct tm timeStructure ;

	//commented by vivek
    //ofstream FlightDataLogFile;ifstream inBound;
    //JsonDS jsonDs;
    //ofstream programLog; bool activateProgramLog = true;
    //ofstream LogDataTv; time_t LogTimeTv;
    bool littleEndian,startup=false,takeoffReceived = false;
    

    public : 
    const unsigned char TX_Home[7] = {0x55, 0x42, 0, 0, 0, 0x42, 0xAA}; // SOP,packet_type,0,0,0,checksum,EOP
    unsigned char TX_SetAlt[12] = {0x55, 0x20, 0x05, 0x00, 0x00, 0x00, 0x42, 0x70, 0x00, 0x00, 0xD2, 0xAA};
    //unsigned char TX_Land[24] = {0x55, 0x22, 0x11, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0xAA};
    TelemetryValidator();
    void TelemetryValidatorInit(); 
    bool init = false;
	std::mutex teleMutex;
    ~TelemetryValidator();

    // FROM GCS
    public :
    bool isReceivedPacketValid(unsigned char* _packetData,int packetBytesCount);
    private :
    void TX(unsigned char data);
    void rxSetWaypoint(void);
    void rxAltAirspeedPacket(void);
    void rxTakeoffLandPacket(void);
    void checkTakeOffPos(void);
    // FROM AUTOPILOT 
    public : 
    bool isTransmittedPacketValid(unsigned char *_packetData, int packetBytesCount, bool &execute_RTH, bool &execute_ALT, bool &execute_TIME);
    void createLogFile(); bool isLogFileCreated = false;
    void closeLogFile();
	FlightLogData getCurrentUavData();
    void getCurrentUavData(FlightLogData& a);
    private :
    void RX (unsigned char data);
    bool isFlightDataValid();
    void logFlightData(string logMessage);
    void rxGpsPacket ();
    void rxAttitudePacket ();
    void rxSensorPacket();
    long calcTimeElapsed();
	long convertDateTimeToEpoch(uint32_t date, uint32_t time);
};


#endif
