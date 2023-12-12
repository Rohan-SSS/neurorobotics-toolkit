#include "sensors/TelemetryValidator.h"
#include "sensors/TelemetryValidatorConstants.h"
using namespace TelemetryValidatorConstants;

std::string logFileNameJson="";

TelemetryValidator::~TelemetryValidator()
{
    //if(FlightDataLogFile.is_open()){FlightDataLogFile.close();}
}

TelemetryValidator::TelemetryValidator()
{

    //if(access(BINARY_BOUND ,F_OK)==-1) //Bound.bin is the file derived from PA which has count and coordinates of vertices,altitude and time reference 
    //{isPermArtPresent =false;init=false;/*system("killall bfcom4");*/return;}
    

    //usleep(2000000); // time given to load file ( can be removed if not required anymore )
    
    unsigned short testVal = UCHAR_MAX;
    littleEndian = ( *reinterpret_cast< unsigned char * >( &testVal ) == UCHAR_MAX );
    
    FlightDataGCS.groundSpeed = 0.0;
    FlightDataGCS.altitudeAGL=0;
    FlightDataGCS.altitudeMSL=0;
    FlightDataGCS.latitude=0;
    FlightDataGCS.longitude=0;
    FlightDataGCS.mode = 0;
    FlightDataGCS.date = 0;
    FlightDataGCS.time = 0;
	FlightDataGCS.Az = 0;
	FlightDataGCS.Ay = 0;
	FlightDataGCS.Ax = 0;
	FlightDataGCS.Bz = 0;
	FlightDataGCS.By = 0;
	FlightDataGCS.Bx = 0;
	FlightDataGCS.R = 0;
	FlightDataGCS.Q = 0;
	FlightDataGCS.P = 0;
	FlightDataGCS.dRoll = 0;
	FlightDataGCS.dPitch = 0;
	FlightDataGCS.dYaw = 0;
    FlightDataGCS.float_dYaw = 0.0;
    FlightDataGCS.float_dPitch = 0.0;
    FlightDataGCS.float_dRoll = 0.0;


    FlightDataUAV.groundSpeed = 0.0;
    FlightDataUAV.altitudeAGL=0;
    FlightDataUAV.altitudeMSL=0;
    FlightDataUAV.latitude=0;
    FlightDataUAV.longitude=0;
    FlightDataUAV.mode = 0;
    FlightDataUAV.date = 0;
    FlightDataUAV.time = 0;
	FlightDataUAV.Az = 0;
	FlightDataUAV.Ay = 0;
	FlightDataUAV.Ax = 0;
	FlightDataUAV.Bz = 0;
	FlightDataUAV.By = 0;
	FlightDataUAV.Bx = 0;
	FlightDataUAV.R = 0;
	FlightDataUAV.Q = 0;
	FlightDataUAV.P = 0;
	FlightDataUAV.dRoll = 0;
	FlightDataUAV.dPitch = 0;
	FlightDataUAV.dYaw = 0;
    FlightDataUAV.float_dYaw = 0.0;
    FlightDataUAV.float_dPitch = 0.0;
    FlightDataUAV.float_dRoll = 0.0;

    takeoffReceived = false;
    isPermArtPresent = true;
    init = true; 
}  

long TelemetryValidator::calcTimeElapsed()
{
    return 0;
    /* 
    currentSystemTime = ::time(nullptr);
    return (currentSystemTime - startSystemTime);
    */ // Use if GPS Time disabled
}

long TelemetryValidator::convertDateTimeToEpoch(uint32_t date,uint32_t time)
{
    timeStructure.tm_mday = (int)date/10000;
    timeStructure.tm_mon = (int)(int(date/100)-(timeStructure.tm_mday*100));
    timeStructure.tm_year = (int)(date-(timeStructure.tm_mday*10000 + timeStructure.tm_mon*100));
    timeStructure.tm_hour = (int) ((time/10000));
    timeStructure.tm_min = (int)(int(time/100)-(timeStructure.tm_hour*100));
    timeStructure.tm_sec = (int)(int(time) - (timeStructure.tm_hour*10000 + timeStructure.tm_min*100));
    timeStructure.tm_mon-=1;
    timeStructure.tm_year+=100;
    return (long)mktime(&timeStructure);
}

bool TelemetryValidator::isReceivedPacketValid(unsigned char* _packetData,int packetBytesCount)
{
    for (int i=0;i<packetBytesCount;i++)
    {TX(_packetData[i]);} 
    if(!isWPATpacketValid)
    {isWPATpacketValid=true;return false;}
    else return true;
}

void TelemetryValidator::TX(unsigned char data)
{
    if(rx_state==0 && data==0x55) rx_state = 1;
    else if(rx_state==1) 
    { 
        packet_type = data;
        sub_state = 0;
        chksum = packet_type;
        if(packet_type==0x01) rx_state = 0;
        else if(packet_type==0xF0) rx_state = 0;
        else rx_state = 2;
    }
    else if(rx_state==2) 
    {
        packet_bytes = data;
        if(packet_bytes>252) rx_state=0;
        else rx_state = 3;
    }
    else if(rx_state==3)
    {
        id_received = data;
        rx_state = 4;
    }
    else if(rx_state==4)
    {
        id_received += data<<8;
        rx_state = 12;
    }
    else if(rx_state==12)
    {
        if(sub_state < packet_bytes) { rxR_buffer[sub_state] = data; chksum+=data; sub_state++; }
        else if (sub_state==packet_bytes) { chksum_recv = data; sub_state++; }
        else if (sub_state==(packet_bytes+1))
        {
            if(data==0xAA && chksum==chksum_recv)
            {
                if(id_received==communication_id)
                {
                    switch (packet_type)
                    {
                        case GCS_SetWaypointPacket : rxSetWaypoint();            //WAYPOINT
                        break;
                        case GCS_AltAirspeedPacket : rxAltAirspeedPacket();      //ALTITUDE
                        break;
                        case GCS_TakeoffLandPacket : rxTakeoffLandPacket();      //TAKE OFF
                        break;
                        default:;
                    }
                }
                else if(communication_id==0) communication_id=id_received;
            }
        rx_state=0;
        }
        else rx_state=0;
    }
    else rx_state=0;
}

void TelemetryValidator::rxSetWaypoint(void)
{
    if(!isPermArtPresent){isWPATpacketValid=false;return;}
    memcpy(&polygon.uavPos.lat,rxR_buffer + GCS_SetWaypointPacket_UavLat,SIZE_FLOAT);
    memcpy(&polygon.uavPos.lon,rxR_buffer + GCS_SetWaypointPacket_UavLon,SIZE_FLOAT);
    if (littleEndian){polygon.uavPos.lat=ReverseFloat(polygon.uavPos.lat);polygon.uavPos.lon = ReverseFloat(polygon.uavPos.lon);}
    if(polygon.PIP()==0)
    {
        isWPATpacketValid=false;
        cout << "Invalid Co-ordinates" << endl;
    }
}

void TelemetryValidator::rxAltAirspeedPacket(void)
{
    if(!isPermArtPresent){isWPATpacketValid=false;return;}
    uint8_t check;
    memcpy(&check,rxR_buffer + GCS_AltAirspeedPacket_Check,SIZE_BYTE);
    if(check==0)
    {
        memcpy(&FlightDataGCS.altitudeAGL,rxR_buffer + GCS_AltAirspeedPacket_AGL,SIZE_FLOAT);
        if(littleEndian)
        {FlightDataGCS.altitudeAGL = (ReverseFloat(FlightDataGCS.altitudeAGL)+homeAlt);}
        else {FlightDataGCS.altitudeAGL += homeAlt;}
        if(FlightDataGCS.altitudeAGL > altitude)
        {
            isWPATpacketValid = false; 
            cout<<"Invalid Altitude" << endl;
        }
    }
}

void TelemetryValidator::rxTakeoffLandPacket(void)
{
    if(!isPermArtPresent){isWPATpacketValid=false;return;}
    uint8_t type;
    memcpy(&type,rxR_buffer+GCS_TakeoffLandPacket_Type,SIZE_BYTE);
    if(type == 0)
    {
        if(takeoffReceived==false && startup == true)
        {         
            homeAlt = FlightDataUAV.altitudeMSL; homeLat=FlightDataUAV.latitude; homeLong=FlightDataUAV.longitude;
            cout << "Home Alt:" << homeAlt << endl;
            if(homeAlt < -10 || homeAlt > 60)  // limit set to remove error temp ; NEED TO DISCUSS WITH TEAM ! ! !
            {
                isWPATpacketValid = false;
                return;
            }
            checkTakeOffPos();
            if(isWPATpacketValid)
            {
                takeoffReceived=true;
                //if(!isLogFileCreated) createLogFile();
                //cout << "Take-off" << endl;
                //logFlightData("Take-Off");
                altitude-=homeAlt; 
                setAlt= altitude-10.0;
                ptr = (unsigned char*)&setAlt;
                memcpy(TX_SetAlt + 6, ptr, SIZE_FLOAT);
                for (int i = 0; i < 4; i++)
                {
                    chksm += *ptr;
                    ptr++;
                }
                TX_SetAlt[10] = chksm;
                ptr=(unsigned char*)&setAlt;chksm=0x20;
            }
        }
        else
        {
            checkTakeOffPos();
        }
    }
}

void TelemetryValidator::checkTakeOffPos(void)
{
    polygon.uavPos.lat=FlightDataUAV.latitude;polygon.uavPos.lon=FlightDataUAV.longitude;
    memcpy(&FlightDataGCS.altitudeAGL,rxR_buffer + GCS_TakeoffLandPacket_AGL,SIZE_FLOAT);
    if(littleEndian)
    {FlightDataGCS.altitudeAGL=(ReverseFloat(FlightDataGCS.altitudeAGL)+ homeAlt);}
    else {FlightDataGCS.altitudeAGL += homeAlt;}
    if(polygon.PIP()==0)
    {   
        isWPATpacketValid=false; 
        cout <<"Invalid Co-ordinates Take-off"<<endl;
    }
    if(FlightDataGCS.altitudeAGL > altitude)
    {
        isWPATpacketValid = false;
        cout<<"Invalid Altitude Take-off"<<endl;
    }
    if((currentTime + calcTimeElapsed()) > endTime || (currentTime + calcTimeElapsed()) < startTime )
    {
        isWPATpacketValid = false;
        cout<<"Invalid Time Take-off"<<endl;
    }
}

bool TelemetryValidator::isTransmittedPacketValid(unsigned char *_packetData, int packetBytesCount, bool &execute_RTH, bool &execute_ALT, bool &execute_TIME)
{  
    for (int i=0;i<packetBytesCount;i++)
    {RX(_packetData[i]);}
    //if(FlightDataUAV.mode == 7) cout << "Received Land Mode" << endl;
    if ((FlightDataUAV.mode != 0) && (FlightDataUAV.mode != 6) && (FlightDataUAV.mode != 7) && (gpsCount >= 2))
    {  
        gpsCount=0;
        if (!isPermArtPresent){return false;}
        if (!isFlightDataValid())
        {  
            if (!result_PIP) {execute_RTH = true;}
            if (!result_ALT) {execute_ALT = true;}
            if (!result_TIME){execute_TIME = true;}
            return false;
        }
    }
    else if((FlightDataUAV.mode == 0)&&(takeoffReceived==true))
    {      
        takeoffReceived=false;startup=false; 
        //logFlightData("Landed"); 
        //cout << "Landed" << endl;
        //closeLogFile();
    }
    return true;
} 

void TelemetryValidator::RX (unsigned char data)
{
    try
        {
            if (rxf_state == 0 && data == 0x55) rxf_state = 1;
            else if (rxf_state == 1)
            {rxf_packet_type = data;rxf_sub_state = 0;rxf_chksum = rxf_packet_type;rxf_chksum += PROTOCOL_VERSION;rxf_state = 2;}
                
            else if (rxf_state == 2)
            {
                rxf_packet_bytes = data;
                if (rxf_packet_bytes > 252) rxf_state = 0;
                else rxf_state = 3;
            }
                
            else if (rxf_state == 3)
            {rxf_id = data;rxf_state = 4;}
                
            else if (rxf_state == 4)
            {rxf_id += (unsigned short)(data << 8);rxf_state = 10;}

            else if (rxf_state == 10)
            {
                if (rxf_sub_state < rxf_packet_bytes) { rxf_buffer[rxf_sub_state] =data; rxf_chksum += data; rxf_sub_state++; }
                else if (rxf_sub_state == rxf_packet_bytes) { rxf_chksum_recv = data; rxf_sub_state++; }
                else if (rxf_sub_state == (rxf_packet_bytes + 1))
                {
                    if (data == 0xAA && rxf_chksum == rxf_chksum_recv)
                    {
                        for (unsigned short i = 0; i < rxf_packet_bytes; i++) rx_buffer[i] = rxf_buffer[rxf_packet_bytes - 1 - i];
                        PacketsReceived++;
                        BytesReceived += rxf_packet_bytes;
                        try
                        {
                            switch(rxf_packet_type)
                            {
                                case UAV_GpsPacket      : rxGpsPacket();
                                break;
                                case UAV_AttitudePacket : rxAttitudePacket();
                                break;
                                case UAV_SensorPacket   : rxSensorPacket();
                                default: ;
                            }    
                        }catch (...){;}
                        isConnected = true;
                        FirstPacketReceived = true;
                    }
                    rxf_state = 0;
                }
                else rxf_state = 0;
            }
            else rxf_state = 0;
        }
    catch(...){;}
}

bool TelemetryValidator::isFlightDataValid()
{ 
    result_PIP= true;
    result_ALT = true;
    result_TIME = true;
    polygon.uavPos.lat=FlightDataUAV.latitude;polygon.uavPos.lon=FlightDataUAV.longitude;
    // checking Geo - fence 
    if(polygon.PIP()==0)
    {
        if(polygon.PIP_LOG()==0)
        {
            if(takeoffReceived)
            {
                //logFlightData("Geofence Breach");
                //cout <<"Geofence Breach" << endl;
            }
        }
        result_PIP=false;
    }
    //checking altitude
    if(FlightDataUAV.altitudeAGL > altitude)
    {
        if(FlightDataUAV.altitudeAGL > altitudeLog)
        {
            if(takeoffReceived)
            {
                //logFlightData("Altitude Breach");
                //cout << "Altitude Breach" << endl;
            }
        }
        
        result_ALT = false;
    }
    //checking time 
    if(currentTime + calcTimeElapsed() > endTime )
    {
        if(currentTime + calcTimeElapsed()> endTime )
            {
                if(takeoffReceived)
                {
                    //logFlightData("Time Breach");
                    //cout << "TIme Breach" << endl;
                }
            }     
        result_TIME= false;
    }
    return (result_PIP && result_ALT && result_TIME);
}

void TelemetryValidator::rxGpsPacket ()
{
    memcpy(&FlightDataUAV.mode, rx_buffer + UAV_GpsPacket_Mode,SIZE_BYTE);
    //std::cout << "Mode from telemetry.cpp " << (int)FlightDataUAV.mode << std::endl;
    memcpy(&FlightDataUAV.longitude, rx_buffer + UAV_GpsPacket_Lon ,SIZE_FLOAT);
    memcpy(&FlightDataUAV.latitude , rx_buffer + UAV_GpsPacket_Lat ,SIZE_FLOAT);
    //littleEndian = true;
    if(!littleEndian)
    {
        FlightDataUAV.longitude = ReverseFloat(FlightDataUAV.longitude);
        FlightDataUAV.latitude = ReverseFloat(FlightDataUAV.latitude);
    }
    if(FlightDataUAV.mode == 13)
    {
        startup=true;
    }
    gpsCount++;
    //std::cout << "Memcopy done" << (int)FlightDataUAV.mode << std::endl;


}

void TelemetryValidator::rxAttitudePacket ()
{
    memcpy(&Alt, rx_buffer + UAV_AttitudePacket_AGL,SIZE_SHORT);
    memcpy(&FlightDataUAV.dYaw, rx_buffer + UAV_AttitudePacket_r, SIZE_SHORT);
    memcpy(&FlightDataUAV.dPitch, rx_buffer + UAV_AttitudePacket_q, SIZE_SHORT);
    memcpy(&FlightDataUAV.dRoll, rx_buffer + UAV_AttitudePacket_p, SIZE_SHORT);
    //littleEndian = true;
    if(!littleEndian)
    {
        Alt = ReverseInt16(Alt);
        FlightDataUAV.dYaw = ReverseInt16(FlightDataUAV.dYaw);
        FlightDataUAV.dPitch = ReverseInt16(FlightDataUAV.dPitch);
        FlightDataUAV.dRoll = ReverseInt16(FlightDataUAV.dRoll);
    }
    FlightDataUAV.altitudeAGL=(float)Alt/10.0;
    FlightDataUAV.float_dYaw = (float)(FlightDataUAV.dYaw * 180) / (1000 * PI);    
    FlightDataUAV.float_dPitch = (float)(FlightDataUAV.dPitch * 180) / (1000 * PI);
    FlightDataUAV.float_dRoll = (float)(FlightDataUAV.dRoll * 180) / (1000 * PI);
}

/*
private void rx_sensor_packet()
       {
           rx_buf_ptr = 0;
 
           if (FlightData.ModeHybrid == 0)
           {
               FlightData.WindDirection = extract_float() * 180 / Math.PI;//0
               FlightData.WindMagnitude = extract_float();//4
           }
           else
           {
               extract_float();
               extract_float();
           }
 
           FlightData.CommStatus.GSperSec = extract_uint8();//8
           FlightData.CommStatus.RCperSec = extract_uint8();//9
 
           FlightData.Sensors.HealthHistory = extract_uint32();//10
           FlightData.Sensors.Health = extract_uint32();//14
 
           FlightData.GPS_Mix.Heading = extract_int16() * 180.0 / (10000.0 * Math.PI);//18
           FlightData.GPS_Mix.GroundSpeed = extract_int16() / 100.0;//20
           FlightData.GPS_Mix.AltitudeAGL = extract_float();//22
           FlightData.GPS_Mix.AltitudeMSL = extract_float();//26
           FlightData.GPS_Mix.Longitude = extract_float();//30
           FlightData.GPS_Mix.Latitude = extract_float();//34
 
           for (byte i = 0; i < 2; i++)
           {
               FlightData.GPS[i].HDOP = extract_uint16() / 100.0;//38,61
               FlightData.GPS[i].NoSatellites = extract_uint8();//40,63
               FlightData.GPS[i].FixType = (FlightData.GPS[i].NoSatellites & 0xE0) >> 5;
               FlightData.GPS[i].NoSatellites &= 0x1F;
               FlightData.GPS[i].Heading = extract_int16() * 180.0 / (10000.0 * Math.PI);//41,64
               FlightData.GPS[i].GroundSpeed = extract_int16() / 100.0;//43,66
               FlightData.GPS[i].AltitudeAGL = extract_float();//45,68
               FlightData.GPS[i].AltitudeMSL = extract_float();//49,72
               FlightData.GPS[i].Longitude = extract_float();//53,76
               FlightData.GPS[i].Latitude = extract_float();//57,80
           }
 
           FlightData.GPS_Mix.FusedHDOPMin = Math.Min(FlightData.GPS[0].HDOP, FlightData.GPS[1].HDOP);
           FlightData.GPS_Mix.FusedHDOPMax = Math.Max(FlightData.GPS[0].HDOP, FlightData.GPS[1].HDOP);
           FlightData.GPS_Mix.FusedNoSatellites = Math.Max(FlightData.GPS[0].NoSatellites, FlightData.GPS[1].NoSatellites);
 
           FlightData.GPS_Mix.HDOP = FlightData.GPS_Mix.FusedHDOPMin;
           FlightData.GPS_Mix.NoSatellites = FlightData.GPS_Mix.FusedNoSatellites;
 
           FlightData.AvailableSafeDistance = extract_float();//84
 
           FlightData.SensorVoltage = extract_uint16() / 100.0;//88
           FlightData.uC_Voltage = extract_uint16() / 100.0;//90
           FlightData.VoltageMid = extract_uint16() / 100.0;//92
           FlightData.Voltage = extract_uint16() / 100.0;//94
 
           if (FlightData.Voltage < 0.1) FlightData.VoltageImbalance = 0;
           else { FlightData.VoltageImbalance = Math.Abs(100 * (FlightData.Voltage - FlightData.VoltageMid * 2) / FlightData.Voltage); }
 
           FlightData.Sensors.Tx = extract_int16();//96
           FlightData.Sensors.T2 = extract_int16();//98
           FlightData.Sensors.T1 = extract_int16();//100
           FlightData.Sensors.T0 = extract_int16();//102
 
           FlightData.Sensors.AGL2 = extract_float();//104
           FlightData.Sensors.AGL1 = extract_float();//108
           FlightData.Sensors.AGL0 = extract_float();//112
           FlightData.Sensors.MSL2 = extract_float();//116
           FlightData.Sensors.MSL1 = extract_float();//120
           FlightData.Sensors.MSL0 = extract_float();//124
 
           FlightData.ThrottleAdjust = (double)extract_int16() / 100;//128
 
           FlightData.AltitudeCorrectionUAV = extract_int16();//130
 
           FlightData.Sensors.RBias = (double)extract_int16() / 100;//132
           FlightData.Sensors.QBias = (double)extract_int16() / 100;//134
           FlightData.Sensors.PBias = (double)extract_int16() / 100;//136
 
           FlightData.Sensors.Az = extract_int16();//138
           FlightData.Sensors.Ay = extract_int16();//140
           FlightData.Sensors.Ax = extract_int16();//142
 
           if (FlightData.Sensors.AccelerometerCalibration.InProgress)
           {
               FlightData.Sensors.AccelerometerCalibration.ReadingsSensor++;
 
               FlightData.Sensors.AccelerometerCalibration.AxLPF += 0.1 * (FlightData.Sensors.Ax - FlightData.Sensors.AccelerometerCalibration.AxLPF);
               FlightData.Sensors.AccelerometerCalibration.AyLPF += 0.1 * (FlightData.Sensors.Ay - FlightData.Sensors.AccelerometerCalibration.AyLPF);
               FlightData.Sensors.AccelerometerCalibration.AzLPF += 0.1 * (FlightData.Sensors.Az - FlightData.Sensors.AccelerometerCalibration.AzLPF);
 
               if (Math.Abs(FlightData.Sensors.Ax - FlightData.Sensors.AccelerometerCalibration.AxLPF) > 10) FlightData.Sensors.AccelerometerCalibration.AxLPF += 0.7 * (FlightData.Sensors.Ax - FlightData.Sensors.AccelerometerCalibration.AxLPF);
               if (Math.Abs(FlightData.Sensors.Ay - FlightData.Sensors.AccelerometerCalibration.AyLPF) > 10) FlightData.Sensors.AccelerometerCalibration.AyLPF += 0.7 * (FlightData.Sensors.Ay - FlightData.Sensors.AccelerometerCalibration.AyLPF);
               if (Math.Abs(FlightData.Sensors.Az - FlightData.Sensors.AccelerometerCalibration.AzLPF) > 10) FlightData.Sensors.AccelerometerCalibration.AzLPF += 0.7 * (FlightData.Sensors.Az - FlightData.Sensors.AccelerometerCalibration.AzLPF);
           }
 
           FlightData.Sensors.Bz = extract_int16();//144
           FlightData.Sensors.By = extract_int16();//146
           FlightData.Sensors.Bx = extract_int16();//148
 
           FlightData.Sensors.R = extract_int16();//150
           FlightData.Sensors.Q = extract_int16();//152
           FlightData.Sensors.P = extract_int16();//154
 
           FlightData.NumberOfFlights = extract_uint16();//156
 
           FlightData.GPS_Mix.Date = extract_uint32();//158
           FlightData.GPS_Mix.Time = extract_uint32();//162
 
           FlightData.Sensors.Radar.L = extract_int16();//166
           FlightData.Sensors.Radar.B = extract_int16();//168
           FlightData.Sensors.Radar.R = extract_int16();//170
           FlightData.Sensors.Radar.F = extract_int16();//172
 
           FlightData.FirmwareVersion = (double)extract_int16() / 100;//174
 
           if (!APInfoWritten)
           {
               Logger.WriteAPInfo();
               APInfoWritten = true;
           }
 
           FlightData.Sensors.External.LR = extract_int16();//176
           FlightData.Sensors.External.Tx = extract_int16();//178
           FlightData.Sensors.External.Az = extract_int16();//180
           FlightData.Sensors.External.Ay = extract_int16();//182
           FlightData.Sensors.External.Ax = extract_int16();//184
           FlightData.Sensors.External.Bz = extract_int16();//186
           FlightData.Sensors.External.By = extract_int16();//188
           FlightData.Sensors.External.Bx = extract_int16();//190
           FlightData.Sensors.External.R = extract_int16();//192
           FlightData.Sensors.External.Q = extract_int16();//194
           FlightData.Sensors.External.P = extract_int16();//196
 
           if (extract_uint16() == 0xBABA)
           {
               FlightData.Camera.Type = extract_uint16();//198
               FlightData.Camera.Health = extract_uint8();//200
 
               try
               {
                   if (FlightData.Camera.PayloadNames.ContainsKey(FlightData.Camera.Type))
                   {
                       FlightData.Camera.Name = FlightData.Camera.PayloadNames[FlightData.Camera.Type];
                   }
                   else FlightData.Camera.Name = null;
               }
               catch { }
           }
       }
*/

void TelemetryValidator::rxSensorPacket()
{
    memcpy(&FlightDataUAV.altitudeMSL, rx_buffer + UAV_SensorPacket_MSL ,SIZE_FLOAT);
    memcpy(&FlightDataUAV.date, rx_buffer + UAV_SensorPacket_Date, SIZE_INT);
    memcpy(&FlightDataUAV.time, rx_buffer + UAV_SensorPacket_Time, SIZE_INT); // date time in utc
    memcpy(&FlightDataUAV.Az, rx_buffer + UAV_SensorPacket_Az, SIZE_SHORT);
    memcpy(&FlightDataUAV.Ay, rx_buffer + UAV_SensorPacket_Ay, SIZE_SHORT);
    memcpy(&FlightDataUAV.Ax, rx_buffer + UAV_SensorPacket_Ax, SIZE_SHORT);
    memcpy(&FlightDataUAV.Bz, rx_buffer + UAV_SensorPacket_Bz, SIZE_SHORT);
    memcpy(&FlightDataUAV.By, rx_buffer + UAV_SensorPacket_By, SIZE_SHORT);
    memcpy(&FlightDataUAV.Bx, rx_buffer + UAV_SensorPacket_Bx, SIZE_SHORT);
    memcpy(&FlightDataUAV.R, rx_buffer + UAV_SensorPacket_R, SIZE_SHORT);
    memcpy(&FlightDataUAV.Q, rx_buffer + UAV_SensorPacket_Q, SIZE_SHORT);
    memcpy(&FlightDataUAV.P, rx_buffer + UAV_SensorPacket_P, SIZE_SHORT);
    //littleEndian = true;
    if (!littleEndian)
    {
        FlightDataUAV.Az = ReverseInt16(FlightDataUAV.Az);
        FlightDataUAV.Ay = ReverseInt16(FlightDataUAV.Ay);
        FlightDataUAV.Ax = ReverseInt16(FlightDataUAV.Ax);
        FlightDataUAV.Bz = ReverseInt16(FlightDataUAV.Bz);
        FlightDataUAV.By = ReverseInt16(FlightDataUAV.By);
        FlightDataUAV.Bx = ReverseInt16(FlightDataUAV.Bx);
        FlightDataUAV.R = ReverseInt16(FlightDataUAV.R);
        FlightDataUAV.Q = ReverseInt16(FlightDataUAV.Q);
        FlightDataUAV.P = ReverseInt16(FlightDataUAV.P);
        FlightDataUAV.altitudeMSL = ReverseInt16(FlightDataUAV.altitudeMSL);
        FlightDataUAV.date = ReverseInt32(FlightDataUAV.date);
        FlightDataUAV.time = ReverseInt32(FlightDataUAV.time);

    }
    currentTime = convertDateTimeToEpoch(FlightDataUAV.date, FlightDataUAV.time);
    currentTime+=19800; // Converting to IST

    //memcpy(&FlightDataUAV.altitudeMSL, rx_buffer + UAV_SensorPacket_MSL ,SIZE_FLOAT);
    // FlightData.GPS[i].HDOP = extract_uint16() / 100.0;//38,61
    // FlightData.GPS[i].NoSatellites = extract_uint8();//40,63
    // FlightData.GPS[i].FixType = (FlightData.GPS[i].NoSatellites & 0xE0) >> 5;
    // FlightData.GPS[i].NoSatellites &= 0x1F;
    // FlightData.GPS[i].Heading = extract_int16() * 180.0 / (10000.0 * Math.PI);//41,64
    // FlightData.GPS[i].GroundSpeed = extract_int16() / 100.0;//43,66
    // FlightData.GPS[i].AltitudeAGL = extract_float();//45,68
    // FlightData.GPS[i].AltitudeMSL = extract_float();//49,72
    // FlightData.GPS[i].Longitude = extract_float();//53,76
    // FlightData.GPS[i].Latitude = extract_float();//57,80
    // const int UAV_GPS_1_Heading = 41;
    // const int UAV_GPS_2_Heading = 64;

    // const int UAV_GPS_1_AltitudeMSL = 49;
    // const int UAV_GPS_2_AltitudeMSL = 72;

    // const int UAV_GPS_1_HDOP = 41;
    // const int UAV_GPS_2_HDOP = 64;

    // const int UAV_GPS_1_GroundSpeed = 43;
    // const int UAV_GPS_2_GroundSpeed = 66;

    // const int UAV_GPS_1_Longitude = 53;
    // const int UAV_GPS_2_Longitude = 76;

    // const int UAV_GPS_1_Latitude = 57;
    // const int UAV_GPS_2_Latitude = 80;

    // const int UAV_GPS_1_AltitudeAGL = 45;
    // const int UAV_GPS_2_AltitudeAGL = 68;

    short heading_1(0),heading_2(0);
    memcpy(&heading_1, rx_buffer + UAV_GPS_1_Heading ,SIZE_SHORT);
    FlightDataUAV.sensorData[0].heading =  heading_1*180.0 / (10000.0 * 3.141593);
    memcpy(&heading_2, rx_buffer + UAV_GPS_2_Heading ,SIZE_SHORT);
    FlightDataUAV.sensorData[1].heading = heading_2*180.0 / (10000.0 * 3.141593);

    
    memcpy(&FlightDataUAV.sensorData[0].altMSL, rx_buffer + UAV_GPS_1_AltitudeMSL ,SIZE_FLOAT);
    memcpy(&FlightDataUAV.sensorData[1].altMSL, rx_buffer + UAV_GPS_2_AltitudeMSL ,SIZE_FLOAT);

    short hdop_1(0),hdop_2(0);
    memcpy(&hdop_1, rx_buffer + UAV_GPS_1_HDOP ,SIZE_SHORT);
    FlightDataUAV.sensorData[0].hdop = hdop_1/100.0;
    memcpy(&hdop_2, rx_buffer + UAV_GPS_2_HDOP ,SIZE_SHORT);
    FlightDataUAV.sensorData[1].hdop = hdop_2/100.0;
    //std::cout << hdop_1 << "," << hdop_2 << std::endl;
    short gs_1(0),gs_2(0);
    memcpy(&gs_1, rx_buffer + UAV_GPS_1_GroundSpeed ,SIZE_SHORT);
    FlightDataUAV.sensorData[0].groundSpeed = gs_1/100.0;

    memcpy(&gs_2, rx_buffer + UAV_GPS_2_GroundSpeed ,SIZE_SHORT);
    FlightDataUAV.sensorData[1].groundSpeed = gs_2/100.0;    
    memcpy(&FlightDataUAV.sensorData[0].latitude, rx_buffer + UAV_GPS_1_Latitude ,SIZE_FLOAT);    
    memcpy(&FlightDataUAV.sensorData[1].latitude, rx_buffer + UAV_GPS_2_Latitude ,SIZE_FLOAT);
    memcpy(&FlightDataUAV.sensorData[0].longitude, rx_buffer + UAV_GPS_1_Longitude ,SIZE_FLOAT);
    memcpy(&FlightDataUAV.sensorData[1].longitude, rx_buffer + UAV_GPS_2_Longitude ,SIZE_FLOAT);
    //std::cout << FlightDataUAV.sensorData[0].latitude << "," << FlightDataUAV.sensorData[0].longitude << "," << FlightDataUAV.sensorData[1].latitude << "," << FlightDataUAV.sensorData[1].longitude << std::endl;
    memcpy(&FlightDataUAV.sensorData[0].altAGL, rx_buffer + UAV_GPS_1_AltitudeAGL ,SIZE_FLOAT);
    memcpy(&FlightDataUAV.sensorData[1].altAGL, rx_buffer + UAV_GPS_2_AltitudeAGL ,SIZE_FLOAT);
    //UAV_GPS_1_NoOfSatellites
    
    char num_sat_gps_1(0), num_sat_gps_2(0);
    memcpy(&num_sat_gps_1, rx_buffer + UAV_GPS_1_NoOfSatellites ,SIZE_BYTE);
    char fixType = (num_sat_gps_1 & 0xE0) >> 5;
    num_sat_gps_1 &= 0x1F;
    FlightDataUAV.sensorData[0].numSattelites = (float)num_sat_gps_1;
    
    memcpy(&num_sat_gps_2, rx_buffer + UAV_GPS_2_NoOfSatellites ,SIZE_BYTE);
    fixType = (num_sat_gps_2 & 0xE0) >> 5;
    num_sat_gps_2 &= 0x1F;
    FlightDataUAV.sensorData[1].numSattelites = (float)num_sat_gps_2;

    //std::cout << (int)num_sat_gps_1 << "," << (int)num_sat_gps_2 << std::endl;
        
}

void TelemetryValidator::getCurrentUavData(FlightLogData& a)
{

    
    a.groundSpeed = 0.0;
    a.altitudeAGL=0;
    a.altitudeMSL=0;
    a.latitude=0;
    a.longitude=0;
    a.mode = 0;
    a.date = 0;
    a.time = 0;
	a.Az = 0;
	a.Ay = 0;
	a.Ax = 0;
	a.Bz = 0;
	a.By = 0;               
	a.Bx = 0;
	a.R = 0;
	a.Q = 0;
	a.P = 0;
	a.dRoll = 0;
	a.dPitch = 0;
	a.dYaw = 0;
    a.float_dYaw = 0.0;
    a.float_dPitch = 0.0;
    a.float_dRoll = 0.0;
    a = FlightDataUAV;
    //std::cout << FlightDataUAV.sensorData[0].hdop << "," << FlightDataUAV.sensorData[1].hdop << std::endl;
    //return a;
}

FlightLogData TelemetryValidator::getCurrentUavData()
{

    FlightLogData a;
    a.groundSpeed = 0.0;
    a.altitudeAGL=0;
    a.altitudeMSL=0;
    a.latitude=0;
    a.longitude=0;
    a.mode = 0;
    a.date = 0;
    a.time = 0;
	a.Az = 0;
	a.Ay = 0;
	a.Ax = 0;
	a.Bz = 0;
	a.By = 0;               
	a.Bx = 0;
	a.R = 0;
	a.Q = 0;
	a.P = 0;
	a.dRoll = 0;
	a.dPitch = 0;
	a.dYaw = 0;
    a.float_dYaw = 0.0;
    a.float_dPitch = 0.0;
    a.float_dRoll = 0.0;

    a = FlightDataUAV;
    //std::cout << FlightDataUAV.sensorData[0].hdop << "," << FlightDataUAV.sensorData[1].hdop << std::endl;
    return a;
}
