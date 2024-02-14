#pragma once


namespace TelemetryValidatorConstants
{
  // ALL GCS Transmitted Packets (refer to PacketHandlerTX.cs for any mods)

/*
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
*/

  const int UAV_GPS_1_Heading = 41;
  const int UAV_GPS_2_Heading = 64;

  const int UAV_GPS_1_AltitudeMSL = 49;
  const int UAV_GPS_2_AltitudeMSL = 72;

  const int UAV_GPS_1_HDOP = 38;
  const int UAV_GPS_2_HDOP = 61;

  const int UAV_GPS_1_GroundSpeed = 43;
  const int UAV_GPS_2_GroundSpeed = 66;

  const int UAV_GPS_1_Longitude = 53;
  const int UAV_GPS_2_Longitude = 76;

  const int UAV_GPS_1_Latitude = 57;
  const int UAV_GPS_2_Latitude = 80;

  const int UAV_GPS_1_AltitudeAGL = 45;
  const int UAV_GPS_2_AltitudeAGL = 68;

  const int UAV_GPS_1_NoOfSatellites = 40;
  const int UAV_GPS_2_NoOfSatellites = 63;


  const int GCS_SetWaypointPacket = 0x41;
  const int GCS_SetWaypointPacket_UavLat = 0;
  const int GCS_SetWaypointPacket_UavLon = 4;

  const int GCS_AltAirspeedPacket = 0x20;
  const int GCS_AltAirspeedPacket_Check = 0;
  const int GCS_AltAirspeedPacket_AGL = 1;

  const int GCS_TakeoffLandPacket = 0x22;
  const int GCS_TakeoffLandPacket_Type = 0;
  const int GCS_TakeoffLandPacket_AGL = 1;

  // ALL UAV Received Packets (refer to PacketHandlerRX.cs for any mods)
  const int UAV_GpsPacket = 0x81;
  const int UAV_GpsPacket_Mode = 1;
  const int UAV_GpsPacket_Lon = 18;
  const int UAV_GpsPacket_Lat = 22;

  const int UAV_AttitudePacket = 0x84;
  const int UAV_AttitudePacket_AGL = 11;
  const int UAV_AttitudePacket_r = 51;
  const int UAV_AttitudePacket_q = 53;
  const int UAV_AttitudePacket_p = 55;

  const int UAV_SensorPacket = 0x86;
  const int UAV_SensorPacket_MSL = 26;
  const int UAV_SensorPacket_Az = 138; // Accelerometer
  const int UAV_SensorPacket_Ay = 140; // Accelerometer 
  const int UAV_SensorPacket_Ax = 142; // Accelerometer
  const int UAV_SensorPacket_Bz = 144; // Magnetometer
  const int UAV_SensorPacket_By = 146; // Magnetometer
  const int UAV_SensorPacket_Bx = 148; // Magnetometer
  const int UAV_SensorPacket_R = 150; // Change in Yaw(Psi) (rad/sec)
  const int UAV_SensorPacket_Q = 152; // Change in Pitch(Theta) (rad/sec)
  const int UAV_SensorPacket_P = 154; // Change in Roll(Phi) (rad/sec)
  const int UAV_SensorPacket_Date = 158;
  const int UAV_SensorPacket_Time = 162;

  //const int UAV_SensorPacket_


  // ALL Datatype lengths  
  const int SIZE_BYTE = 1;
  const int SIZE_FLOAT = 4;
  const int SIZE_INT = 4;
  const int SIZE_SHORT = 2;

}
