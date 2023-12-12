#include "sensors/TelemetryValidator.h"
#ifndef UTIL_CLASSES
#define UTIL_CLASSES
#include "sensors/UtilClasses.h"
#endif
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>

#define TRANSMIT_PORT	65000
#define RECEIVE_PORT	7000

class UDPCommunication{
	public:
		UDPCommunication(rclcpp::Logger logger, std::function<void(FlightLogData)> cb);
		~UDPCommunication();
		void testReceiver(std::string gpsDataFilePath);
		void CloseCommunication();
		int start();
	private:
		void AcquireComData();
		void testThreadFunc(std::string filePath);
		std::thread* UDPCommunicationThread;
		
		int socketTransmit, socketReceive, addressLength, transmitAddressLength;
		struct sockaddr_in transmitAddress, receiveAddress;
		struct in_addr receiveFromIPAddress;
		
		FlightLogData currentUavData;
		TelemetryValidator telemetryValidator; 
		
		std::atomic<bool> stopAcquireFlag;
		bool flagExecuteRTH, flagExecuteALT, flagExecuteTIME;
		
		const int64_t communicationFailureTimeoutInSeconds = 20;
		
		std::mutex flightDataMutex;
		
		unsigned char RXPacket[1024];
		unsigned char TXPacket[1024];

		std::function<void(FlightLogData)> callback;
		std::function<void(FlightLogData)> originalCallback;
		
		rclcpp::Logger LOGGER;
};

class DayGpsFileReader{
	public:
		DayGpsFileReader(rclcpp::Logger logger, std::function<void(FlightLogData)> cb, std::string filePath);
		void AcquireFileData();
		void start();
	private:
		void readGPSFromText(std::string &line, FlightLogData &data);
		void Tokenize(std::string const &str, const char delim, std::vector<std::string> &out);
		int StringVecToSensorData(std::vector<std::string>& line, FlightLogData &mCurrentGPSPosition);
		rclcpp::Logger LOGGER;
		
		std::string gpsFilePath;
		
		std::function<void(FlightLogData)> callback;
		std::function<void(FlightLogData)> originalCallback;
		
		std::thread *th;
};

class SimpleGpsPublisher: public rclcpp::Node{
	public:
		SimpleGpsPublisher(std::string nodeName, std::string type);
		void setup(std::string type);
		void gpsCallback(FlightLogData data);
	private:
		UDPCommunication* comm;
		std::string commType;
		DayGpsFileReader* reader;
};
