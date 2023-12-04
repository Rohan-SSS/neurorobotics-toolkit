#include "sensors/TelemetryValidator.h"
#include "sensors/UtilClasses.h"
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

class SimpleGpsPublisher: public rclcpp::Node{
	public:
		SimpleGpsPublisher(std::string nodeName);
		void setup();
		void gpsCallback(FlightLogData data);
	private:
		UDPCommunication* comm;
};
