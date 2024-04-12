#ifndef _LEPTON_SYNC_
#define _LEPTON_SYNC_
// #include <iostream>
// #include <stdio.h>
// #include "opencv2/opencv.hpp"
// #include <mutex>
// #include <chrono>
// #include <thread>
// #include <atomic>
// #include <condition_variable>
#include "sensors/lepton/LeptonBase.h"
// #include "libuvc/libuvc.h"
#include "sensors/lepton/GPIO.h"
using namespace std;
using namespace cv;

class LeptonCameraSync : private LeptonBase
{
public:
      // uvc_device_handle_t *devh;

#ifdef WRITE_MODULE_TS
      static std::ofstream lepton_timestamps;
#endif

      LeptonCameraSync();
      LeptonCameraSync(uvc_device_t* d, std::function<void(Frame, sensor_id)> cb);
      ~LeptonCameraSync();
      int InitDevice();
      bool IsDeviceFound();
      void GetImage(cv::Mat &, double &cameraTimeStamp, int &seqID);
      void CloseDevice();
      int LeptonCommandIdToUnitId(LEP_COMMAND_ID commandID);
      static void LeptonCallback(uvc_frame_t *frame, void *ptr);
      bool GetCurrentCameraStatusAndFrameIndex(int &index);
      void ReadPulse();
      int UpdatetimeStamp();
	  bool publishFrame();
	  bool start();

private:
      std::atomic<bool> keepReading, IsInitial;
      std::mutex GPIOMutex;
      double PrevtimeStamp;
      std::atomic<bool> stopCapture;
      std::atomic<bool> isFrameAvailable;
      std::thread *InterruptThread;
      bool StateTransition;
      bool State, PrevState;
      int PrevId;
      GPIO *PulsePin;
};
#endif
