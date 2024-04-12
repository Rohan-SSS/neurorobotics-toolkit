#ifndef _LeptonCamera_
#define _LeptonCamera_
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

using namespace std;
using namespace cv;

class LeptonCamera : public LeptonBase
{
public:
      // uvc_device_handle_t *devh;
#ifdef WRITE_MODULE_TS
      static std::ofstream lepton_timestamps;
#endif
      std::atomic<bool> stopCapture;
      std::atomic<bool> isFrameAvailable;

      LeptonCamera(uvc_device_t *d, std::function<void(Frame, sensor_id)> cb);
	  LeptonCamera();
      ~LeptonCamera();
      int InitDevice();
      bool IsDeviceFound();
      void GetImage(cv::Mat &, double &cameraTimeStamp, int &seqID);
      void CloseDevice();
	  bool start();

      static void LeptonCallback(uvc_frame_t *frame, void *ptr);
      bool GetCurrentCameraStatusAndFrameIndex(int &index);
};

#endif
