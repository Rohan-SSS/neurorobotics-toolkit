#ifndef _LEPTON_UTILS_
#define _LEPTON_UTILS_
#include <iostream>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include "uvc_sdk.h"
#include "LEPTON_RAD.h"
#include "LEPTON_ErrorCodes.h"
#include "LEPTON_Macros.h"
#include "LEPTON_OEM.h"
#include "LEPTON_VID.h"
#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_Types.h"
#include "LEPTON_SDKConfig.h"
#include "crc16.h"
#include "libuvc/libuvc.h"
#include <string>

#define LEP_CID_AGC_MODULE (0x0100)
#define LEP_CID_OEM_MODULE (0x0800)
#define LEP_CID_RAD_MODULE (0x0E00)
#define LEP_CID_SYS_MODULE (0x0200)
#define LEP_CID_VID_MODULE (0x0300)

typedef enum
{
  VC_CONTROL_XU_LEP_AGC_ID = 3,
  VC_CONTROL_XU_LEP_OEM_ID,
  VC_CONTROL_XU_LEP_RAD_ID,
  VC_CONTROL_XU_LEP_SYS_ID,
  VC_CONTROL_XU_LEP_VID_ID,
} VC_TERMINAL_ID;

class LeptonBase
{
public:

  void SetParams(uvc_device_handle_t *obj, LEP_CAMERA_PORT_DESC_T port);

  int LeptonCommandIdToUnitId(LEP_COMMAND_ID commandID);

  int SendY16Settings(LEP_CAMERA_PORT_DESC_T &port);

  int SendGray8Settings(LEP_CAMERA_PORT_DESC_T &port);

  LEP_RESULT UVC_GetAttribute(LEP_COMMAND_ID commandID,
                              LEP_ATTRIBUTE_T_PTR attributePtr,
                              LEP_UINT16 attributeWordLength);

  LEP_RESULT UVC_SetAttribute(LEP_COMMAND_ID commandID,
                              LEP_ATTRIBUTE_T_PTR attributePtr,
                              LEP_UINT16 attributeWordLength);

  LEP_RESULT UVC_RunCommand(LEP_COMMAND_ID commandID);
  virtual bool start() = 0;

protected:
  uvc_device_handle_t *devh;
  LEP_CAMERA_PORT_DESC_T portDesc;
  uvc_device_descriptor_t *mDeviceDescriptor = NULL;
  std::string mDeviceSerialNumber;
  uvc_device_t *dev;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  uvc_error_t ctrl_res;
  LEP_CAMERA_PORT_DESC_T m_portDesc;
  bool initialized = false;

  inline static cv::Mat rawImage;
  inline static std::atomic<int> mLeptonFrameID;
  inline static double timeStamp;
  inline static std::mutex mFrameLock;
};

#endif
