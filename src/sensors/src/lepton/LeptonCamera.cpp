#include "sensors/lepton/LeptonCamera.hpp"
#include <thread>

#define LEPTON_Y16_SETTINGS
// #define LEPTON_GRAY8_SETTINGS

#ifdef WRITE_MODULE_TS
std::ofstream LeptonCamera::lepton_timestamps("lepton_timestamps.txt", std::ios::out);
#endif

LeptonCamera::LeptonCamera()
{
	std::cout<<"Creating LeptonCamera"<<std::endl;
}
LeptonCamera::LeptonCamera(uvc_device_t* d)
{
    LeptonCamera::rawImage = cv::Mat();
    LeptonCamera::mLeptonFrameID = 0;
    LeptonCamera::timeStamp = 0;
	LeptonCamera::dev = d;
    stopCapture = false;
    isFrameAvailable = false;
    /* Locates the first attached UVC device, stores in dev */
    InitDevice();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool LeptonCamera::IsDeviceFound()
{
    return (!stopCapture);
}

void LeptonCamera::LeptonCallback(uvc_frame_t *frame, void *ptr)
{
    double cbEnterTime, cbExitTime;
    struct timeval timeVal; // timeval type varirable from UVC library
    cbEnterTime = cv::getTickCount();
    LeptonCamera *_this = static_cast<LeptonCamera *>(ptr);
    uvc_error_t ret;

#ifdef LEPTON_GRAY8_SETTINGS
    {
        std::unique_lock<std::mutex> lock(mFrameLock);
        rawImage = cv::Mat(frame->height, frame->width, CV_8UC1, frame->data, 0);
    }
#endif

#ifdef LEPTON_Y16_SETTINGS
    {
        std::unique_lock<std::mutex> lock(mFrameLock);
        cv::Mat tempImage(frame->height, frame->width, CV_16UC1, frame->data, 0);
        rawImage = tempImage.clone();
        if (frame->sequence != mLeptonFrameID)
        {
            timeStamp = static_cast<double>(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count());
            mLeptonFrameID = frame->sequence; // Considering it is monotically increasing
#ifdef WRITE_MODULE_TS
            lepton_timestamps << timeStamp << " " << mLeptonFrameID << " "
                              << "0" << std::endl;
#endif
        }
    }
#endif
    cbExitTime = cv::getTickCount();
}

int LeptonCamera::InitDevice()
{
    if (stopCapture)
    {
        return -1;
    }
    else
    {
        /* Try to open the device: requires exclusive access */
        res = uvc_open(dev, &devh);
        if (res < 0)
        {
            stopCapture = true;
            uvc_perror(res, "Unable to open Device"); /* unable to open device */
        }
        else
        {
            uvc_error_t dev_error;
            mDeviceDescriptor = NULL;
            dev_error = uvc_get_device_descriptor(dev, &mDeviceDescriptor);

            if (dev_error == UVC_SUCCESS)
            {
                mDeviceSerialNumber = std::string(mDeviceDescriptor->serialNumber);
            }
            else
            {
                uvc_free_device_descriptor(mDeviceDescriptor);
                mDeviceDescriptor = NULL;
            }
            // Nikhil 01/02/22
            LeptonCamera::m_portDesc.portID = 0;
            LeptonCamera::m_portDesc.portType = LEP_CCI_UVC;
            LeptonCamera::m_portDesc.userPtr = this;
            SetParams(devh, LeptonCamera::m_portDesc);

#ifdef LEPTON_GRAY8_SETTINGS
            ctrl_res = uvc_get_stream_ctrl_format_size(
                devh, &ctrl,            /* result stored in ctrl */
                UVC_FRAME_FORMAT_GRAY8, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED, UVC_FRAME_FORMAT_UYVY*/
                160, 120, 9             /* width, height, fps */
            );
#endif

#ifdef LEPTON_Y16_SETTINGS
            ctrl_res = uvc_get_stream_ctrl_format_size(
                devh, &ctrl,          /* result stored in ctrl */
                UVC_FRAME_FORMAT_Y16, /*UVC_FRAME_FORMAT_GRAY8, YUV 422, aka YUV 4:2:2. try _COMPRESSED, UVC_FRAME_FORMAT_UYVY*/
                160, 120, 9           /* width, height, fps */
            );
#endif
			initialized = true;
        }
        /* Release the device descriptor */
    }
    return 0;
}

bool LeptonCamera::start(){
	if (ctrl_res == UVC_SUCCESS)
	{
		cout << "uvc_get_stream_ctrl_format_size" << endl;
	}
	if (ctrl_res < 0 && initialized)
	{
		stopCapture = true;
		uvc_perror(res, "device doesn't provide a matching stream "); /* device doesn't provide a matching stream */
	}
	else
	{
		// Start the video stream. The library will call user function LeptonCallback:
		res = uvc_start_streaming(devh, &ctrl, LeptonCamera::LeptonCallback, this, 0);
#ifdef LEPTON_GRAY8_SETTINGS
		SendGray8Settings(m_portDesc);
#endif

#ifdef LEPTON_Y16_SETTINGS
		SendY16Settings(m_portDesc);
#endif
		if (res < 0)
		{
			stopCapture = true;
			uvc_perror(res, "unable to start stream"); /* unable to start stream */
		}
		else
		{
			uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */
		}
	}
	/* Release our handle on the device */

	return true;
}

void LeptonCamera::GetImage(cv::Mat &im, double &ts, int &seq_id)
{
    std::unique_lock<std::mutex> locker(mFrameLock);
    if (!rawImage.empty())
    {
        rawImage.copyTo(im);
    }
    ts = timeStamp;
    seq_id = mLeptonFrameID;
}

void LeptonCamera::CloseDevice()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#ifdef WRITE_MODULE_TS
    lepton_timestamps.close();
#endif

    uvc_stop_streaming(devh);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uvc_close(devh);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uvc_unref_device(dev);
    // uvc_exit(ctx);

    stopCapture = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

LeptonCamera::~LeptonCamera()
{
    if (mDeviceDescriptor != NULL)
    {
        uvc_free_device_descriptor(mDeviceDescriptor);
    }
}

bool LeptonCamera::GetCurrentCameraStatusAndFrameIndex(int &index)
{
    std::lock_guard<std::mutex> lock(mFrameLock);
    index = mLeptonFrameID;
    return true;
}
