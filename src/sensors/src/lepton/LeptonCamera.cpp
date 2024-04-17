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

LeptonCamera::LeptonCamera(uvc_device_t* d, std::function<void(Frame, sensor_id)> cb)
{
	LeptonCamera::initializeFormatsMaps();
    LeptonCamera::rawImage = cv::Mat();
    LeptonCamera::mLeptonFrameID = 0;
    LeptonCamera::timeStamp = 0;
	LeptonCamera::dev = d;
	LeptonCamera::originalCallback = cb;
    stopCapture = false;
    isFrameAvailable = false;
    /* Locates the first attached UVC device, stores in dev */
    // InitDevice();
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

    {
        std::unique_lock<std::mutex> lock(mFrameLock);
		Frame f;
		cv::Mat tempImage(frame->height, frame->width, _lepton_format_to_cv_format[prop->format], frame->data, 0);
		f.frame = tempImage.clone();
		if(prop->format == UVC_FRAME_FORMAT_Y16){
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
		f.timestamp = timeStamp;
        //std::cout<<"Device Serial Number created "<<mDeviceSerialNumber<<std::endl;
		originalCallback(f, mDeviceSerialNumber);
    }

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
            std::cout<<"dev_error is "<<dev_error<<std::endl; 
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
            std::cout<<"uvc frame format is "<<prop->format<<"       "<<UVC_FRAME_FORMAT_Y16<<"       "<<ctrl_res<<std::endl;     
			std::cout<<prop->frameWidth<<"   "<<prop->frameHeight<<"    "<<prop->frameRate<<"   "<<std::endl;
			if(prop->format == UVC_FRAME_FORMAT_GRAY8){
                std::cout<<"here, frame format gray8"<<std::endl;
				ctrl_res = uvc_get_stream_ctrl_format_size(
					devh, &ctrl,            /* result stored in ctrl */
					UVC_FRAME_FORMAT_GRAY8, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED, UVC_FRAME_FORMAT_UYVY*/
					prop->frameWidth, prop->frameHeight, prop->frameRate             /* width, height, fps */
				);
				initialized = true;
			}
			else if(prop->format == UVC_FRAME_FORMAT_Y16){
				std::cout<<"uvc frame format is Y16 "<<std::endl;
				//ctrl_res = uvc_get_stream_ctrl_format_size(
				//	devh, &ctrl,          /* result stored in ctrl */
				//	UVC_FRAME_FORMAT_Y16, /*UVC_FRAME_FORMAT_GRAY8, YUV 422, aka YUV 4:2:2. try _COMPRESSED, UVC_FRAME_FORMAT_UYVY*/
				//	prop->frameWidth, prop->frameHeight, prop->frameRate           /* width, height, fps */
				//);
				ctrl_res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_Y16, 160, 120, 9);
				initialized = true;
			}
			else{
				initialized = false;
				std::cout<<"Invalid Sensor Frame Type"<<std::endl;
			}
			std::cout<<"ctrl_res value is "<<ctrl_res<<std::endl;
        }
        /* Release the device descriptor */
    }
    return 0;
}

bool LeptonCamera::start(){
	std::cout<<"start function ctrl_res "<<ctrl_res<<std::endl;
	if (ctrl_res == UVC_SUCCESS)
	{
		cout << "uvc_get_stream_ctrl_format_size" << endl;
	}
	if (ctrl_res < 0 && initialized)
	{
		stopCapture = true;
		uvc_perror(res, "device doesn't provide a matching stream "); /* device doesn't provide a matching stream */
		return false;
	
	}
	else
	{

		// Start the video stream. The library will call user function LeptonCallback:
		std::cout<<"before streaming "<<std::endl;
		res = uvc_start_streaming(devh, &ctrl, LeptonCamera::LeptonCallback, this, 0);
		std::cout<<"after streaming "<<std::endl;
		if(prop->format == UVC_FRAME_FORMAT_GRAY8){
			SendGray8Settings(m_portDesc);
		}
		else if(prop->format == UVC_FRAME_FORMAT_Y16){
			SendY16Settings(m_portDesc);
		}
		std::cout<<" res "<<res<<std::endl;
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
