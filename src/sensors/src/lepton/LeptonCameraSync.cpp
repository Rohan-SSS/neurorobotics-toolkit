#include "sensors/lepton/LeptonCameraSync.hpp"
#include <thread>
#define LEPTON_Y16_SETTINGS
// #define LEPTON_GRAY8_SETTINGS

void CheckUniqueness(cv::Mat, cv::Mat, bool &);

#ifdef WRITE_MODULE_TS
std::ofstream LeptonCameraSync::lepton_timestamps("lepton_timestamps.txt", std::ios::out);
#endif

LeptonCameraSync::LeptonCameraSync(uvc_device_t* d)
{
    LeptonCameraSync::rawImage = cv::Mat();
    LeptonCameraSync::mLeptonFrameID = 0;
    LeptonCameraSync::timeStamp = 0;
	LeptonCameraSync::dev = d;
    keepReading = true;
    stopCapture = false;
    isFrameAvailable = false;
    IsInitial = true;
    PulsePin = new GPIO(5, 0);
    PulsePin->Initialize();
    InitDevice();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool LeptonCameraSync::IsDeviceFound()
{

    return (!stopCapture);
}

void LeptonCameraSync::LeptonCallback(uvc_frame_t *frame, void *ptr)
{

    LeptonCameraSync *_this = static_cast<LeptonCameraSync *>(ptr);

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
            mLeptonFrameID = frame->sequence; // Considering it is monotically increasing

#ifdef WRITE_MODULE_TS
            lepton_timestamps << timeStamp << " " << mLeptonFrameID << " "
                              << "0" << std::endl;
#endif
        }
    }
#endif
}

int LeptonCameraSync::InitDevice()
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
            uvc_perror(res, "unable to open device"); /* unable to open device */
        }
        else
        {
            /* Print out a message containing all the information that libuvc
             * knows about the device */
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
            LeptonCameraSync::m_portDesc.portID = 0;
            LeptonCameraSync::m_portDesc.portType = LEP_CCI_UVC;
            LeptonCameraSync::m_portDesc.userPtr = this;
            SetParams(devh, LeptonCameraSync::m_portDesc);

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
            /* Release our handle on the device */
        }
        /* Release the device descriptor */
    }
    return 0;
}

bool LeptonCameraSync::start(){
	if (ctrl_res == UVC_SUCCESS)
	{
		cout << "uvc_get_stream_ctrl_format_size" << endl;
	}

	/* Print out the result */
	if (ctrl_res < 0 && initialized)
	{
		stopCapture = true;
		uvc_perror(res, "device doesn't provide a matching stream"); /* device doesn't provide a matching stream */
	}
	else
	{
		InterruptThread = new std::thread(&LeptonCameraSync::ReadPulse, this);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		// Start the video stream. The library will call user function LeptonCallback:
		res = uvc_start_streaming(devh, &ctrl, LeptonCameraSync::LeptonCallback, this, 0);

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
	return 0;

}

void LeptonCameraSync::GetImage(cv::Mat &im, double &ts, int &seq_id)
{
    if (IsInitial)
    { // Checks for the special condition of initial frame capture treats everything as true
        std::unique_lock<std::mutex> locker(mFrameLock);
        if (!rawImage.empty())
        {
            rawImage.copyTo(im);
            std::cout << "Initial Frame Captured \t";
        }
        seq_id = mLeptonFrameID;
        PrevId = mLeptonFrameID;
        ts = timeStamp;
        std::cout << std::setprecision(15) << timeStamp << std::endl;
        std::cout << mLeptonFrameID << std::endl;
        IsInitial = false;
    }
    else
    {
        std::unique_lock<std::mutex> locker(mFrameLock);
        if ((mLeptonFrameID != PrevId) && (!rawImage.empty()) && (PrevtimeStamp != timeStamp))
        { // StateTransition checks for Pulse Previd checks for callback function and rawImage checks for data sanity
            rawImage.copyTo(im);
            ts = timeStamp;
            PrevtimeStamp = timeStamp;
            seq_id = mLeptonFrameID;
            PrevId = mLeptonFrameID;
            std::cout << "Unique Image Captured" << std::endl;
            std::cout << std::setprecision(15) << timeStamp << std::endl;
            std::cout << mLeptonFrameID << std::endl;
        }
        else
        {
            // Sends previous id until the a new frame has arrived
            seq_id = PrevId;
        }
    }
}

void LeptonCameraSync::CloseDevice()
{
    keepReading = false; // inverts the variable for pulse reading loop and handles its closure
    try
    {
        if (InterruptThread != NULL)
        {
            if (InterruptThread->joinable())
            {
                InterruptThread->join();
            }
            delete InterruptThread;
            InterruptThread = NULL;
        }
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
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

LeptonCameraSync::~LeptonCameraSync()
{
    keepReading = false;
    try
    {
        if (InterruptThread != NULL)
        { // handling of thread in event of abrupt closure
            if (InterruptThread->joinable())
            {
                InterruptThread->join();
            }
            delete InterruptThread;
        }
        if (mDeviceDescriptor != NULL)
        {
            uvc_free_device_descriptor(mDeviceDescriptor);
        }
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}

bool LeptonCameraSync::GetCurrentCameraStatusAndFrameIndex(int &index)
{
    std::lock_guard<std::mutex> lock(mFrameLock);
    index = mLeptonFrameID;
    return true;
}

void CheckUniqueness(cv::Mat img1, cv::Mat img2, bool &res)
{
    cv::Mat temp;
    cv::absdiff(img1, img2, temp);
    cv::Scalar sum = cv::sum(temp);
    if (sum[0] == 0)
    {
        res = true;
    }
    else
    {
        res = false;
    }
}

void LeptonCameraSync::ReadPulse()
{ // thread function which is to be called for reading
    try
    {
        while (keepReading)
        {
            if (IsInitial) // special condition of initial frame capture
            {
                std::unique_lock<std::mutex> lock(GPIOMutex);
                State = PulsePin->Read();
                PrevState = State;
                if (State == true)
                    StateTransition = true;
            }
            else
            {
                std::unique_lock<std::mutex> lock(GPIOMutex);
                State = PulsePin->Read();
                if (State == true && PrevState == false)
                { // updates transition only if positive edge is detected
                    StateTransition = true;
                }
                else
                {
                    StateTransition = false;
                }
                PrevState = State;
            }
            int val = UpdatetimeStamp();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
    }
    catch (std::exception &e)
    {
    std:;
        cout << e.what() << std::endl;
    }
}

int LeptonCameraSync::UpdatetimeStamp()
{ // updates timestamp according to the state transition
    if (StateTransition)
    {
        timeStamp = static_cast<double>(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count());
        return 1;
    }
    else
    {
        return 0;
    }
}
