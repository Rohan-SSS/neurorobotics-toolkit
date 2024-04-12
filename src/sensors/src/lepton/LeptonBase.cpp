#include "sensors/lepton/LeptonBase.h"
#include <iostream>

void LeptonBase::SetParams(uvc_device_handle_t *obj, LEP_CAMERA_PORT_DESC_T port)
{
    devh = obj;
    portDesc = port;
    LEP_OpenPort(portDesc.portID, portDesc.portType, 0, &portDesc);
}

int LeptonBase::LeptonCommandIdToUnitId(LEP_COMMAND_ID commandID)
{
    int unit_id;

    switch (commandID & 0x3f00) // Ignore upper 2 bits including OEM bit
    {
    case LEP_CID_AGC_MODULE:
        unit_id = VC_CONTROL_XU_LEP_AGC_ID;
        break;

    case LEP_CID_OEM_MODULE:
        unit_id = VC_CONTROL_XU_LEP_OEM_ID;
        break;

    case LEP_CID_RAD_MODULE:
        unit_id = VC_CONTROL_XU_LEP_RAD_ID;
        break;

    case LEP_CID_SYS_MODULE:
        unit_id = VC_CONTROL_XU_LEP_SYS_ID;
        break;

    case LEP_CID_VID_MODULE:
        unit_id = VC_CONTROL_XU_LEP_VID_ID;
        break;

    default:
        return LEP_RANGE_ERROR;
    }

    return unit_id;
}

LEP_RESULT LeptonBase::UVC_GetAttribute(LEP_COMMAND_ID commandID, LEP_ATTRIBUTE_T_PTR attributePtr, LEP_UINT16 attributeWordLength)
{
    int unit_id;
    int control_id;
    int result;

    unit_id = LeptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
        return (LEP_RESULT)unit_id;

    control_id = ((commandID & 0x00ff) >> 2) + 1;

    // Size in 16-bit words needs to be in bytes
    attributeWordLength *= 2;

    // QMutexLocker lock(&m_mutex);
    // mFrameLock.lock();
    result = uvc_get_ctrl(devh, unit_id, control_id, attributePtr, attributeWordLength, UVC_GET_CUR);
    if (result != attributeWordLength)
    {
        printf("UVC_GetAttribute failed: %d", result);
        return LEP_COMM_ERROR_READING_COMM;
    }
    // mFrameLock.unlock();
    return LEP_OK;
}

LEP_RESULT LeptonBase::UVC_SetAttribute(LEP_COMMAND_ID commandID, LEP_ATTRIBUTE_T_PTR attributePtr, LEP_UINT16 attributeWordLength)
{
    int unit_id;
    int control_id;
    int result;

    unit_id = LeptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
        return (LEP_RESULT)unit_id;

    control_id = ((commandID & 0x00ff) >> 2) + 1;

    // Size in 16-bit words needs to be in bytes
    attributeWordLength *= 2;

    // QMutexLocker lock(&m_mutex);
    result = uvc_set_ctrl(devh, unit_id, control_id, attributePtr, attributeWordLength);
    if (result != attributeWordLength)
    {
        printf("UVC_SetAttribute failed: %d", result);
        return LEP_COMM_ERROR_READING_COMM;
    }

    return LEP_OK;
}

LEP_RESULT LeptonBase::UVC_RunCommand(LEP_COMMAND_ID commandID)
{
    int unit_id;
    int control_id;
    int result;

    unit_id = LeptonCommandIdToUnitId(commandID);
    if (unit_id < 0)
        return (LEP_RESULT)unit_id;

    control_id = ((commandID & 0x00ff) >> 2) + 1;

    // QMutexLocker lock(&m_mutex);
    result = uvc_set_ctrl(devh, unit_id, control_id, &control_id, 1);
    if (result != 1)
    {
        printf("UVC_RunCommand failed: %d", result);
        return LEP_COMM_ERROR_READING_COMM;
    }

    return LEP_OK;
}

int LeptonBase::SendY16Settings(LEP_CAMERA_PORT_DESC_T &port)
{
    LEP_RAD_ENABLE_E radEnableState;
    LEP_RAD_ENABLE_E radEnableStateSET = LEP_RAD_DISABLE;

    LEP_GetRadEnableState(&port, &radEnableState);

    LEP_SetRadEnableState(&port, radEnableStateSET);

    LEP_GetRadEnableState(&port, &radEnableState);

    LEP_SYS_FFC_SHUTTER_MODE_OBJ_T ffcShutterProp;
    LEP_GetSysFfcShutterModeObj(&port, &ffcShutterProp);

    ffcShutterProp.shutterMode = LEP_SYS_FFC_SHUTTER_MODE_MANUAL;
    LEP_SetSysFfcShutterModeObj(&port, ffcShutterProp);

    LEP_GetSysFfcShutterModeObj(&port, &ffcShutterProp);

    LEP_RunSysFFCNormalization(&port);
    return 0;
}

int LeptonBase::SendGray8Settings(LEP_CAMERA_PORT_DESC_T &port)
{
    LEP_RAD_ENABLE_E radEnableState;
    LEP_RAD_ENABLE_E radEnableStateSET = LEP_RAD_DISABLE;
    LEP_AGC_ENABLE_E agcEnableState;
    LEP_AGC_ENABLE_E agcEnableStateSET = LEP_AGC_ENABLE;
    LEP_AGC_POLICY_E agcPolicy;
    LEP_AGC_POLICY_E agcPolicySET = LEP_AGC_HEQ;
    LEP_SYS_GAIN_MODE_E gainState;
    LEP_SYS_GAIN_MODE_E gainStateSET = LEP_SYS_GAIN_MODE_HIGH;

    LEP_GetRadEnableState(&port, &radEnableState);
    // std::cout << "Radiometry Status "<< radEnableState << std::endl;

    LEP_SetRadEnableState(&port, radEnableStateSET);

    LEP_GetRadEnableState(&port, &radEnableState);
    // std::cout << "Radiometry Status "<< radEnableState << std::endl;

    LEP_GetAgcEnableState(&port, &agcEnableState);
    // std::cout << "AGC Status "<< agcEnableState << std::endl;

    LEP_SetAgcEnableState(&port, agcEnableStateSET);

    LEP_GetAgcEnableState(&port, &agcEnableState);
    // std::cout << "AGC Status "<< agcEnableState << std::endl;

    LEP_GetAgcPolicy(&port, &agcPolicy);
    // std::cout << "AGC Policy "<< agcPolicy << std::endl;

    LEP_SetAgcPolicy(&port, agcPolicySET);

    LEP_GetAgcPolicy(&port, &agcPolicy);
    // std::cout << "AGC Policy "<< agcPolicy << std::endl;

    LEP_GetSysGainMode(&port, &gainState);
    // std::cout << "Gain State "<< gainState << std::endl;

    LEP_SetSysGainMode(&port, gainStateSET);

    LEP_GetSysGainMode(&port, &gainState);
    std::cout << "Gain State " << gainState << std::endl;

    LEP_SYS_FFC_SHUTTER_MODE_OBJ_T ffcShutterProp;
    LEP_GetSysFfcShutterModeObj(&port, &ffcShutterProp);
    // std::cout << "Shutter Mode "<< ffcShutterProp.shutterMode << std::endl;

    ffcShutterProp.shutterMode = LEP_SYS_FFC_SHUTTER_MODE_MANUAL;
    LEP_SetSysFfcShutterModeObj(&port, ffcShutterProp);

    LEP_GetSysFfcShutterModeObj(&port, &ffcShutterProp);
    // std::cout << "Shutter Mode "<< ffcShutterProp.shutterMode << std::endl;
}

/* --------------------------------------------------------------------- */
/* -------- static wrapper functions for use by Lepton SDK only -------- */
/* --------------------------------------------------------------------- */

// These are the external functions which had to be defined for the library

LEP_RESULT UVC_GetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                            LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    return static_cast<LeptonBase *>(portDescPtr->userPtr)->UVC_GetAttribute(commandID, attributePtr, attributeWordLength);
}

LEP_RESULT UVC_SetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                            LEP_COMMAND_ID commandID,
                            LEP_ATTRIBUTE_T_PTR attributePtr,
                            LEP_UINT16 attributeWordLength)
{
    return static_cast<LeptonBase *>(portDescPtr->userPtr)->UVC_SetAttribute(commandID, attributePtr, attributeWordLength);
}

LEP_RESULT UVC_RunCommand(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                          LEP_COMMAND_ID commandID)
{
    return static_cast<LeptonBase *>(portDescPtr->userPtr)->UVC_RunCommand(commandID);
}

void LeptonBase::initializeFormatsMaps(){
	_lepton_format_to_cv_format[UVC_FRAME_FORMAT_GRAY8] = CV_8UC1;
	_lepton_format_to_cv_format[UVC_FRAME_FORMAT_Y16] = CV_16UC1;
	_lepton_format_to_cv_format[UVC_FRAME_FORMAT_UYVY] = CV_8UC2;
	
	_lepton_format_to_ros_format[UVC_FRAME_FORMAT_Y16] = sensor_msgs::image_encodings::MONO16;
	_lepton_format_to_ros_format[UVC_FRAME_FORMAT_GRAY8] = sensor_msgs::image_encodings::TYPE_8UC1;
	_lepton_format_to_ros_format[UVC_FRAME_FORMAT_UYVY] = sensor_msgs::image_encodings::YUV422;
}
