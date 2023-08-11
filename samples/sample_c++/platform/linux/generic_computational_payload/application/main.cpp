/**
 ********************************************************************
 * @file    main.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dji_sdk_app_info.h"
#include "dji_sdk_config.h"
#include "../hal/hal_network.h"
#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include <hms/test_hms.h>
#include <waypoint_v2/test_waypoint_v2.h>
#include <waypoint_v3/test_waypoint_v3.h>
#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <camera_manager/test_camera_manager.h>
#include "camera_manager/test_camera_manager_entry.h"

// other stuff
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#define DATA_SEND_FROM_VIDEO_STREAM_MAX_LEN  60000
#define USER_UTIL_UNUSED(x)                                 ((x) = (x))
#define USER_UTIL_MIN(a, b)                                 (((a) < (b)) ? (a) : (b))
#define USER_UTIL_MAX(a, b)                                 (((a) > (b)) ? (a) : (b))

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit();
static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState);

/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{
    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

//    returnCode = DjiTest_CameraEmuBaseStartService();
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//	USER_LOG_ERROR("camera emu common init error");
//    }
//
//    if (DjiPlatform_GetSocketHandler() != nullptr) {
//	returnCode = DjiTest_CameraEmuMediaStartService();
//	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//	    USER_LOG_ERROR("camera emu media init error");
//	}
//    }

    cv::VideoCapture capture("/dev/video0");
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open video device." << std::endl;
        return 1;
    }

    // Create a named window to display the video feed
    cv::namedWindow("Video Feed", cv::WINDOW_NORMAL);

    while (true)
    {
        cv::Mat frame;
        cv::Mat frame_uint8;

        // Capture a frame from the video device
        capture >> frame;
	frame.convertTo(frame_uint8, CV_8U);

        if (frame.empty()) {
            std::cerr << "Error: No frame captured." << std::endl;
            break;
        }

        // Display the captured frame in the named window
        cv::imshow("Video Feed", frame_uint8);

        // Exit loop if 'q' is pressed
        if (cv::waitKey(1) == 'q')
	{
            break;
        }

//	size_t dataLength = frame_uint8.rows * frame_uint8.cols * frame_uint8.channels() * sizeof(uint8_t);
//	size_t lengthOfDataHaveBeenSent = 0;
//	while (dataLength - lengthOfDataHaveBeenSent)
//	{
//		size_t lengthOfDataToBeSent = USER_UTIL_MIN(DATA_SEND_FROM_VIDEO_STREAM_MAX_LEN, dataLength - lengthOfDataHaveBeenSent);
//		returnCode = DjiPayloadCamera_SendVideoStream((const uint8_t *) frame_uint8.data + lengthOfDataHaveBeenSent,
//		lengthOfDataToBeSent);
//		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//		{
//			USER_LOG_ERROR("send video stream error: 0x%08llX.", returnCode);
//		}
//		lengthOfDataHaveBeenSent += lengthOfDataToBeSent;
//	}
    }

    // Release the video capture object and destroy the window
    capture.release();
    cv::destroyAllWindows();

    while(true)
    {
	    std::cout << "Hello, World!" << std::endl;
	    osalHandler->TaskSleepMs(500);
    }

}

/* Private functions definition-----------------------------------------------*/
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit()
{
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState)
{
    //attention: please pull up the HWPR pin state by hardware.
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
