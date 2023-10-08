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

#define VISUALIZE 0

// standard includes
#include <csignal>
#include <thread>
#include <chrono>
#include <mutex>

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <apriltag/apriltag.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/common/getopt.h>

/* Includes ------------------------------------------------------------------*/
//#include <liveview/test_liveview_entry.hpp>
#include <rpi_liveview/test_liveview.hpp>
//#include <custom_liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include <gimbal/test_gimbal_entry.cpp>
#include "application.hpp"
#include <fc_subscription/test_fc_subscription.h>
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
//
#include "widget/test_widget.h"
#include <dji_widget.h>
#include <dji_logger.h>
#include "utils/util_misc.h"
#include <dji_platform.h>
#include <stdio.h>
#include "dji_sdk_config.h"
#include "widget/file_binary_array_list_en.h"
//
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"

using namespace std;
using namespace cv;
using namespace chrono;

// global variables :(
bool running = true; // can only be changed to false only in one single place
pthread_t subscription_thread;
T_DjiReturnCode djiStat;
T_DjiFcSubscriptionGimbalAngles gimbal_angles;
T_DjiFcSubscriptionRC rc_status;
T_DjiDataTimestamp timestamp = {0};
mutex fcu_subscription_mutex;

pthread_t gimbal_control_thread;
E_DjiMountPosition gimbal_mount_position = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
E_DjiGimbalMode gimbal_mode = DJI_GIMBAL_MODE_FREE;
mutex apriltag_detection_mutex;
apriltag_detection_t apriltag_detection;
chrono::system_clock::time_point apriltag_detection_timestamp = chrono::system_clock::from_time_t(0);

// widget variables
//pthread_t widget_poll_thread;
mutex widget_mutex;
int aim_gimbal_index = 0;
bool aim_gimbal = false;
bool autonomous_control = true;
#define WIDGET_DIR_PATH_LEN_MAX         (256)
#define WIDGET_TASK_STACK_SIZE          (2048)
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData);
static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData);
static const T_DjiWidgetHandlerListItem s_widgetHandlerList[] = {
//    {0, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {1, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {0, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {3, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {4, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {5, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {6, DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {7, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {8, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
};
static const uint32_t s_widgetHandlerListCount = sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem);
static int32_t s_widgetValueList[sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem)] = {0};

#define MAIN_CAMERA 0
#if MAIN_CAMERA
const char* camera_name = "MAIN_CAM";
#else
const char* camera_name = "FPV_CAM";
#endif
LiveviewSample* liveviewSample;// = new LiveviewSample();
apriltag_family_t *tf = nullptr;
apriltag_detector_t *td = nullptr;
double global_image_half_width  = 0;
double global_image_half_height = 0;

// function declarations
void initialize();
void deinitialize();
void sigint_handler();
void* subscription_thread_function(void* args);
void* gimbal_control_function(void* args);
//void* widget_poll_function(void* args);
static void apriltag_image_callback(CameraRGBImage img, void *userData);
void initialize_widget();

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit();
static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState);

void sigint_handler(int signal)
{
	running = false;
	cout << "sending signal for graceful exit..." << endl;
}

void initialize()
{
//    DjiTest_WidgetStartService();
//    if( pthread_create( & widget_poll_thread, NULL, widget_poll_function, NULL ) )
//    {
//	cout << "problem" << endl;
//	return deinitialize();
//    }
    initialize_widget();

    if ( DjiTest_WidgetSpeakerStartService() )
    {
	USER_LOG_ERROR("widget speaker test init error");
    }

    // initialize fcu subscription thread
    if( pthread_create( & subscription_thread, NULL, subscription_thread_function, NULL ) )
    {
	cout << "problem" << endl;
	return deinitialize();
    }
    

    LiveviewSample* liveviewSample = new LiveviewSample();
#if MAIN_CAM
    liveviewSample->joshua_start_camera_stream_main(&apriltag_image_callback, &camera_name);

    // initialize gimbal control thread
    if( pthread_create( & gimbal_control_thread, NULL, gimbal_control_function, NULL ) )
    {
	cout << "problem" << endl;
	return deinitialize();
    }
#else
    liveviewSample->start_camera_stream(&apriltag_image_callback, &camera_name);
#endif
}

void deinitialize()
{
    cout << "deinitializing..." << endl;
//    cout << "joining widget poll thread" << endl;
//    pthread_join( widget_poll_thread, NULL );
    cout << "joining FCU data retrieval thread" << endl;
    pthread_join( subscription_thread, NULL );
#if MAIN_CAM
    cout << "joining gimbal control thread" << endl;
    pthread_join( gimbal_control_thread, NULL );
#else
    cout << "using FPV cam: no gimbal thread to join" << endl;
#endif
    cout << "all threads joined" << endl;

    // liveviewSample->stop_camera_stream
    cout << "deleting liveview object" << endl;
    delete liveviewSample;
    cout << "deleted liveview object" << endl;
}

// *******************************************************************************************************************************
static T_DjiTaskHandle s_widgetTestThread;
static bool s_isWidgetFileDirPathConfigured = false;
static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = {0};
//static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = "/home/joshua/git/Payload-SDK/samples/sample_c/module_sample/rpi_landing_widget/";


static const char *s_widgetTypeNameArray[] = {
    "Unknown",
    "Button",
    "Switch",
    "Scale",
    "List",
    "Int input box"
};
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData)
{
//    cout << "set widget value" << endl;
    
    widget_mutex.lock();
    USER_UTIL_UNUSED(userData);
    USER_LOG_INFO("Set widget value, widgetType = %s, widgetIndex = %d ,widgetValue = %d", s_widgetTypeNameArray[widgetType], index, value);
    s_widgetValueList[index] = value;
    widget_mutex.unlock();

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData)
{
//    cout << "get widget value" << endl;
    
    widget_mutex.lock();
    USER_UTIL_UNUSED(userData);
    USER_UTIL_UNUSED(widgetType);
    *value = s_widgetValueList[index];
    widget_mutex.unlock();

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

double cpu_temperature()
{
    std::string thermal_zone = "/sys/class/thermal/thermal_zone0/temp";
    std::ifstream temp_file(thermal_zone);
    if (!temp_file) {
        std::cerr << "Failed to open temperature file." << std::endl;
        return -1;
    }

    double temperature;
    temp_file >> temperature;
    temp_file.close();

    // The temperature is usually in millidegrees Celsius, so divide by 1000 to get degrees Celsius
    return temperature / 1000.0;
}

static void *DjiTest_WidgetTask(void *arg)
{
    char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
    uint32_t sysTimeMs = 0;
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    USER_UTIL_UNUSED(arg);

    osalHandler->TaskSleepMs(2000);

    while( running )
    {
        djiStat = osalHandler->GetTimeMs(&sysTimeMs);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
            USER_LOG_ERROR("Get system time ms error, stat = 0x%08llX", djiStat);
        }

#ifndef USER_FIRMWARE_MAJOR_VERSION
        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN,
			"System time (s) : %.1f\nCPU temp (C)  : %.1f \nApril Tag px: (%.1f, %.1f)\n", sysTimeMs/1000.0, cpu_temperature(), apriltag_detection.c[0], apriltag_detection.c[1]);
//        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN, "CPU temp C  : %f ms\n", cpu_temperature());
#else
        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN,
                 "System time : %u s\r\nVersion: v%02d.%02d.%02d.%02d\r\nBuild time: %s %s", sysTimeMs,
                 USER_FIRMWARE_MAJOR_VERSION, USER_FIRMWARE_MINOR_VERSION,
                 USER_FIRMWARE_MODIFY_VERSION, USER_FIRMWARE_DEBUG_VERSION,
                 __DATE__, __TIME__);
#endif


        djiStat = DjiWidgetFloatingWindow_ShowMessage(message);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
            USER_LOG_ERROR("Floating window show message error, stat = 0x%08llX", djiStat);
        }

        osalHandler->TaskSleepMs(50);
    }
}

void initialize_widget()
{
    printf("in the rpi pidget start function\n");

    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    cout << "starting step 1" << endl;
    //Step 1 : Init DJI Widget
    djiStat = DjiWidget_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji test widget init error, stat = 0x%08llX", djiStat);
    }

//    cout << "starting step 2" << endl;
//    //Step 2 : Set UI Config (Linux environment)
//    char curFileDirPath[WIDGET_DIR_PATH_LEN_MAX];
//    char tempPath[WIDGET_DIR_PATH_LEN_MAX];
//    djiStat = DjiUserUtil_GetCurrentFileDirPath(__FILE__, WIDGET_DIR_PATH_LEN_MAX, curFileDirPath);
//    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", djiStat);
//    }
//
//    cout << "starting step 3" << endl;
//    if (s_isWidgetFileDirPathConfigured == true) {
//        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", s_widgetFileDirPath);
//    } else {
//        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", curFileDirPath);
//    }
//
//    cout << "starting step 4" << endl;
//    //set default ui config path
//    djiStat = DjiWidget_RegDefaultUiConfigByDirPath(tempPath);
//    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
//    }
//
//    cout << "starting step 5" << endl;
//    //set ui config for English language
//    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
//                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
//                                             tempPath);
//    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
//    }

//    cout << "starting step 6" << endl;
//    //set ui config for Chinese language
//    if (s_isWidgetFileDirPathConfigured == true) {
//        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", s_widgetFileDirPath);
//    } else {
//        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", curFileDirPath);
//    }

//    cout << "starting step 7" << endl;
//    //Step 3 : Set widget handler list
//    djiStat = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
//    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Set widget handler list error, stat = 0x%08llX", djiStat);
//    }

//    cout << "starting step 6" << endl;
//    //Step 4 : Run widget api sample task
//    if (osalHandler->TaskCreate("user_widget_task", DjiTest_WidgetTask, WIDGET_TASK_STACK_SIZE, NULL,
//                                &s_widgetTestThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Dji widget test task create error.");
//    }
    
    cout << "end of widget creation" << endl;
}
// *******************************************************************************************************************************

//void* widget_poll_function(void* args)
//{
//	cout << "beginning widget poll thread" << endl;
//	while( running )
//	{
//		widget_mutex.lock();
////		DjiTestWidget_GetWidgetValue( DJI_WIDGET_TYPE_SWITCH, 0, & aim_gimbal, NULL );
//		widget_mutex.unlock();
//		
//		std::chrono::milliseconds sleep_duration(200);
//		std::this_thread::sleep_for(sleep_duration);
//	}
//	cout << "ending widget poll thread" << endl;
//}

void* subscription_thread_function(void* args)
{
	cout << "\tstarting data retrival thread" << endl;
	
	// initialize fcu subscription
	djiStat = DjiFcSubscription_Init();
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		USER_LOG_ERROR("init data subscription module error.");
	}

	// subscribe to gimbal angles topic
	djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		USER_LOG_ERROR("Error subscribing to the gimbal angles topic!");
	}
	
	// subscribe to gimbal angles topic
	djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		USER_LOG_ERROR("Error subscribing to the gimbal angles topic!");
	}

	while(running)
	{
		// ***********************************************************************************************
		fcu_subscription_mutex.lock();
		djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
								  (uint8_t *) &gimbal_angles,
								  sizeof(T_DjiFcSubscriptionGimbalAngles),
								  &timestamp);
		if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
		    USER_LOG_ERROR("Error getting the gimbal angles!");
		}

		djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
								  (uint8_t *) &rc_status,
								  sizeof(T_DjiFcSubscriptionRC),
								  &timestamp);
		if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
		    USER_LOG_ERROR("Error getting the rc topic!");
		}
		fcu_subscription_mutex.unlock();
		// ***********************************************************************************************

		if( autonomous_control && (abs(rc_status.pitch) > 1000 || abs(rc_status.roll) > 1000 || abs(rc_status.yaw) > 1000 || abs(rc_status.throttle) > 1000) )
		{
			autonomous_control = false;
			cout << "Disabling autonomous control because of manual stick input!" << endl;
		}

		std::chrono::milliseconds sleep_duration(20);
		std::this_thread::sleep_for(sleep_duration);
	}

	cout << "unsubscribing from fcu topics" << endl;
	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		USER_LOG_ERROR("Error unsubscribing from the gimbal angles topic!");
	}
	
	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		USER_LOG_ERROR("Error unsubscribing from the gimbal angles topic!");
	}

	cout << "\tending data retrival thread" << endl;
	return nullptr;
}

void* gimbal_control_function(void* args)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiAircraftInfoBaseInfo baseInfo;
    E_DjiAircraftSeries aircraftSeries;
    T_DjiGimbalManagerRotation rotation;

    uint8_t smooth_factor = 255;
    double speed_factor_yaw   = 2.75;
    double speed_factor_pitch = 2.60;

    returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Failed to get aircraft base info, return code 0x%08X", returnCode);
        return nullptr;
    }

    aircraftSeries = baseInfo.aircraftSeries;

    USER_LOG_INFO("Gimbal manager sample start");
    DjiTest_WidgetLogAppend("Gimbal manager sample start");

    // initialize
//    USER_LOG_INFO("--> Step 1: Init gimbal manager module");
//    DjiTest_WidgetLogAppend("--> Step 1: Init gimbal manager module");
    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Init gimbal manager failed, error code: 0x%08X", returnCode);
        return nullptr;
    }

    // set gimbal mode
    if (gimbal_mode == DJI_GIMBAL_MODE_FREE)
    {
        USER_LOG_INFO("--> Step 2: Set gimbal to free mode");
        DjiTest_WidgetLogAppend("--> Step 2: Set gimbal to free mode");
    }
    else if (gimbal_mode == DJI_GIMBAL_MODE_YAW_FOLLOW)
    {
        USER_LOG_INFO("--> Step 2: Set gimbal to yaw follow mode");
        DjiTest_WidgetLogAppend("--> Step 2: Set gimbal to yaw follow mode");
    }
    returnCode = DjiGimbalManager_SetMode(gimbal_mount_position, gimbal_mode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Set gimbal mode failed, error code: 0x%08X", returnCode);
        return nullptr;
    }

    // reset gimbal angles
//    USER_LOG_INFO("--> Step 3: Reset gimbal angles.\r\n");
    returnCode = DjiGimbalManager_Reset(gimbal_mount_position, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
    }
    
//    //set smooth factors
//    returnCode = DjiGimbalManager_SetControllerSmoothFactor(gimbal_mount_position, DJI_GIMBAL_AXIS_YAW, (uint8_t)smooth_factor);
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//    {
//        USER_LOG_ERROR("Failed!");
//    }
//    returnCode = DjiGimbalManager_SetControllerSmoothFactor(gimbal_mount_position, DJI_GIMBAL_AXIS_PITCH, (uint8_t)smooth_factor);
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//    {
//        USER_LOG_ERROR("Failed!");
//    }

// typedef struct {
//     E_DjiGimbalRotationMode rotationMode; /*!< Rotation gimbal mode. */
//     dji_f32_t pitch; /*!< Pitch angle in degree, unit: deg */
//     dji_f32_t roll; /*!< Roll angle in degree, unit: deg */
//     dji_f32_t yaw; /*!< Yaw angle in degree, unit: deg */
//     dji_f64_t time; /*!< Expect execution time of gimbal rotation, unit: second. */
// } T_DjiGimbalManagerRotation;

    aim_gimbal = s_widgetValueList[aim_gimbal_index];
    bool old_aim_gimbal = aim_gimbal;
    while( running )
    {
	widget_mutex.lock();
        aim_gimbal = s_widgetValueList[aim_gimbal_index];
	widget_mutex.unlock();
//	if( aim_gimbal != old_aim_gimbal )
//	{
//		cout << "aim gimbal edge" << endl;
//		// off edge -> reset the gimbal ?
//		if( ! aim_gimbal )
//		{
//			cout << "\n\n\nCENTERING THE GIMBAL!\n\n\n" << endl;
//			T_DjiGimbalManagerRotation rotation;
//			rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE; 
//			rotation.pitch =  0.0;
//			rotation.roll  =  0.0;
//			rotation.yaw   =  0.0;
//			rotation.time  =  2.0;
//			returnCode = DjiGimbalManager_Rotate(gimbal_mount_position, rotation);
//			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//			{
//				cerr << "error moving the gimbal!" << endl;
//			}
//		}
//		old_aim_gimbal = s_widgetValueList[0];
//	}

	auto now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - apriltag_detection_timestamp);
//	cout << "time since apriltag detection:" << duration.count() << endl;


	double u,v;
	if( aim_gimbal && duration.count() < 500 && global_image_half_width != 0 && global_image_half_height != 0 )
	{
		apriltag_detection_mutex.lock();
		u = apriltag_detection.c[0];
		v = apriltag_detection.c[1];
		apriltag_detection_mutex.unlock();
		double u_n = (u - global_image_half_width)  / global_image_half_width;
		double v_n = (v - global_image_half_height) / global_image_half_height;

		T_DjiGimbalManagerRotation rotation;
		rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE; 
		rotation.pitch =  - speed_factor_pitch * v_n;
		rotation.roll  =  0.0;
		rotation.yaw   =    speed_factor_yaw * u_n;
		rotation.time  = 2.0;
	      
//		cout << endl;
//		cout << "u: " << u << " , u_n: " << u_n << " , half_width:  " << global_image_half_width << endl;
//		cout << "v: " << v << " , v_n: " << v_n << " , half_height: " << global_image_half_height << endl;
//		cout << endl;

		returnCode = DjiGimbalManager_Rotate(gimbal_mount_position, rotation);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			cerr << "error moving the gimbal!" << endl;
		}
	}

	std::chrono::milliseconds sleep_duration(10);
	std::this_thread::sleep_for(sleep_duration);
    }

    cout << "reseting gimbal angles" << endl;
    USER_LOG_INFO("--> Step 3: Reset gimbal angles.\r\n");
    returnCode = DjiGimbalManager_Reset(gimbal_mount_position, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
    }
    cout << "exiting gimbal control thread" << endl;
}

/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{
    signal(SIGINT, sigint_handler);

    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

    initialize();

    while(running)
    {
//	    cout << "main loop" << endl;
//	    fcu_subscription_mutex.lock();
//	    cout << "gimbal PRY: "
//		 << gimbal_angles.x
//		 << " , "
//		 << gimbal_angles.y
//		 << " , "
//		 << gimbal_angles.z
//		 << endl;
//	    fcu_subscription_mutex.unlock();
//
//	    cout << "aim gimbal: " << s_widgetValueList[aim_gimbal_index] << endl;

	    std::chrono::milliseconds sleep_duration(500);
	    std::this_thread::sleep_for(sleep_duration);
    }

    deinitialize();

    exit(0);
}

static void apriltag_image_callback(CameraRGBImage img, void *userData)
{
//    auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
//    cout << "Joshua received image( " << millisec_since_epoch << " ): width=" << img.width << ", height=" << img.height << endl;

    string name = string(reinterpret_cast<char *>(userData));

    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);

        cvtColor(mat, mat, COLOR_RGB2BGR);

	// ***************************************************************************
	if( nullptr == tf )
	{
	    tf = tagCustom48h12_create();
	    td = apriltag_detector_create();
	    apriltag_detector_add_family(td, tf);

	    td->quad_decimate = 2;
	    td->quad_sigma    = 0;
	    td->nthreads      = 4;
	    td->debug         = false;
	    td->refine_edges  = false;
	}

	if( global_image_half_width == 0 || global_image_half_height == 0 )
	{
		global_image_half_width  = (double)img.width  / 2.0;
		global_image_half_height = (double)img.height / 2.0;
	}

	cv::Mat gray;
	cvtColor(mat, gray, COLOR_BGR2GRAY);
	image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
	zarray_t *detections = apriltag_detector_detect(td, &im);

//	cout << "tags detected: " << zarray_size(detections) << endl;

	// get the minimum ID in the detections:
	int maximum_id = -1;
        for (int i = 0; i < zarray_size(detections); i++)
	{
		apriltag_detection_t* det;
		zarray_get(detections, i, &det);
		if( maximum_id < det->id )
		{
			maximum_id = det->id;
			apriltag_detection_mutex.lock();
			apriltag_detection = *det;
			apriltag_detection_timestamp = std::chrono::system_clock::now();
			apriltag_detection_mutex.unlock();
		}
//		cout << "( " << det->c[0] << " , " << det->c[1] << " )" << endl;
	}

#if VISUALIZE
        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(mat, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(mat, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(mat, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(mat, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(mat, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

        }
#endif
        apriltag_detections_destroy(detections);	
	// ***************************************************************************

//    auto time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
//    cout << time << endl;

#if VISUALIZE
        imshow(name, mat);
	cv::waitKey(1);
#endif
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
