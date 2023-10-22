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

#define MAIN_CAMERA 1
#define VISUALIZE 0
#define ENABLE_CAMERA_SWITCHING 1
#define ENABLE_PID_SWITCHING    1

#define FOCAL_LENGTH_WIDE     4.50
#define SENSOR_WIDTH_WIDE     6.29
#define SENSOR_HEIGHT_WIDE    4.71
#define FOCAL_LENGTH_ZOOM     6.83 // this is the base focal length -- can be up to 119.94
#define SENSOR_WIDTH_ZOOM     7.41
#define SENSOR_HEIGHT_ZOOM    5.56
#define FOCAL_LENGTH_THERMAL  13.5
#define SENSOR_WIDTH_THERMAL  7.68
#define SENSOR_HEIGHT_THERMAL 6.144

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
#include <utils/util_misc.h>
#include "camera_manager/test_camera_manager.h"
#include "dji_camera_manager.h"
#include "dji_platform.h"
#include "dji_logger.h"

#include "pid.h"

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
E_DjiGimbalMode gimbal_mode = DJI_GIMBAL_MODE_YAW_FOLLOW;
mutex apriltag_detection_mutex;
double u_n, v_n;
double theta_u, theta_v;
apriltag_detection_t apriltag_detection;
double apriltag_detection_area;
chrono::system_clock::time_point apriltag_detection_timestamp = chrono::system_clock::from_time_t(0);

pthread_t camera_management_thread;
typedef struct {
    E_DjiCameraType cameraType;
    char *cameraTypeStr;
} T_DjiTestCameraTypeStr;
static const T_DjiTestCameraTypeStr s_cameraTypeStrList[] = {
    {DJI_CAMERA_TYPE_UNKNOWN, "Unknown"},
    {DJI_CAMERA_TYPE_Z30,     "Zenmuse Z30"},
    {DJI_CAMERA_TYPE_XT2,     "Zenmuse XT2"},
    {DJI_CAMERA_TYPE_PSDK,    "Payload Camera"},
    {DJI_CAMERA_TYPE_XTS,     "Zenmuse XTS"},
    {DJI_CAMERA_TYPE_H20,     "Zenmuse H20"},
    {DJI_CAMERA_TYPE_H20T,    "Zenmuse H20T"},
    {DJI_CAMERA_TYPE_P1,      "Zenmuse P1"},
    {DJI_CAMERA_TYPE_L1,      "Zenmuse L1"},
    {DJI_CAMERA_TYPE_H20N,    "Zenmuse H20N"},
    {DJI_CAMERA_TYPE_M30,     "M30 Camera"},
    {DJI_CAMERA_TYPE_M30T,    "M30T Camera"},
    {DJI_CAMERA_TYPE_M3E,     "M3E Camera"},
    {DJI_CAMERA_TYPE_M3T,     "M3T Camera"},
};
E_DjiCameraManagerStreamSource current_stream_source = DJI_CAMERA_MANAGER_SOURCE_DEFAULT_CAM;
E_DjiCameraManagerStreamSource intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM;
mutex current_stream_source_mutex;
mutex intended_stream_source_mutex;
T_DjiCameraManagerOpticalZoomParam opticalZoomParam;

// widget variables
//pthread_t widget_poll_thread;
mutex widget_mutex;
int32_t aim_gimbal_index = 0;
bool aim_gimbal = false;
bool aim_gimbal_default = true;
int32_t autonomous_control_index = 1;
bool autonomous_control = false;
bool autonomous_control_default = false;
#define WIDGET_DIR_PATH_LEN_MAX         (256)
#define WIDGET_TASK_STACK_SIZE          (2048)
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData);
static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData);
static const T_DjiWidgetHandlerListItem s_widgetHandlerList[] = {
//    {0, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {1, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {0, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {1, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {2, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
    {3, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {3, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {4, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {5, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {6, DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {7, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {8, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
};
static const uint32_t s_widgetHandlerListCount = sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem);
static int32_t s_widgetValueList[sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem)] = {0};

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
void* camera_management_function(void* args);
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
#if MAIN_CAMERA
    liveviewSample->joshua_start_camera_stream_main(&apriltag_image_callback, &camera_name);

    // initialize gimbal control thread
    if( pthread_create( & gimbal_control_thread, NULL, gimbal_control_function, NULL ) )
    {
	cout << "problem" << endl;
	return deinitialize();
    }
    
    // initialize gimbal control thread
    if( pthread_create( & camera_management_thread, NULL, camera_management_function, NULL ) )
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
#if MAIN_CAMERA
    cout << "joining gimbal control thread" << endl;
    pthread_join( gimbal_control_thread, NULL );
    cout << "joining camera management thread" << endl;
    pthread_join( camera_management_thread, NULL );
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

    if( index == aim_gimbal_index )
    {
	    aim_gimbal = (bool) *value;
    }
    else if( index == autonomous_control_index )
    {
	    autonomous_control = (bool) *value;
    }

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

//    osalHandler->TaskSleepMs(3000);

    DjiTestWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, aim_gimbal_index, (int32_t) aim_gimbal_default, nullptr);
    DjiTestWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, autonomous_control_index, autonomous_control_default, nullptr);
    
    while( running )
    {
        djiStat = osalHandler->GetTimeMs(&sysTimeMs);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
            USER_LOG_ERROR("Get system time ms error, stat = 0x%08llX", djiStat);
        }

#ifndef USER_FIRMWARE_MAJOR_VERSION
        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN,
			"System time (s) : %.1f\nCPU temp (C)  : %.1f \nApril Tag: (%.5f, %.5f)\n", sysTimeMs/1000.0, cpu_temperature(), theta_u, theta_v);// u_n, v_n);
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
    
    std::chrono::milliseconds initial_sleep_duration(300);
    std::this_thread::sleep_for(initial_sleep_duration);

    //Step 1 : Init DJI Widget
    djiStat = DjiWidget_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji test widget init error, stat = 0x%08llX", djiStat);
    }
    
    std::this_thread::sleep_for(initial_sleep_duration);
    //Step 2 : Set UI Config (Linux environment)
    char curFileDirPath[WIDGET_DIR_PATH_LEN_MAX];
    char tempPath[WIDGET_DIR_PATH_LEN_MAX];
    djiStat = DjiUserUtil_GetCurrentFileDirPath(__FILE__, WIDGET_DIR_PATH_LEN_MAX, curFileDirPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", djiStat);
    }
    
    std::this_thread::sleep_for(initial_sleep_duration);

    if (s_isWidgetFileDirPathConfigured == true) {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", s_widgetFileDirPath);
    } else {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", curFileDirPath);
    }

    std::this_thread::sleep_for(initial_sleep_duration);
    
    //set default ui config path
    djiStat = DjiWidget_RegDefaultUiConfigByDirPath(tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
    }

    //set ui config for English language
    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                             tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
    }

    //set ui config for Chinese language
    if (s_isWidgetFileDirPathConfigured == true) {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", s_widgetFileDirPath);
    } else {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", curFileDirPath);
    }

    //Step 3 : Set widget handler list
    djiStat = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set widget handler list error, stat = 0x%08llX", djiStat);
    }

    //Step 4 : Run widget api sample task
    if (osalHandler->TaskCreate("user_widget_task", DjiTest_WidgetTask, WIDGET_TASK_STACK_SIZE, NULL,
                                &s_widgetTestThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji widget test task create error.");
    }
}

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
			DjiTestWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, autonomous_control_index, (int32_t)autonomous_control, nullptr);

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
//    double speed_factor_yaw   = 2.75;
//    double speed_factor_pitch = 2.60;
//    double speed_factor_yaw     = 3;
//    double speed_factor_pitch   = 2.75;
    double speed_factor_yaw     = 1;
    double speed_factor_pitch   = 1;
//    double speed_factor_yaw   = 120;
//    double speed_factor_pitch = speed_factor_yaw * 3 / 4; // 4:3 aspect ratio
    double zoom_factor_scalar = 0.1;

    returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Failed to get aircraft base info, return code 0x%08X", returnCode);
        return nullptr;
    }

    //PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
    // for angles
//    PID pid_u = PID(0.1, 10, -10, 2.5, 0.2, 0);
//    PID pid_v = PID(0.1, 10, -10, 1.5, 0.1, 0);
    PID pid_u_s = PID(0.1, 100, -100, 150, 30, 1.0);
    PID pid_v_s = PID(0.1, 100, -100, 150, 30, 1.0);
    
    PID pid_u_a = PID(0.1, 10, -10, 2.5, 0.2, 0);
    PID pid_v_a = PID(0.1, 10, -10, 2.5, 0.2, 0);

//    PID pid_u = PID(2.5, 0.00001, 0.0, 1);
//    pid_u.set_set_point(0);
//    pid_u.printParameters();
    
//    PID pid_v = PID(2.5, 0.75, 0.01, 10);
//    pid_v.set_set_point(0);
//    pid_v.printParameters();

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
    
// typedef struct {
//     E_DjiGimbalRotationMode rotationMode; /*!< Rotation gimbal mode. */
//     dji_f32_t pitch; /*!< Pitch angle in degree, unit: deg */
//     dji_f32_t roll; /*!< Roll angle in degree, unit: deg */
//     dji_f32_t yaw; /*!< Yaw angle in degree, unit: deg */
//     dji_f64_t time; /*!< Expect execution time of gimbal rotation, unit: second. */
// } T_DjiGimbalManagerRotation;

//    bool aim_gimbal = s_widgetValueList[aim_gimbal_index];
    bool old_aim_gimbal = aim_gimbal;
    while( running )
    {
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - apriltag_detection_timestamp);
//	cout << "time since apriltag detection:" << duration.count() << endl;

	double u,v;
	if( duration.count() < 500 && global_image_half_width != 0 && global_image_half_height != 0 )
	{
		apriltag_detection_mutex.lock();
		u = apriltag_detection.c[0];
		v = apriltag_detection.c[1];
		apriltag_detection_mutex.unlock();
		u_n = (u - global_image_half_width)  / global_image_half_width;
		v_n = (v - global_image_half_height) / global_image_half_height;

		if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM )
		{
			theta_u = u_n * atan( SENSOR_WIDTH_WIDE  / (2 * FOCAL_LENGTH_WIDE ) );
			theta_v = v_n * atan( SENSOR_HEIGHT_WIDE / (2 * FOCAL_LENGTH_WIDE ) );
		}
		else if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM )
		{
			theta_u = u_n * atan( SENSOR_WIDTH_WIDE   / ( opticalZoomParam.currentOpticalZoomFactor * 2 * FOCAL_LENGTH_WIDE ) );
			theta_v = v_n * atan( SENSOR_HEIGHT_WIDE  / ( opticalZoomParam.currentOpticalZoomFactor * 2 * FOCAL_LENGTH_WIDE ) );
		}
		else if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM )
		{
			theta_u = u_n * atan( SENSOR_WIDTH_ZOOM  / (2 * FOCAL_LENGTH_ZOOM ) );
			theta_v = v_n * atan( SENSOR_HEIGHT_ZOOM / (2 * FOCAL_LENGTH_ZOOM ) );
		}

//		cout << "( " << theta_u << " , " << theta_v << " )" << endl;
		
		if( aim_gimbal )
		{
			T_DjiGimbalManagerRotation rotation;
			
			double actuation_u_a = 0;
			double actuation_v_a = 0;
			double actuation_u_s = 0;
			double actuation_v_s = 0;
			
			actuation_u_a = pid_u_a.calculate(0, theta_u);
			actuation_v_a = pid_v_a.calculate(0, theta_v);
			actuation_u_s = pid_u_s.calculate(0, theta_u);
			actuation_v_s = pid_v_s.calculate(0, theta_v);


//			actuation_u = pid_u.output( theta_u );
//			actuation_v = pid_v.output( theta_v );

			cout << "actuation: " << actuation_u_a << " , " << actuation_v_a << endl;

#if ENABLE_PID_SWITCHING
			if( abs(actuation_u_a) > 0.11 || abs(actuation_v_a) > 0.11 )
			{
#endif
				rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE; 
				rotation.pitch =  1.0 * actuation_v_a;
				rotation.roll  =  0.0;
				rotation.yaw   =  -1.0 * actuation_u_a;
				rotation.time  = 2.0;

				cout << "angle actuation" << endl;
#if ENABLE_PID_SWITCHING				
			}
			else
			{
				rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_SPEED; 
				rotation.pitch =  1.0 * actuation_v_s;
				rotation.roll  =  0.0;
				rotation.yaw   =  -1.0 * actuation_u_s;
				rotation.time  = 0.1;

				cout << "speed actuation" << endl;
			}
#endif

//			T_DjiGimbalManagerRotation rotation;
////			rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE; 
//			rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_SPEED; 
//			rotation.pitch =  1.0 * actuation_v;
//			rotation.roll  =  0.0;
//			rotation.yaw   =  -1.0 * actuation_u;
//			rotation.time  = 0.1;

//			T_DjiGimbalManagerRotation rotation;
//			rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_SPEED; 
//			rotation.pitch =  actuation_v;
//			rotation.roll  =  0.0;
//			rotation.yaw   =  actuation_u;
//			rotation.time  = 0.1;
		      
			returnCode = DjiGimbalManager_Rotate(gimbal_mount_position, rotation);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
			{
				cerr << "error moving the gimbal!" << endl;
			}
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

static uint8_t DjiTest_CameraManagerGetCameraTypeIndex(E_DjiCameraType cameraType)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_cameraTypeStrList) / sizeof(s_cameraTypeStrList[0]); i++) {
        if (s_cameraTypeStrList[i].cameraType == cameraType) {
            return i;
        }
    }

    return 0;
}

void* camera_management_function(void* args)
{
	cout << "entering camera management thread" << endl;

    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    E_DjiCameraType cameraType;
    T_DjiCameraManagerFirmwareVersion firmwareVersion;
    T_DjiCameraManagerFocusPosData focusPosData;
    T_DjiCameraManagerTapZoomPosData tapZoomPosData;

    E_DjiMountPosition mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
		
    std::chrono::milliseconds initial_sleep_duration(500);
    std::this_thread::sleep_for(initial_sleep_duration);

    USER_LOG_INFO("--> Step 1: Init camera manager module");
    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X\r\n", returnCode);
	return nullptr;
    }

    std::this_thread::sleep_for(initial_sleep_duration);
    
    USER_LOG_INFO("--> Step 2: Get camera type and version");
    returnCode = DjiCameraManager_GetCameraType(mountPosition, &cameraType);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get mounted position %d camera's type failed, error code: 0x%08X\r\n",
                       mountPosition, returnCode);
	return nullptr;
    }
    USER_LOG_INFO("Mounted position %d camera's type is %s",
                  mountPosition,
                  s_cameraTypeStrList[DjiTest_CameraManagerGetCameraTypeIndex(cameraType)].cameraTypeStr);

    returnCode = DjiCameraManager_GetFirmwareVersion(mountPosition, &firmwareVersion);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get mounted position %d camera's firmware version failed, error code: 0x%08X\r\n",
                       mountPosition, returnCode);
	return nullptr;
    }
    USER_LOG_INFO("Mounted position %d camera's firmware is V%02d.%02d.%02d.%02d\r\n", mountPosition,
                  firmwareVersion.firmware_version[0], firmwareVersion.firmware_version[1],
                  firmwareVersion.firmware_version[2], firmwareVersion.firmware_version[3]);

    dji_f32_t default_zoom_factor = 2;
    USER_LOG_INFO("Set mounted position %d camera's zoom factor: %0.1f x.", mountPosition, default_zoom_factor);
    returnCode = DjiCameraManager_SetOpticalZoomParam(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_IN, default_zoom_factor);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_INFO("Set mounted position %d camera's zoom factor(%0.1f) failed, error code :0x%08X",
                      mountPosition, default_zoom_factor, returnCode);
    }

    returnCode = DjiCameraManager_SetFocusMode(mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
	returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
	USER_LOG_ERROR("Set mounted position %d camera's focus mode(%d) failed,"
		       " error code :0x%08X.", mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO,
		       returnCode);
    }

//		    T_DjiCameraManagerFocusPosData focus_point;
//		    focus_point.focusX = apriltag_detection.c[0];
//		    focus_point.focusY = apriltag_detection.c[1];
//		    USER_LOG_INFO("Set mounted position %d camera's focus point to (%0.1f, %0.1f).",
//				  mountPosition, apriltag_detection.c[0], apriltag_detection.c[1]);
//		    returnCode = DjiCameraManager_SetFocusTarget(mountPosition, focus_point);
//		    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
//			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
//			USER_LOG_ERROR("Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
//				       " error code :0x%08X.", mountPosition, focus_point.focusX, focus_point.focusY,
//				       returnCode);
//		    }

//typedef enum {
//    DJI_CAMERA_MANAGER_SOURCE_DEFAULT_CAM = 0x0,
//    DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM = 0x1,
//    DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM = 0x2,
//    DJI_CAMERA_MANAGER_SOURCE_IR_CAM = 0x3,
//    DJI_CAMERA_MANAGER_SOURCE_VISIBLE_CAM = 0x7,
//} E_DjiCameraManagerStreamSource;
	
	std::chrono::milliseconds sleep_duration(200);
	while( running )
	{
		intended_stream_source_mutex.lock();
		// cout << "Switching stream sources from " << current_stream_source << " to " << intended_stream_source << "..." << endl;
		returnCode = DjiCameraManager_SetStreamSource(mountPosition, intended_stream_source);
		// cout << "returnCode: " << returnCode << endl;
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			cout << "failed setting the stream source..." << endl;
		}
		else
		{
			current_stream_source_mutex.lock();
			current_stream_source = intended_stream_source;
			current_stream_source_mutex.unlock();
		}
		intended_stream_source_mutex.unlock();

		returnCode = DjiCameraManager_GetOpticalZoomParam(mountPosition, &opticalZoomParam);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
		{
			USER_LOG_ERROR("Get mounted position %d camera's zoom param failed, error code :0x%08X", mountPosition, returnCode);
		}

//typedef enum {
//    DJI_CAMERA_ZOOM_SPEED_SLOWEST = 72, /*!< Lens zooms in slowest speed. */
//    DJI_CAMERA_ZOOM_SPEED_SLOW = 73, /*!< Lens zooms in slow speed. */
//    DJI_CAMERA_ZOOM_SPEED_MODERATELY_SLOW = 74, /*!< Lens zooms in speed slightly slower than normal speed. */
//    DJI_CAMERA_ZOOM_SPEED_NORMAL = 75, /*!< Lens zooms in normal speed. */
//    DJI_CAMERA_ZOOM_SPEED_MODERATELY_FAST = 76, /*!< Lens zooms very in speed slightly faster than normal speed. */
//    DJI_CAMERA_ZOOM_SPEED_FAST = 77, /*!< Lens zooms very in fast speed. */
//    DJI_CAMERA_ZOOM_SPEED_FASTEST = 78, /*!< Lens zooms very in fastest speed. */
//} E_DjiCameraZoomSpeed;

//typedef enum {
//    DJI_CAMERA_ZOOM_DIRECTION_OUT = 0, /*!< The lens moves in the far direction, the zoom factor becomes smaller. */
//    DJI_CAMERA_ZOOM_DIRECTION_IN = 1, /*!< The lens moves in the near direction, the zoom factor becomes larger. */
//} E_DjiCameraZoomDirection;
		auto now = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - apriltag_detection_timestamp);
		// if the april tag detection is young enough to be considered a valid recognition of the landing pad
		if( aim_gimbal && duration.count() < 500 && global_image_half_width != 0 && global_image_half_height != 0 )
		{
			if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM )
			{
				// if the landing pad is too small
				if( apriltag_detection_area != 0 && apriltag_detection_area < 0.02 )
				{
					USER_LOG_INFO("landing pad too small");
					USER_LOG_INFO("zooming in...");
					// zoom in
					returnCode = DjiCameraManager_StartContinuousOpticalZoom(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_IN, DJI_CAMERA_ZOOM_SPEED_MODERATELY_SLOW);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
					{
						USER_LOG_ERROR("Mounted position %d camera start continuous zoom  failed,"
							       " error code :0x%08X.", mountPosition, returnCode);
					}
				}
				// else if the landing pad is too big
				else if( apriltag_detection_area != 0 && apriltag_detection_area > 0.0325 )
				{
					USER_LOG_INFO("landing pad too big");
					if( opticalZoomParam.currentOpticalZoomFactor > 2.0 )
					{
						USER_LOG_INFO("zooming out...");
						// zoom out
						returnCode = DjiCameraManager_StartContinuousOpticalZoom(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_OUT, DJI_CAMERA_ZOOM_SPEED_MODERATELY_SLOW);
						if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
						{
							USER_LOG_ERROR("Mounted position %d camera start continuous zoom  failed,"
								       " error code :0x%08X.", mountPosition, returnCode);
						}
					}
					else
					{
						USER_LOG_INFO("trying to switch to wide angle camera...");
						intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM;
					}
				}
				else if( apriltag_detection_area != 0 ) // if the landing pad is an ok size
				{
					USER_LOG_INFO("landing pad ok size");

					returnCode = DjiCameraManager_StopContinuousOpticalZoom(mountPosition);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
					    returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
					    USER_LOG_ERROR("Mounted position %d camera stop continuous zoom failed,"
						       " error code :0x%08X", mountPosition, returnCode);
					}
				}
			}
			else if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM )
			{
				// if the landing pad is too small
				if( apriltag_detection_area != 0 && apriltag_detection_area < 0.015 )
				{
					intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM;
				}
			}
		}
		else // if the landing pad detection has timed out
		{
			// stop any continuous zooming
			returnCode = DjiCameraManager_StopContinuousOpticalZoom(mountPosition);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			    returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			    USER_LOG_ERROR("Mounted position %d camera stop continuous zoom failed,"
			    	       " error code :0x%08X", mountPosition, returnCode);
			}
		}
//		USER_LOG_INFO("Current zoom factor(%0.1f)", opticalZoomParam.currentOpticalZoomFactor);
		std::this_thread::sleep_for(sleep_duration);
	}
    
    returnCode = DjiCameraManager_SetFocusMode(mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
	returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
	USER_LOG_ERROR("Set mounted position %d camera's focus mode(%d) failed,"
		       " error code :0x%08X.", mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO,
		       returnCode);
    }

    USER_LOG_INFO("Set mounted position %d camera's zoom factor: %0.1f x.", mountPosition, default_zoom_factor);
    returnCode = DjiCameraManager_SetOpticalZoomParam(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_IN, default_zoom_factor);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_INFO("Set mounted position %d camera's zoom factor(%0.1f) failed, error code :0x%08X",
                      mountPosition, default_zoom_factor, returnCode);
    }

	cout << "exiting camera management thread" << endl;
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
//	    cout << "aim gimbal: " << aim_gimbal << endl;
//	    cout << "autonomous control: " << autonomous_control << endl;

	    std::chrono::milliseconds sleep_duration(100);
	    std::this_thread::sleep_for(sleep_duration);
    }

    deinitialize();

    exit(0);
}

static void apriltag_image_callback(CameraRGBImage img, void *userData)
{
	apriltag_detection_area =0 ;
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
	
	// we have to handle IR images differently
	if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_IR_CAM )
	{
		cv::bitwise_not(gray, gray);
	}
	
	image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

	zarray_t *detections = apriltag_detector_detect(td, &im);

//	cout << "tags detected: " << zarray_size(detections) << endl;

//	cout << "tags: ( ";
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

			

//			// get area of april tag quadrilateral
//			for (i = 0; i < 4; i++)
//			{
//			    int j = (i + 1) % 4; // Calculate the next index (wraps around to 0 for the last point)
//			    apriltag_detection_area += det->p[i][0] * det->p[j][1] - det->p[j][0] * det->p[i][1];
//			}
//			apriltag_detection_area = 0.5 * fabs(apriltag_detection_area);
//			apriltag_detection_area /= (img.width * img.height);
		}
//		cout << "( " << det->c[0] << " , " << det->c[1] << " )" << endl;
//		cout << det->id << " ";
	}
	
	double x1 = apriltag_detection.p[0][0];
	double y1 = apriltag_detection.p[0][1];
	double x2 = apriltag_detection.p[1][0];
	double y2 = apriltag_detection.p[1][1];
	double x3 = apriltag_detection.p[2][0];
	double y3 = apriltag_detection.p[2][1];
	double x4 = apriltag_detection.p[3][0];
	double y4 = apriltag_detection.p[3][1];

	double area = 0.5 * abs( (x1*y2 + x2*y3 + x3*y4 + x4*y1) - (x2*y1 + x3*y2 + x4*y3 + x1*y4) ) / (img.width * img.height);

	if( apriltag_detection_area == 0 )
	{
		apriltag_detection_area = area;
	}
	else
	{
		apriltag_detection_area = 0.75*area + 0.25*apriltag_detection_area;
	}

//	cout << ") id: " << apriltag_detection.id << ", area: " << apriltag_detection_area << endl;

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
