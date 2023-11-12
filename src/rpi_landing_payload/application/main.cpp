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

#define GIMBAL_AIM_MODE_ANGLE 0
#define ZOOM_SCALAR 1
#define MAX_SPEED 30
#define SPEED_FACTOR 5
#define RAD_TO_DEG 57.2957795
#define DEG_TO_RAD  0.0174533

#define MAIN_CAMERA 1
#define VISUALIZE 0
#define ENABLE_CAMERA_SWITCHING 1
#define ENABLE_PID_SWITCHING    0

#define FOCAL_LENGTH_WIDE     4.50
#define SENSOR_WIDTH_WIDE     6.29
#define SENSOR_HEIGHT_WIDE    4.71
#define FOCAL_LENGTH_ZOOM     6.83 // this is the base focal length -- can be up to 119.94
#define SENSOR_WIDTH_ZOOM     7.41
#define SENSOR_HEIGHT_ZOOM    5.56
#define FOCAL_LENGTH_IR  13.5
#define SENSOR_WIDTH_IR  7.68
#define SENSOR_HEIGHT_IR 6.144

// standard includes
#include <csignal>
#include <thread>
#include <chrono>
#include <mutex>
#include <exception>

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

#include <dji_flight_controller.h>

#include "pid.h"

using namespace std;
using namespace cv;
using namespace chrono;

T_DjiAircraftInfoBaseInfo baseInfo;

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
double aircraft_yaw = 0;

pthread_t camera_management_thread;
typedef struct
{
	E_DjiCameraType cameraType;
	char *cameraTypeStr;
} T_DjiTestCameraTypeStr;
static const T_DjiTestCameraTypeStr s_cameraTypeStrList[] =
{
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

bool intended_stream_rgb = true;
E_DjiCameraManagerStreamSource current_stream_source = DJI_CAMERA_MANAGER_SOURCE_DEFAULT_CAM;
E_DjiCameraManagerStreamSource intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM;
mutex current_stream_source_mutex;
mutex intended_stream_source_mutex;
T_DjiCameraManagerOpticalZoomParam opticalZoomParam;

// widget variables
//pthread_t widget_poll_thread;
int32_t stop_switch_index = 5;
mutex widget_mutex;
int32_t aim_gimbal_index = 0;
bool aim_gimbal = false;
bool aim_gimbal_default = true;
int32_t intended_stream_rgb_index = 1;
int32_t autonomous_control_index = 2;
int32_t automatic_landing_index = 3;
bool automatic_landing = false;
int32_t automatic_takeoff_index = 4;
bool automatic_takeoff = false;
bool autonomous_control = false;
bool autonomous_control_default = false;
#define WIDGET_DIR_PATH_LEN_MAX         (256)
#define WIDGET_TASK_STACK_SIZE          (2048)
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData);
static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData);
static const T_DjiWidgetHandlerListItem s_widgetHandlerList[] =
{
//    {0, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {1, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
	{0, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
	{1, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
	{2, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
	{3, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
	{4, DJI_WIDGET_TYPE_SWITCH,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
	{5, DJI_WIDGET_TYPE_SWITCH,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {4, DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {5, DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {6, DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {7, DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
//    {8, DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue, DjiTestWidget_GetWidgetValue, NULL},
};
static const uint32_t s_widgetHandlerListCount = sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem);
static int32_t s_widgetValueList[sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem)] = {0};

pthread_t flight_control_thread;

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
void graceful_exit();
void* subscription_thread_function(void* args);
void* gimbal_control_function(void* args);
void* flight_control_function(void* args);
void* camera_management_function(void* args);
//void* widget_poll_function(void* args);
static void apriltag_image_callback(CameraRGBImage img, void *userData);
void initialize_widget();

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit();
static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState);

void graceful_exit(int signal)
{
	running = false;
	cout << "received signal " << signal << " for graceful exit..." << endl;
}

void initialize()
{

	T_DjiReturnCode returnCode;
	returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Failed to get aircraft base info, return code 0x%08X", returnCode);
		return deinitialize();
	}

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

	if( pthread_create( & flight_control_thread, NULL, flight_control_function, NULL ) )
	{
		cout << "problem" << endl;
		return deinitialize();
	}
}

void deinitialize()
{
	USER_LOG_INFO("deinitializing...");
	USER_LOG_INFO("joining FCU data retrieval thread");
	pthread_join(subscription_thread, NULL);

#if MAIN_CAMERA
	USER_LOG_INFO("joining gimbal control thread");
	pthread_join(gimbal_control_thread, NULL);
	USER_LOG_INFO("joining camera management thread");
	pthread_join(camera_management_thread, NULL);
#else
	USER_LOG_INFO("using FPV cam: no gimbal thread to join");
#endif

	USER_LOG_INFO("joining flight control thread");
	pthread_join(flight_control_thread, NULL);
	USER_LOG_INFO("all threads joined");

	USER_LOG_INFO("deleting liveview object");
	delete liveviewSample;
	USER_LOG_INFO("deleted liveview object");
}

// *******************************************************************************************************************************
static T_DjiTaskHandle s_widgetTestThread;
static bool s_isWidgetFileDirPathConfigured = false;
static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = {0};
//static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = "/home/joshua/git/Payload-SDK/samples/sample_c/module_sample/rpi_landing_widget/";


static const char *s_widgetTypeNameArray[] =
{
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
	else if( index == intended_stream_rgb_index )
	{
		if( (bool)* value )
		{
			intended_stream_rgb = false;
		}
		else
		{
			intended_stream_rgb = true;
		}
	}
	else if( index == autonomous_control_index )
	{
		autonomous_control = (bool) *value;
	}
	else if( index == automatic_landing_index )
	{
		automatic_landing = (bool) *value;
	}
	else if( index == automatic_takeoff_index )
	{
		automatic_takeoff = (bool) *value;
	}
	else if( index == stop_switch_index )
	{
		if( *value )
		{
			running = false;
		}
	}

	widget_mutex.unlock();

	return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

double cpu_temperature()
{
	std::string thermal_zone = "/sys/class/thermal/thermal_zone0/temp";
	std::ifstream temp_file(thermal_zone);
	if (!temp_file)
	{
		std::cerr << "Failed to open temperature file." << std::endl;
		return -1;
	}

	double temperature;
	temp_file >> temperature;
	temp_file.close();

	// The temperature is usually in millidegrees Celsius, so divide by 1000 to get degrees Celsius
	return temperature / 1000.0;
}

double get_landing_pad_relative_pitch()
{
	return gimbal_angles.x - theta_v * 180 / M_PI;
}

double get_landing_pad_relative_yaw()
{
	return theta_u + gimbal_angles.z - aircraft_yaw;
}

static void *DjiTest_WidgetTask(void *arg)
{
	cout << "lalalalala" << endl;
	char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
	uint32_t sysTimeMs = 0;
	T_DjiReturnCode djiStat;
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

	USER_UTIL_UNUSED(arg);

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
		         "System time (s) : %.1f\nCPU temp (C)  : %.1f \nApril Tag: (%.5f, %.5f)\n", sysTimeMs/1000.0, cpu_temperature(), get_landing_pad_relative_pitch(), get_landing_pad_relative_yaw());//, theta_u, theta_v);// u_n, v_n);
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
	USER_LOG_INFO("Starting the widget.");

	T_DjiReturnCode djiStat;
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

//	std::chrono::milliseconds initial_sleep_duration(300);
//	std::this_thread::sleep_for(initial_sleep_duration);


	cout << "before" << endl;
	// initialize DJI Widget
	djiStat = DjiWidget_Init();
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Dji test widget init error, stat = 0x%08llX", djiStat);
	}
	cout << "after" << endl;

//	std::this_thread::sleep_for(initial_sleep_duration);

	cout << "before2" << endl;
	// set UI Config (Linux environment)
	char curFileDirPath[WIDGET_DIR_PATH_LEN_MAX];
	char tempPath[WIDGET_DIR_PATH_LEN_MAX];
	djiStat = DjiUserUtil_GetCurrentFileDirPath(__FILE__, WIDGET_DIR_PATH_LEN_MAX, curFileDirPath);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", djiStat);
	}
	cout << curFileDirPath << endl;
	cout << tempPath << endl;
	cout << "after2" << endl;

//	std::this_thread::sleep_for(initial_sleep_duration);

	if (s_isWidgetFileDirPathConfigured == true)
	{
		snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", s_widgetFileDirPath);
	}
	else
	{
		snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", curFileDirPath);
	}

//	std::this_thread::sleep_for(initial_sleep_duration);

	// set default ui config path
	djiStat = DjiWidget_RegDefaultUiConfigByDirPath(tempPath);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
	}

	// set ui config for English language
	djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
	          DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
	          tempPath);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
	}

	// set ui config for Chinese language
	if (s_isWidgetFileDirPathConfigured == true)
	{
		snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", s_widgetFileDirPath);
	}
	else
	{
		snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", curFileDirPath);
	}

	// set widget handler list
	djiStat = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Set widget handler list error, stat = 0x%08llX", djiStat);
	}

	// run widget api sample task
	if (osalHandler->TaskCreate("user_widget_task", DjiTest_WidgetTask, WIDGET_TASK_STACK_SIZE, NULL,
	                            &s_widgetTestThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Dji widget test task create error.");
	}
}

void* subscription_thread_function(void* args)
{
	T_DjiFcSubscriptionQuaternion quaternion = {0};

	USER_LOG_INFO("Starting data retrival thread.");

	// initialize fcu subscription
	djiStat = DjiFcSubscription_Init();
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error initializing data subscription module error.");
	}

	// subscribe to the gimbal angles topic
	djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error subscribing to the gimbal angles topic!");
	}

	// subscribe to the RC topic
	djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error subscribing to the RC topic!");
	}

	// subscribe to the attitude quaternion
	djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error subscribing to the quaternion topic!");
	}

//	_CONTROL_DEVICE
	djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error subscribing to the control device topic!");
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

		djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
		          (uint8_t *) &quaternion,
		          sizeof(T_DjiFcSubscriptionQuaternion),
		          &timestamp);
		if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Error getting the quaternion topic!");
		}

		T_DjiFcSubscriptionControlDevice controlDevice;

		djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
		          (uint8_t *) &controlDevice,
		          sizeof(T_DjiFcSubscriptionControlDevice),
		          &timestamp);

		if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
		}
		else
		{
			USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
			               timestamp.microsecond);
		}
//		cout << "control mode:  " << controlDevice.controlMode  << endl;
//		cout << "device status: " << controlDevice.deviceStatus << endl;
//		cout << "flight status: " << controlDevice.flightStatus << endl;
//		cout << "vrc status:    " << controlDevice.vrcStatus    << endl;
//		cout << "reserved:      " << controlDevice.reserved     << endl;

//		T_DjiFcSubscriptionControlDevice control_info;
//		djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
//		          (uint8_t *) &control_info,
//		          sizeof(T_DjiFcSubscriptionControlDevice),
//		          &timestamp);
//		if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//		{
//			cout << "error getting the control device topic" << endl;
//			USER_LOG_ERROR("Error getting the control device topic!");
//		}
//		else
//		{
//			cout << "no error getting the control device topic" << endl;
//		}

		fcu_subscription_mutex.unlock();

//		cout << "control mode:  " << control_info.controlMode  << endl;
//		cout << "device status: " << control_info.deviceStatus << endl;
//		cout << "flight status: " << control_info.flightStatus << endl;
//		cout << "vrc status:    " << control_info.vrcStatus    << endl;
//		cout << "reserved:      " << control_info.reserved     << endl;

		// ***********************************************************************************************

		double aircraftYawInRad = atan2(2 * ((double) quaternion.q0 * quaternion.q3 + (double) quaternion.q1 * quaternion.q2),
		                                (double) 1 -
		                                2 * ((double) quaternion.q2 * quaternion.q2 + (double) quaternion.q3 * quaternion.q3));
		aircraft_yaw = aircraftYawInRad * 180 / DJI_PI;

//		if( autonomous_control && (abs(rc_status.pitch) > 1000 || abs(rc_status.roll) > 1000 || abs(rc_status.yaw) > 1000 || abs(rc_status.throttle) > 1000) )
//		{
//			autonomous_control = false;
//			DjiTestWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, autonomous_control_index, (int32_t)autonomous_control, nullptr);
//
//			USER_LOG_INFO("Disabling autonomous control because of manual stick input!");
//		}

		std::chrono::milliseconds sleep_duration(20);
		std::this_thread::sleep_for(sleep_duration);
	}

	USER_LOG_INFO("Unsubscribing from FCU data topics.");
	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error unsubscribing from the gimbal angles topic!");
	}

	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error unsubscribing from the RC topic!");
	}

	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error unsubscribing from the quaternion topic!");
	}

	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE);
	if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Error unsubscribing from the control device topic!");
	}

	USER_LOG_INFO("Ending data retrieval thread.");
	return nullptr;
}

static T_DjiReturnCode
DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
	switch (eventData.joystickCtrlAuthoritySwitchEvent)
	{
	case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT:
	{
		if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK)
		{
			USER_LOG_INFO("[Event] Msdk request to obtain joystick ctrl authority\r\n");
		}
		else
		{
			USER_LOG_INFO("[Event] Msdk request to release joystick ctrl authority\r\n");
		}
		break;
	}
	case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT:
	{
		if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL)
		{
			USER_LOG_INFO("[Event] Internal request to obtain joystick ctrl authority\r\n");
		}
		else
		{
			USER_LOG_INFO("[Event] Internal request to release joystick ctrl authority\r\n");
		}
		break;
	}
	case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT:
	{
		if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK)
		{
			USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
		}
		else
		{
			USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
		}
		break;
	}
	case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc lost\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery return\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery land\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
		break;
	case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
		USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to near boundary\r\n");
		break;
	default:
		USER_LOG_INFO("[Event] Unknown joystick ctrl authority event\r\n");
	}

	return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void DjiTest_FlightControlVelocityAndYawRateCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate, uint32_t timeMs)
{
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	uint32_t originTime = 0;
	uint32_t currentTime = 0;
	uint32_t elapsedTimeInMs = 0;
	osalHandler->GetTimeMs(&originTime);
	osalHandler->GetTimeMs(&currentTime);
	elapsedTimeInMs = currentTime - originTime;
	T_DjiFlightControllerJoystickMode joystickMode =
	{
		DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
		DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
		DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
		DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
//		DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
		DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
	};

	DjiFlightController_SetJoystickMode(joystickMode);
	T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z, yawRate};

	while (elapsedTimeInMs <= timeMs)
	{
		DjiFlightController_ExecuteJoystickAction(joystickCommand);
		osalHandler->TaskSleepMs(2);
		osalHandler->GetTimeMs(&currentTime);
		elapsedTimeInMs = currentTime - originTime;
	}
}

void* flight_control_function(void* args)
{
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	T_DjiReturnCode returnCode;
	E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalVisualObstacleAvoidanceStatus;
	E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalRadarObstacleAvoidanceStatus;
	E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsVisualObstacleAvoidanceStatus;
	E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsRadarObstacleAvoidanceStatus;
	E_DjiFlightControllerObstacleAvoidanceEnableStatus downloadsVisualObstacleAvoidanceStatus;
	E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
	E_DjiFlightControllerRtkPositionEnableStatus rtkEnableStatus;
	E_DjiFlightControllerRCLostAction rcLostAction;
	T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

	T_DjiFlightControllerRidInfo ridInfo = {0};
//	ridInfo.latitude = 22.542812;
//	ridInfo.longitude = 113.958902;
//	ridInfo.altitude = 10;
	ridInfo.latitude  =  64.035921;
	ridInfo.longitude = -21.943566;
	ridInfo.altitude  =  10;

	returnCode = DjiFlightController_Init(ridInfo);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Initializing flight controller module failed, error code:0x%08llX", returnCode);
		return nullptr;
	}
	else
	{
		USER_LOG_INFO("Initialized flight controller module.");
	}

	returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
	                 DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT)
	{
		USER_LOG_ERROR("Register joystick control authority event callback failed, errno = 0x%08llX", returnCode);
		return NULL;
	}

	returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("get aircraft base info error");
	}

	/*! Turn on horizontal vision avoid enable */
	USER_LOG_INFO("Turning on horizontal visual obstacle avoidance");
	DjiTest_WidgetLogAppend("Turning on horizontal visual obstacle avoidance");
	returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Turning on horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
	};
	USER_LOG_INFO("Getting horizontal visual obstacle status");
	DjiTest_WidgetLogAppend("Getting horizontal horizontal visual obstacle status");
	returnCode = DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(& horizontalVisualObstacleAvoidanceStatus);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Getting horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
	}
	USER_LOG_INFO("Current horizontal visual obstacle avoidance status is %d",
	              horizontalVisualObstacleAvoidanceStatus);

	/*! Turn on horizontal radar avoid enable */
	USER_LOG_INFO("Turning on horizontal radar obstacle avoidance");
	DjiTest_WidgetLogAppend("Turning on horizontal radar obstacle avoidance");
	if (baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
	        baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
	        baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
	        baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK)
	{
		returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Turning on horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
		};
	}

	USER_LOG_INFO("Getting horizontal radar obstacle avoidance status");
	DjiTest_WidgetLogAppend("Getting horizontal radar obstacle avoidance status");
	if (baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
	        baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
	        baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
	        baseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK)
	{
		returnCode = DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus( & horizontalRadarObstacleAvoidanceStatus );
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Get horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
		}
	}
	/*! Turn on upwards vision avoid enable */
	USER_LOG_INFO("Turning on upwards visual obstacle avoidance.");
	DjiTest_WidgetLogAppend("Turning on upwards visual obstacle avoidance.");
	returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
	                 DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Turning on upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
	};

	USER_LOG_INFO("Getting upwards visual obstacle avoidance status");
	DjiTest_WidgetLogAppend("Getting upwards visual obstacle avoidance status");
	returnCode = DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
	                 &upwardsVisualObstacleAvoidanceStatus);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Getting upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
	}
	USER_LOG_INFO("Current upwards visual obstacle avoidance status is %d", upwardsVisualObstacleAvoidanceStatus);

	/*! Turn on upwards radar avoid enable */
	USER_LOG_INFO("Turning on upwards radar obstacle avoidance.");
	DjiTest_WidgetLogAppend("Turning on upwards radar obstacle avoidance.");
	if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
	        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
	        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
	        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK)
	{
		returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
		                 DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Turn on upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
		}
	}

	USER_LOG_INFO("Getting upwards radar obstacle avoidance status");
	DjiTest_WidgetLogAppend("Getting upwards radar obstacle avoidance status");
	if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
	        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
	        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
	        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK)
	{
		returnCode = DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
		                 &upwardsRadarObstacleAvoidanceStatus);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Getting upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
		}
	}


	// local variables
	bool previous_autonomous_control = autonomous_control;
	while( running )
	{
		// obtain or release joystick authority based on the value of autonomous_control
		if( autonomous_control )
		{
			if( ! previous_autonomous_control )
			{
				cout << "enabling autonomous control!" << endl;

//				T_DjiFlightControllerJoystickMode joystickMode =
//				{
//					DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
//					DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
//					DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
//					DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
//					DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
//				};


				USER_LOG_INFO("Obtaining joystick control authority.");
				DjiTest_WidgetLogAppend("Obtaining joystick control authority.");
				returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
				{
					USER_LOG_ERROR("Obtaining joystick authority failed, error code: 0x%08X", returnCode);
				}
				else
				{
					USER_LOG_INFO("Obtained joystick control authority.");
				}
//				DjiFlightController_SetJoystickMode(joystickMode);
			}
		}
		else
		{
			if( previous_autonomous_control )
			{
				cout << "disabling autonomous control!" << endl;
				USER_LOG_INFO("Releasing joystick authority");
				DjiTest_WidgetLogAppend("Releasing joystick authority");
				returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
				{
					USER_LOG_ERROR("Releasing joystick authority failed, error code: 0x%08X", returnCode);
				}
				else
				{
					USER_LOG_INFO("Released joystick authority");
				}
			}
		}

		// track variables
		previous_autonomous_control = autonomous_control;

		// implement control
		if( autonomous_control )
		{
			double forward     =   0.0; // positive is forward
			double right       =   0.0; // positive is right
			double up          =   0.0; // positive is up
			double yaw_rate_cw =   0.0; // positive is cw

			cout << "Controlling joysticks with FRUY = {"
			     << forward
			     << ", "
			     << right
			     << ", "
			     << up
			     << ", "
			     << yaw_rate_cw
			     << "}"
			     << endl;

			T_DjiFlightControllerJoystickMode joystickMode =
			{
				DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
				DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
				DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
				DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
//				DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
				DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
			};
			DjiFlightController_SetJoystickMode(joystickMode);
			T_DjiFlightControllerJoystickCommand joystickCommand = {forward, right, up, yaw_rate_cw};
			DjiFlightController_ExecuteJoystickAction(joystickCommand);

//			DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {0.0, 0.0, 0.0}, 20, 2000);
//			DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f){forward, right, up}, yaw_rate_cw, 200);
//			osalHandler->TaskSleepMs(2);

		}

		if( automatic_landing )
		{
			cout << "automatic landing!" << endl;
			automatic_landing = false;
			djiStat = DjiFlightController_StartLanding();
			if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
			{
				USER_LOG_ERROR("Start landing failed, error code: 0x%08X", djiStat);
			}
			else
			{
				USER_LOG_INFO("Started landing!");
			}
			DjiTestWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, automatic_landing_index, false, nullptr);
		}

		if( automatic_takeoff )
		{
			cout << "automatic takeoff!" << endl;
			automatic_takeoff = false;
			djiStat = DjiFlightController_StartTakeoff();
			if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
			{
				USER_LOG_ERROR("Start takeoff failed, error code: 0x%08X", djiStat);
			}
			else
			{
				USER_LOG_INFO("Started takeoff!");
			}
			DjiTestWidget_SetWidgetValue(DJI_WIDGET_TYPE_SWITCH, automatic_takeoff_index, false, nullptr);
		}

		// loop sleep (20 Hz)
		std::chrono::milliseconds sleep_duration(50);
		std::this_thread::sleep_for(sleep_duration);
	}
	
	returnCode = DjiFlightController_DeInit();
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("De-initializing flight controller module failed, error code:0x%08llX", returnCode);
	}
	else
	{
		USER_LOG_INFO("De-initialized flight controller module.");
	}

	USER_LOG_INFO("Leaving flight_control_thread.");
}

double constrain(double value, double lower, double upper)
{
	double result = value;
	if(      result < lower )
	{
		result = lower;
	}
	else if( result > upper )
	{
		result = upper;
	}
	return result;
}

void* gimbal_control_function(void* args)
{
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	T_DjiReturnCode returnCode;
	T_DjiGimbalManagerRotation rotation;

	// instantiate speed PID controllers
	PID pid_u_s = PID(0.1, 100, -100, 500, 150, 2.0);
	PID pid_v_s = PID(0.1, 100, -100, 500, 150, 2.0);

	// instantiate angle PID controllers
//	PID pid_u_a = PID(0.1, 10, -10, 2.5, 0.2, 0);
//	PID pid_v_a = PID(0.1, 10, -10, 2.5, 0.2, 0);
	PID pid_u_a = PID(0.1, 2, -2, 5, 1.5, 0.1);
	PID pid_v_a = PID(0.1, 2, -2, 5, 1.5, 0.1);

	USER_LOG_INFO("Starting gimbal_control_thread.");

	// initialize
	returnCode = DjiGimbalManager_Init();
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Init gimbal manager failed, error code: 0x%08X", returnCode);
		return nullptr;
	}

	// set gimbal mode
	if (gimbal_mode == DJI_GIMBAL_MODE_FREE)
	{
		USER_LOG_INFO("Setting gimbal to free mode");
	}
	else if (gimbal_mode == DJI_GIMBAL_MODE_YAW_FOLLOW)
	{
		USER_LOG_INFO("Setting gimbal to yaw follow mode");
	}
	returnCode = DjiGimbalManager_SetMode(gimbal_mount_position, gimbal_mode);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Set gimbal mode failed, error code: 0x%08X", returnCode);
		return nullptr;
	}

	returnCode = DjiGimbalManager_SetControllerSmoothFactor(gimbal_mount_position, DJI_GIMBAL_AXIS_YAW,   (uint8_t) 10.0);
	returnCode = DjiGimbalManager_SetControllerSmoothFactor(gimbal_mount_position, DJI_GIMBAL_AXIS_PITCH, (uint8_t) 10.0);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Failed!");
	}

	// reset gimbal angles
	returnCode = DjiGimbalManager_Reset(gimbal_mount_position, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
	}

	/*
		typedef struct {
		    E_DjiGimbalRotationMode rotationMode;	// Gimbal rotation mode
		    dji_f32_t pitch;				// Pitch angle in degree, unit: deg
		    dji_f32_t roll;				// Roll angle in degree, unit: deg
		    dji_f32_t yaw;				// Yaw angle in degree, unit: deg
		    dji_f64_t time;				// Expected execution time of gimbal rotation, unit: second.
		} T_DjiGimbalManagerRotation;
	*/

	bool old_aim_gimbal = aim_gimbal; // to track for edges

	while( running )
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - apriltag_detection_timestamp);

		double u,v;
		if( duration.count() < 100 && global_image_half_width != 0 && global_image_half_height != 0 )
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
			else if( current_stream_source == DJI_CAMERA_MANAGER_SOURCE_IR_CAM )
			{
				theta_u = u_n * atan( SENSOR_WIDTH_ZOOM  / (2 * FOCAL_LENGTH_IR ) );
				theta_v = v_n * atan( SENSOR_HEIGHT_ZOOM / (2 * FOCAL_LENGTH_IR ) );
			}
			else
			{
				USER_LOG_ERROR("Stream source error in gimbal aim function!");
			}

			if( aim_gimbal )
			{
				T_DjiGimbalManagerRotation rotation;

				theta_u *= RAD_TO_DEG;
				theta_v *= RAD_TO_DEG;

#if GIMBAL_AIM_MODE_ANGLE
				rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
				rotation.pitch =  -1.0 * theta_v;
				rotation.roll  =   0.0;
				rotation.yaw   =   1.0 * theta_u;
				rotation.time  =   0.5;
#else
				rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_SPEED;

				double zoom_factor = ZOOM_SCALAR * opticalZoomParam.currentOpticalZoomFactor / 2;
				double   yaw_speed =  SPEED_FACTOR * theta_u * zoom_factor;
				double pitch_speed = -SPEED_FACTOR * theta_v * zoom_factor;

				yaw_speed = constrain( yaw_speed, -MAX_SPEED, MAX_SPEED );
				pitch_speed = constrain( pitch_speed, -MAX_SPEED, MAX_SPEED );

				rotation.pitch =   pitch_speed;
				rotation.roll  =   0.0;
				rotation.yaw   =   yaw_speed;
				rotation.time  =   0.05;
#endif
				cout << theta_u << " " << theta_v << endl;

				returnCode = DjiGimbalManager_Rotate(gimbal_mount_position, rotation);
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
				{
					cerr << "error moving the gimbal!" << endl;
				}
			}
		}

#if GIMBAL_AIM_MODE_ANGLE
		std::chrono::milliseconds sleep_duration(500);
#else
		std::chrono::milliseconds sleep_duration(100);
#endif
		std::this_thread::sleep_for(sleep_duration);
	}

	USER_LOG_INFO("Resetting gimbal angles.");
	returnCode = DjiGimbalManager_Reset(gimbal_mount_position, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
	}
	USER_LOG_INFO("Leaving gimbal_control_thread.");
}

static uint8_t DjiTest_CameraManagerGetCameraTypeIndex(E_DjiCameraType cameraType)
{
	uint8_t i;

	for (i = 0; i < sizeof(s_cameraTypeStrList) / sizeof(s_cameraTypeStrList[0]); i++)
	{
		if (s_cameraTypeStrList[i].cameraType == cameraType)
		{
			return i;
		}
	}

	return 0;
}

void* camera_management_function(void* args)
{
	USER_LOG_INFO("Entering camera_management_thread");

	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	T_DjiReturnCode returnCode;
	E_DjiCameraType cameraType;
	T_DjiCameraManagerFirmwareVersion firmwareVersion;
	T_DjiCameraManagerFocusPosData focusPosData;
	T_DjiCameraManagerTapZoomPosData tapZoomPosData;

	E_DjiMountPosition mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;

	osalHandler->TaskSleepMs(500);

	USER_LOG_INFO("Initializing camera manager module.");
	returnCode = DjiCameraManager_Init();
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X\r\n", returnCode);
		return nullptr;
	}

	osalHandler->TaskSleepMs(500);

	USER_LOG_INFO("Getting camera type and version.");
	returnCode = DjiCameraManager_GetCameraType(mountPosition, &cameraType);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Getting mounted position %d camera's type failed, error code: 0x%08X\r\n",
		               mountPosition, returnCode);
		return nullptr;
	}
	USER_LOG_INFO("Mounted position %d camera's type is %s",
	              mountPosition,
	              s_cameraTypeStrList[DjiTest_CameraManagerGetCameraTypeIndex(cameraType)].cameraTypeStr);

	returnCode = DjiCameraManager_GetFirmwareVersion(mountPosition, &firmwareVersion);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("Getting mounted position %d camera's firmware version failed, error code: 0x%08X\r\n",
		               mountPosition, returnCode);
		return nullptr;
	}
	USER_LOG_INFO("Mounted position %d camera's firmware is V%02d.%02d.%02d.%02d\r\n", mountPosition,
	              firmwareVersion.firmware_version[0], firmwareVersion.firmware_version[1],
	              firmwareVersion.firmware_version[2], firmwareVersion.firmware_version[3]);

	dji_f32_t default_zoom_factor = 2;
	USER_LOG_INFO("Setting mounted position %d camera's zoom factor: %0.1f x.", mountPosition, default_zoom_factor);
	returnCode = DjiCameraManager_SetOpticalZoomParam(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_IN, default_zoom_factor);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
	        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
	{
		USER_LOG_INFO("Setting mounted position %d camera's zoom factor(%0.1f) failed, error code :0x%08X",
		              mountPosition, default_zoom_factor, returnCode);
	}

	returnCode = DjiCameraManager_SetFocusMode(mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
	        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
	{
		USER_LOG_ERROR("Setting mounted position %d camera's focus mode(%d) failed,"
		               " error code :0x%08X.", mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO,
		               returnCode);
	}

//	T_DjiCameraManagerFocusPosData focus_point;
//	focus_point.focusX = 1920 / 2;
//	focus_point.focusY = 1440 / 2;
//	USER_LOG_INFO("Set mounted position %d camera's focus point to (%0.1f, %0.1f).",
//		  mountPosition, apriltag_detection.c[0], apriltag_detection.c[1]);
//	returnCode = DjiCameraManager_SetFocusTarget(mountPosition, focus_point);
//	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
//	    returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
//	    USER_LOG_ERROR("Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
//		       " error code :0x%08X.", mountPosition, focus_point.focusX, focus_point.focusY,
//		       returnCode);
//	}


//typedef enum {
//    DJI_CAMERA_MANAGER_SOURCE_DEFAULT_CAM = 0x0,
//    DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM = 0x1,
//    DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM = 0x2,
//    DJI_CAMERA_MANAGER_SOURCE_IR_CAM = 0x3,
//    DJI_CAMERA_MANAGER_SOURCE_VISIBLE_CAM = 0x7,
//} E_DjiCameraManagerStreamSource;

	osalHandler->TaskSleepMs(500);
	while( running )
	{
		// switch between RGB and IR
		if( intended_stream_rgb )
		{
			if( intended_stream_source != DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM
			        && intended_stream_source != DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM )
			{
				intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM;
			}
		}
		else
		{
			if( intended_stream_source != DJI_CAMERA_MANAGER_SOURCE_IR_CAM )
			{
				intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_IR_CAM;
			}
		}

		// switch stream sources if necessary
		intended_stream_source_mutex.lock();
		returnCode = DjiCameraManager_SetStreamSource(mountPosition, intended_stream_source);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Failed to set the stream source!");
		}
		else
		{
			current_stream_source_mutex.lock();
			current_stream_source = intended_stream_source;
			current_stream_source_mutex.unlock();
		}
		intended_stream_source_mutex.unlock();

		// get zoom factor of zoom camera
		returnCode = DjiCameraManager_GetOpticalZoomParam(mountPosition, &opticalZoomParam);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
		{
			USER_LOG_ERROR("Getting mounted position %d camera's zoom param failed, error code :0x%08X", mountPosition, returnCode);
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
				if( apriltag_detection_area != 0 && apriltag_detection_area < 0.015 )
				{
					// zoom in
					USER_LOG_INFO("Landing pad too small - zooming in...");
					returnCode = DjiCameraManager_StartContinuousOpticalZoom(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_IN, DJI_CAMERA_ZOOM_SPEED_SLOW);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
					{
						USER_LOG_ERROR("Mounted position %d camera start continuous zoom failed,"
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
						returnCode = DjiCameraManager_StartContinuousOpticalZoom(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_OUT, DJI_CAMERA_ZOOM_SPEED_SLOW);
						if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
						{
							USER_LOG_ERROR("Mounted position %d camera start continuous zoom failed,"
							               " error code :0x%08X.", mountPosition, returnCode);
						}
					}
					else
					{
						USER_LOG_INFO("Trying to switch to wide angle camera...");
						intended_stream_source = DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM;
					}
				}
				// if the landing pad is an ok size
				else if( apriltag_detection_area != 0 )
				{
					USER_LOG_INFO("Landing pad ok size - keeping zoom level constant.");

					returnCode = DjiCameraManager_StopContinuousOpticalZoom(mountPosition);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
					        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
					{
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
		else     // if the landing pad detection has timed out
		{
			// stop any continuous zooming
			returnCode = DjiCameraManager_StopContinuousOpticalZoom(mountPosition);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
			{
				USER_LOG_ERROR("Mounted position %d camera stop continuous zoom failed,"
				               " error code :0x%08X", mountPosition, returnCode);
			}
		}
		osalHandler->TaskSleepMs(100);
	}

	returnCode = DjiCameraManager_SetFocusMode(mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
	        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
	{
		USER_LOG_ERROR("Setting mounted position %d camera's focus mode(%d) failed,"
		               " error code :0x%08X.", mountPosition, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO,
		               returnCode);
	}

	USER_LOG_INFO("Setting mounted position %d camera's zoom factor: %0.1f x.", mountPosition, default_zoom_factor);
	returnCode = DjiCameraManager_SetOpticalZoomParam(mountPosition, DJI_CAMERA_ZOOM_DIRECTION_IN, default_zoom_factor);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
	        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
	{
		USER_LOG_INFO("Setting mounted position %d camera's zoom factor(%0.1f) failed, error code :0x%08X",
		              mountPosition, default_zoom_factor, returnCode);
	}

	USER_LOG_INFO("Exiting camera_management_thread.");
}

/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{
	signal(SIGINT, graceful_exit);
	signal(SIGTERM, graceful_exit);

	Application application(argc, argv);
	char inputChar;
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	T_DjiReturnCode returnCode;
	T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

	initialize();

	while(running)
	{
		std::chrono::milliseconds sleep_duration(1000);
		std::this_thread::sleep_for(sleep_duration);
	}

	deinitialize();

	exit(0);
}

static void apriltag_image_callback(CameraRGBImage img, void *userData)
{
	apriltag_detection_area = 0; // 0 -> no detection

	// hold the image in a OpenCV matrix, convert fromt RGB to BGR
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

#if VISUALIZE
	// Draw detection outlines
	for (int i = 0; i < zarray_size(detections); i++)
	{
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

#if VISUALIZE
	string window_name = string(reinterpret_cast<char *>(userData));
	imshow(window_name, mat);
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
