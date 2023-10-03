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
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"

using namespace std;
using namespace cv;

// global variables :(
bool running = true; // can only be changed to false only in one single place
pthread_t subscription_thread;
T_DjiReturnCode djiStat;
T_DjiFcSubscriptionGimbalAngles gimbal_angles;
T_DjiDataTimestamp timestamp = {0};
mutex fcu_subscription_mutex;

pthread_t gimbal_control_thread;
E_DjiMountPosition gimbal_mount_position = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
E_DjiGimbalMode gimbal_mode = DJI_GIMBAL_MODE_FREE;
mutex apriltag_detection_mutex;
apriltag_detection_t apriltag_detection;
chrono::system_clock::time_point apriltag_detection_timestamp = chrono::system_clock::from_time_t(0);

const char* camera_name = "MAIN_CAM";
LiveviewSample* liveviewSample;// = new LiveviewSample();
apriltag_family_t *tf = nullptr;
apriltag_detector_t *td = nullptr;
double global_image_half_width  = 0;
double global_image_half_height = 0;

// function declarations
void init();
void deinit();
void sigint_handler();
void* subscription_thread_function(void* args);
void* gimbal_control_function(void* args);
static void apriltag_image_callback(CameraRGBImage img, void *userData);

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit();
static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState);

void sigint_handler(int signal)
{
	running = false;
	cout << "sending signal for graceful exit..." << endl;
}

void init()
{
    // initialize fcu subscription thread
    if( pthread_create( & subscription_thread, NULL, subscription_thread_function, NULL ) )
    {
	cout << "problem" << endl;
	return deinit();
    }
    
    // initialize gimbal control thread
    if( pthread_create( & gimbal_control_thread, NULL, gimbal_control_function, NULL ) )
    {
	cout << "problem" << endl;
	return deinit();
    }

    LiveviewSample* liveviewSample = new LiveviewSample();
    liveviewSample->joshua_start_camera_stream_main(&apriltag_image_callback, &camera_name);
}

void deinit()
{
    cout << "deinitializing..." << endl;
    cout << "joining data retrieval thread" << endl;
    pthread_join( subscription_thread, NULL );
    cout << "joining data gimbal control thread" << endl;
    pthread_join( gimbal_control_thread, NULL );
    cout << "all threads joined" << endl;

    // liveviewSample->stop_camera_stream
    cout << "deleting liveview object" << endl;
    delete liveviewSample;
    cout << "deleted liveview object" << endl;
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

	while(running)
	{
		fcu_subscription_mutex.lock();
		djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
								  (uint8_t *) &gimbal_angles,
								  sizeof(T_DjiFcSubscriptionGimbalAngles),
								  &timestamp);
		if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		    USER_LOG_ERROR("Error getting the gimbal angles!");
		}
		fcu_subscription_mutex.unlock();

		std::chrono::milliseconds sleep_duration(20);
		std::this_thread::sleep_for(sleep_duration);
	}

	cout << "unsubscribing from fcu topics" << endl;
	djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
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

//    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//    {
//        USER_LOG_ERROR("Failed to subscribe topic %d, 0x%08X", DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, returnCode);
//        return nullptr;
//    }

    // initialize
    USER_LOG_INFO("--> Step 1: Init gimbal manager module");
    DjiTest_WidgetLogAppend("--> Step 1: Init gimbal manager module");
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
    USER_LOG_INFO("--> Step 3: Reset gimbal angles.\r\n");
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

    while( running )
    {
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - apriltag_detection_timestamp);
	cout << "time since apriltag detection:" << duration.count() << endl;


	double u,v;
	if( duration.count() < 500 && global_image_half_width != 0 && global_image_half_height != 0 )
	{
		apriltag_detection_mutex.lock();
		u = apriltag_detection.c[0];
		v = apriltag_detection.c[1];
		apriltag_detection_mutex.unlock();
		double u_n = (u - global_image_half_width)  / global_image_half_width;
		double v_n = (v - global_image_half_height) / global_image_half_height;

		cout << "in gimbal control thread" << endl;
		T_DjiGimbalManagerRotation rotation;
		rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE; 
	//	rotation.pitch = -0.1;
	//	rotation.roll  =  0.0;
	//	rotation.yaw   = -0.1;
		rotation.pitch =  - speed_factor_pitch * v_n;
		rotation.roll  =  0.0;
		rotation.yaw   =    speed_factor_yaw * u_n;
		rotation.time  = 2.0;
	      
		cout << endl;
		cout << "u: " << u << " , u_n: " << u_n << " , half_width:  " << global_image_half_width << endl;
		cout << "v: " << v << " , v_n: " << v_n << " , half_height: " << global_image_half_height << endl;
		cout << endl;

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

//void* gimbal_control_function(void* args)
//{
//	DjiUser_RunGimbalManagerSample();
////	DjiTest_GimbalManagerRunSample();
//	return nullptr;
//}

/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{
    signal(SIGINT, sigint_handler);

    Application application(argc, argv);
    char inputChar;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

    init();

    while(running)
    {
	    cout << "main loop" << endl;
	    fcu_subscription_mutex.lock();
	    cout << "gimbal PRY: "
		 << gimbal_angles.x
		 << " , "
		 << gimbal_angles.y
		 << " , "
		 << gimbal_angles.z
		 << endl;
	    fcu_subscription_mutex.unlock();

	    std::chrono::milliseconds sleep_duration(500);
	    std::this_thread::sleep_for(sleep_duration);
    }

    deinit();

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

//	    if( i == 0 )
//	    {
//		    cout << "( " << det->c[0] << " , " << det->c[1] << " )" << endl;
//	    }
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
