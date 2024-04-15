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
// standard includes
#include <thread>
#include <ctime>
#include <unistd.h>
#include <opencv2/opencv.hpp>

// dji includes
#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include <hms/test_hms.h>
#include <waypoint_v2/test_waypoint_v2.h>
#include <waypoint_v3/test_waypoint_v3.h>
#include <gimbal_manager/test_gimbal_manager.h>
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

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libswscale/swscale.h>
}

#include <SDL2/SDL.h>

#define WIDTH 640
#define HEIGHT 480

#define DISPLAY 1

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
bool running = true;
bool psdk_connector_running = false;

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit();
static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState);

void signal_handler(int signum);
void* video_processing_function(void);

using namespace std;
using namespace cv;

/* Exported functions definition ---------------------------------------------*/
int main(int argc, char **argv)
{
	signal(SIGINT,  signal_handler);  // CTRL+C
	signal(SIGKILL, signal_handler);  // CTRL+C
	signal(SIGQUIT, signal_handler); // CTRL+BACKSLASH
	signal(SIGTERM, signal_handler); // Termination request

	// start the video thread first because ffmpeg is already streaming video to it
	pthread_t video_processing_thread;
	int result = 0;
	result = pthread_create( & video_processing_thread, NULL, video_processing_function, NULL);
	if( result != 0 )
	{
		perror("Video processing thread creation failed!");
		exit(EXIT_FAILURE);
	}

	// start the DJI PSDK connector
	Application* application;
	application = new Application(argc, argv);
	char inputChar;
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	T_DjiReturnCode returnCode;
	T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

	returnCode = DjiTest_CameraEmuBaseStartService();
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
	{
		USER_LOG_ERROR("camera emu common init error");
	}

	if (DjiPlatform_GetSocketHandler() != nullptr)
	{
		returnCode = DjiTest_CameraEmuMediaStartService();
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("camera emu media init error");
		}
	}

	cout << "PSDK connector started!" << endl;
	psdk_connector_running = true;

	// application-specific tasks
	while( running )
	{
		cout << "Main thread loop" << endl;
		sleep(1);
	}

	psdk_connector_running = false;
	usleep(500000);

	cout << "Shutting down the PSDK connector..." << endl;
	application->~Application();
	cout << "Successfully shut down the PSDK connector!" << endl;
	cout << "Ending the program now." << endl;

	return 0;
}

void signal_handler(int signum)
{
	cout << "Signal received: " << signum << endl;

	running = false;
}

void* video_processing_function()
{
	// Initialize ffmpeg codec and context
	avcodec_register_all();
	AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
	if (!codec)
	{
		cerr << "Codec not found" << endl;
		return -1;
	}

	AVCodecContext *codecContext = avcodec_alloc_context3(codec);
	if (!codecContext)
	{
		cerr << "Could not allocate video codec context" << endl;
		return -1;
	}

	// Set codec parameters
	codecContext->width 		= WIDTH;
	codecContext->height		= HEIGHT;
	codecContext->bit_rate		= 400000;
	codecContext->time_base		= {1, 25};
	codecContext->gop_size		= 10;
	codecContext->max_b_frames	= 1;
	codecContext->pix_fmt 		= AV_PIX_FMT_YUV420P;

	if (avcodec_open2(codecContext, codec, nullptr) < 0)
	{
		cerr << "Could not open codec" << endl;
		return -1;
	}

	// Allocate AVFrame
	AVFrame *avFrame	= av_frame_alloc();
	avFrame->format		= codecContext->pix_fmt;
	avFrame->width		= codecContext->width;
	avFrame->height		= codecContext->height;

	// Allocate buffer for AVFrame
	int ret = av_frame_get_buffer(avFrame, 0);
	if (ret < 0)
	{
		cerr << "Error allocating frame buffer!" << endl;
		return -1;
	}

	while( running )
	{
// **************************************************************
		// Read raw video frame from stdin
		vector<uint8_t> raw_frame(640 * 480 * 3);
		cin.read(reinterpret_cast<uint8_t*>(raw_frame.data()), raw_frame.size());

		// Convert raw frame to Mat
		Mat frame(480, 640, CV_8UC3, raw_frame.data());

		// Rearrange channels from RGB to BGR
		cvtColor(frame, frame, COLOR_RGB2BGR);

// **************************************************************
		// Copy data from Mat to AVFrame
		for (int y = 0; y < codecContext->height; y++)
		{
			memcpy(avFrame->data[0] + y * avFrame->linesize[0], frame.data + y * frame.step, codecContext->width * 3);
		}

		// Encode frame
		ret = avcodec_send_frame(codecContext, avFrame);
		if (ret < 0)
		{
			cerr << "Error sending frame to codec!" << endl;
			return -1;
		}

		AVPacket pkt;
		av_init_packet(&pkt);

		ret = avcodec_receive_packet(codecContext, &pkt);
		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			continue;
		else if (ret < 0)
		{
			cerr << "Error encoding frame!" << endl;
			return -1;
		}

		if( psdk_connector_running )
		{
			cout << "*****************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************" << endl;
		}
// **************************************************************
#if DISPLAY
		// Display the frame
		imshow("Desktop", frame);
		// Check for the 'q' key to quit
		if (waitKey(1) == 'q')
		{
			break;
		}
#endif
	}
	// Release VideoCapture and destroy OpenCV windows
	destroyAllWindows();
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
