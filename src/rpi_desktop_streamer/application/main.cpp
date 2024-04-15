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
#include <chrono>

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
#include "dji_high_speed_data_channel.h"
#include "dji_aircraft_info.h"

/* Includes from camera_emu ------------------------------------------------------------------*/
#include <fcntl.h>
#include <stdlib.h>
#include "dji_logger.h"
#include "utils/util_misc.h"
#include "utils/util_time.h"
#include "utils/util_file.h"
#include "utils/util_buffer.h"
#include "camera_emu/dji_media_file_manage/dji_media_file_core.h"
#include "dji_high_speed_data_channel.h"
#include "dji_aircraft_info.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libswscale/swscale.h>
#include <X11/Xlib.h>
}

#include <SDL2/SDL.h>

#define WIDTH 640
#define HEIGHT 480
#define FPS 24
#define BITRATE 2000000

#define DISPLAY 0
#define CONNECT_PSDK 1

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------*/

/* camera_emu stuff ----------------------------------------------------- */
typedef enum
{
	TEST_PAYLOAD_CAMERA_MEDIA_PLAY_COMMAND_STOP = 0,
	TEST_PAYLOAD_CAMERA_MEDIA_PLAY_COMMAND_PAUSE = 1,
	TEST_PAYLOAD_CAMERA_MEDIA_PLAY_COMMAND_START = 2,
} E_TestPayloadCameraPlaybackCommand;

typedef struct
{
	uint8_t isInPlayProcess;
	uint16_t videoIndex;
	char filePath[DJI_FILE_PATH_SIZE_MAX];
	uint32_t videoLengthMs;
	uint64_t startPlayTimestampsUs;
	uint64_t playPosMs;
} T_DjiPlaybackInfo;

typedef struct
{
	E_TestPayloadCameraPlaybackCommand command;
	uint32_t timeMs;
	char path[DJI_FILE_PATH_SIZE_MAX];
} T_TestPayloadCameraPlaybackCommand;
static T_DjiSemaHandle s_mediaPlayWorkSem = NULL;
static T_DjiMutexHandle s_mediaPlayCommandBufferMutex = {0};
static T_DjiTaskHandle s_userSendVideoThread;
static T_DjiMutexHandle s_commonMutex = {0};
static T_DjiCameraCommonHandler s_commonHandler;
static E_DjiCameraVideoStreamType s_cameraVideoStreamType;
static bool s_isCamInited = false;
static uint8_t s_mediaPlayCommandBuffer[sizeof(T_TestPayloadCameraPlaybackCommand) * 32] = {0};
static T_UtilBuffer s_mediaPlayCommandBufferHandler = {0};

/* Private values -------------------------------------------------------------*/
volatile bool running = true;
bool psdk_connector_running = false;
Application* application = NULL;
T_DjiOsalHandler *osalHandler = NULL;

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
	signal(SIGQUIT, signal_handler);  // CTRL+BACKSLASH
	signal(SIGTERM, signal_handler);  // Termination request

	// start the DJI PSDK connector
#if CONNECT_PSDK
	application = new Application(argc, argv);
	osalHandler = DjiPlatform_GetOsalHandler();
#endif

	// start the video thread
	pthread_t video_processing_thread;
	int result = 0;
	result = pthread_create( & video_processing_thread, NULL, video_processing_function, NULL);
	if( result != 0 )
	{
		perror("Video processing thread creation failed!");
		exit(EXIT_FAILURE);
	}

	// application-specific tasks
	while( running )
	{
		//cout << "Main thread loop" << endl;
		sleep(2);
	}

	psdk_connector_running = false;

	pthread_join( video_processing_thread, NULL );

	usleep(500000);

	if( application )
	{
//		T_DjiReturnCode returnCode;
//		if (osalHandler->MutexUnlock(s_mediaPlayCommandBufferMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//		{
//			USER_LOG_ERROR("mutex unlock error");
//			return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
//		}
		cout << "Shutting down the PSDK connector..." << endl;
		application->~Application();
	}

	cout << "Successfully shut down the PSDK connector!" << endl;
	cout << "Ending the program now." << endl;

	return 0;
}

void signal_handler(int signum)
{
	running = false;
}

void* video_processing_function()
{
	cout << "Starting the video processing function!" << endl;

	T_DjiReturnCode returnCode;
	if( application != NULL )
	{
		cout << "Setting bandwidth proportion." << endl;
		const T_DjiDataChannelBandwidthProportionOfHighspeedChannel bandwidthProportionOfHighspeedChannel = {10, 60, 30};
		returnCode = DjiHighSpeedDataChannel_SetBandwidthProportion(bandwidthProportionOfHighspeedChannel);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("Set data channel bandwidth width proportion error.");
		}

		returnCode = DjiPayloadCamera_SetVideoStreamType(DJI_CAMERA_VIDEO_STREAM_TYPE_H264_CUSTOM_FORMAT);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("DJI camera set video stream error.");
		}

		cout << "Starting camera_emu base service..." << endl;
		returnCode = DjiTest_CameraEmuBaseStartService();
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			USER_LOG_ERROR("camera emu common init error");
		}

//		usleep(1000000);
//
//		cout << "Startup wait..." << endl;
//
//		usleep(1000000);

		cout << "PSDK connector started!" << endl;
		psdk_connector_running = true;
	}
	else
	{
		cout << "PSDK connector not starting!" << endl;
	}

	// Initialize ffmpeg codec and context
	avdevice_register_all();
	AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
	if (!codec)
	{
		cout << "Codec not found" << endl;
		signal_handler(SIGTERM);
	}
	cout << "Found codec" << endl;

	AVCodecContext *codecContext = avcodec_alloc_context3(codec);
	if (!codecContext)
	{
		cout << "Could not allocate video codec context" << endl;
		signal_handler(SIGTERM);
	}
	cout << "Allocated video codec context" << endl;

	// Set codec parameters
	codecContext->width 		= WIDTH;
	codecContext->height		= HEIGHT;
	codecContext->bit_rate		= BITRATE;
	codecContext->time_base		= {1, FPS};
	codecContext->framerate		= {FPS, 1};
	codecContext->gop_size		= 2;
	codecContext->max_b_frames	= 0;
	codecContext->pix_fmt 		= AV_PIX_FMT_YUV420P;

	int result;
	result = av_opt_set(codecContext, "threads", "2", 0);
	cout << "result for setting num threads: " << result << endl;
	result = av_opt_set(codecContext->priv_data, "preset", "ultrafast", 0);
	cout << "result for setting ultrafast: " << result << endl;
	result = av_opt_set(codecContext->priv_data, "tune", "zerolatency", 0);
	cout << "result for setting zerolatency: " << result << endl;

	if (avcodec_open2(codecContext, codec, nullptr) < 0)
	{
		cout << "Could not open codec context" << endl;
		signal_handler(SIGTERM);
	}
	cout << "Opened codec context" << endl;

	// Allocate AVFrame
	AVFrame *avFrame	= av_frame_alloc();
	avFrame->format		= codecContext->pix_fmt;
	avFrame->width		= codecContext->width;
	avFrame->height		= codecContext->height;

	// Allocate buffer for AVFrame
	result = av_frame_get_buffer(avFrame, 0);
	if (result < 0)
	{
		cout << "Error allocating frame buffer!" << endl;
		return -1;
	}

	// Command to capture screen using ffmpeg
	std::ostringstream ffmpeg_command_stream;
	ffmpeg_command_stream << "ffmpeg -loglevel quiet -f x11grab -r " << FPS << " -s " << WIDTH << "x" << HEIGHT << " -i :0.0 -f rawvideo -pix_fmt yuv420p -";
	std::string ffmpeg_cmd = ffmpeg_command_stream.str();

	// Open subprocess to capture screen
	FILE* ffmpeg_pipe = popen(ffmpeg_cmd.c_str(), "r");
	if( ! ffmpeg_pipe )
	{
		std::cout << "Error: Unable to open ffmpeg subprocess." << std::endl;
		return 1;
	}

// Allocate memory for each plane
	uint8_t bufferY[WIDTH * HEIGHT];  // Y plane
	uint8_t bufferU[WIDTH * HEIGHT / 4]; // U plane
	uint8_t bufferV[WIDTH * HEIGHT / 4]; // V plane

	const std::chrono::duration<double> desired_period(1.0 / FPS);
	std::chrono::steady_clock::time_point loop_start_time;
	std::chrono::duration<double> elapsed_time;

	std::chrono::steady_clock::time_point last_status_time = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_since_last_status;

	// Main loop to read and display frames
	while( running )
	{
		loop_start_time = std::chrono::steady_clock::now();

		// Allocate memory for the raw frame
		std::vector<uint8_t> raw_frame(WIDTH * HEIGHT * 3 / 2);

		// Read raw YUV420p frame from ffmpeg pipe
		size_t bytes_read = fread(reinterpret_cast<char*>(raw_frame.data()), 1, raw_frame.size(), ffmpeg_pipe);

		// Ensure that the correct number of bytes are read
		if (bytes_read != raw_frame.size())
		{
			std::cout << "Error: Failed to read complete frame from ffmpeg pipe." << std::endl;
			break;
		}

#if DISPLAY
		// Convert YUV420p to BGR
		cv::Mat yuv(HEIGHT * 3 / 2, WIDTH, CV_8UC1, raw_frame.data());
		cv::Mat bgr;
		cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);

		// Display the frame
		cv::imshow("Desktop", bgr);

		// Check for 'q' key press to exit
		if (cv::waitKey(1) == 'q')
		{
			break;
		}
#endif
		// Copy data from raw_frame to AVFrame
		for (int i = 0; i < codecContext->height; ++i)
		{
			memcpy(avFrame->data[0] + i * avFrame->linesize[0], raw_frame.data() + i * WIDTH, codecContext->width);
		}

		for (int i = 0; i < codecContext->height / 2; ++i)
		{
			memcpy(avFrame->data[1] + i * avFrame->linesize[1], raw_frame.data() + WIDTH * HEIGHT + i * WIDTH / 2, codecContext->width / 2);
			memcpy(avFrame->data[2] + i * avFrame->linesize[2], raw_frame.data() + WIDTH * HEIGHT * 5 / 4 + i * WIDTH / 2, codecContext->width / 2);
		}

		int ret = avcodec_send_frame(codecContext, avFrame);
		if (ret < 0)
		{
			std::cout << "Error sending frame to codec context!" << std::endl;
		}

		// Encode a packet
		AVPacket *packet = av_packet_alloc();
		if (!packet)
		{
			std::cout << "Could not allocate video packet" << std::endl;
			return -1;
		}
		av_init_packet(packet);
		packet->data = nullptr;
		packet->size = 0;

		result = avcodec_receive_packet(codecContext, packet);
		if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
		{
			cout << "AVERROR encoding frame!" << endl;
			continue;
		}
		else if (result < 0)
		{
			cout << "Error encoding frame!" << endl;
			continue;
		}
		else if( 0 == result )
		{
			// Process the packet (copy the data out)
			// Allocate memory for the encoded data
			uint8_t *encoded_data = new uint8_t[packet->size];
			if (!encoded_data)
			{
				std::cout << "Failed to allocate memory for encoded data" << std::endl;
				return -1;
			}

			// Copy the packet data into the allocated memory
			std::copy(packet->data, packet->data + packet->size, encoded_data);

			if( psdk_connector_running )
			{
				T_DjiReturnCode returnCode;
				int sent_data_size = 0;
				while( packet->size - sent_data_size )
				{
					int sending_data_size = packet->size - sent_data_size;
					if( sending_data_size > 60000 )
					{
						sending_data_size = 60000;
					}

					cout << "packet size: " << packet->size << ", sending size: " << sending_data_size << endl;

					T_DjiReturnCode returnCode;
					returnCode = DjiPayloadCamera_SendVideoStream((const uint8_t *) encoded_data + sent_data_size, sending_data_size);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
					{
						USER_LOG_ERROR("send video stream error: 0x%08llX.", returnCode);
					}
					sent_data_size += sending_data_size;
				}

				time_since_last_status = std::chrono::steady_clock::now() - last_status_time;
				if( time_since_last_status > std::chrono::duration<double>(1) )
				{
					T_DjiDataChannelState videoStreamState = {0};
					returnCode = DjiPayloadCamera_GetVideoStreamState(&videoStreamState);
					if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
					{
						cout << "video stream state: "
						     << endl
						     << "realtimeBandwidthLimit: "
						     << endl
						     << videoStreamState.realtimeBandwidthLimit
						     << endl
						     << "realtimeBandwidthBeforeFlowController: "
						     << endl
						     << videoStreamState.realtimeBandwidthBeforeFlowController
						     << endl
						     << "realtimeBandwidthAfterFlowController: "
						     << endl
						     << videoStreamState.realtimeBandwidthAfterFlowController
						     << endl
						     << "busyState: "
						     << endl
						     << videoStreamState.busyState
						     << endl;
					}
					last_status_time = std::chrono::steady_clock::now();
				}
			}

			// Free the encoded data
			delete[] encoded_data;
		}
		// Free the packet
		av_packet_unref(packet);

		elapsed_time = std::chrono::steady_clock::now() - loop_start_time;
		auto delay_duration = desired_period - elapsed_time;
		if (delay_duration > std::chrono::duration<double>(0))
		{
			std::this_thread::sleep_for(delay_duration);
		}
	}

	// Close ffmpeg subprocess
	pclose(ffmpeg_pipe);

	return 0;
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
