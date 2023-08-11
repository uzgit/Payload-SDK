/**
 ********************************************************************
 * @file    application.cpp
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
#include "application.hpp"
#include "dji_sdk_app_info.h"
#include <dji_platform.h>
#include <dji_logger.h>
#include <dji_core.h>
#include <dji_aircraft_info.h>
#include <csignal>
#include "dji_sdk_config.h"

#include "../common/osal/osal.h"
#include "../common/osal/osal_fs.h"
#include "../common/osal/osal_socket.h"
#include "../manifold2/hal/hal_usb_bulk.h"
#include "../manifold2/hal/hal_uart.h"
#include "../manifold2/hal/hal_network.h"

/* Private constants ---------------------------------------------------------*/
#define DJI_LOG_PATH                    "Logs/DJI"
#define DJI_LOG_INDEX_FILE_NAME         "Logs/latest"
#define DJI_LOG_FOLDER_NAME             "Logs"
#define DJI_LOG_PATH_MAX_SIZE           (128)
#define DJI_LOG_FOLDER_NAME_MAX_SIZE    (32)
#define DJI_SYSTEM_CMD_STR_MAX_SIZE     (64)
#define DJI_LOG_MAX_COUNT               (10)

#define USER_UTIL_UNUSED(x)                                 ((x) = (x))
#define USER_UTIL_MIN(a, b)                                 (((a) < (b)) ? (a) : (b))
#define USER_UTIL_MAX(a, b)                                 (((a) > (b)) ? (a) : (b))

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
static FILE *s_djiLogFile;
static FILE *s_djiLogFileCnt;

/* Private functions declaration ---------------------------------------------*/
static void DjiUser_NormalExitHandler(int signalNum);

static T_DjiTaskHandle s_userSendVideoThread;

/* Exported functions definition ---------------------------------------------*/
Application::Application(int argc, char **argv)
{
    Application::DjiUser_SetupEnvironment();
    Application::DjiUser_ApplicationStart();

    Osal_TaskSleepMs(3000);
}

Application::~Application()
= default;

//_Noreturn static void *UserCameraMedia_SendVideoTask(void *arg);
//static void *UserCameraMedia_SendVideoTask(void *arg)
//{
//    int ret;
//    T_DjiReturnCode returnCode;
//    static uint32_t sendVideoStep = 0;
//    FILE *fpFile = NULL;
//    unsigned long dataLength = 0;
//    uint16_t lengthOfDataToBeSent = 0;
//    int lengthOfDataHaveBeenSent = 0;
//    char *dataBuffer = NULL;
//    T_TestPayloadCameraPlaybackCommand playbackCommand = {0};
//    uint16_t bufferReadSize = 0;
//    char *videoFilePath = NULL;
//    char *transcodedFilePath = NULL;
//    float frameRate = 1.0f;
//    T_TestPayloadCameraVideoFrameInfo *frameInfo = NULL;
//    uint32_t frameNumber = 0;
//    uint32_t frameCount = 0;
//    uint32_t startTimeMs = 0;
//    bool sendVideoFlag = true;
//    bool sendOneTimeFlag = false;
//    T_DjiDataChannelState videoStreamState = {0};
//    E_DjiCameraMode mode = DJI_CAMERA_MODE_SHOOT_PHOTO;
//    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
//    uint32_t frameBufSize = 0;
//    E_DjiCameraVideoStreamType videoStreamType;
//    char curFileDirPath[DJI_FILE_PATH_SIZE_MAX];
//    char tempPath[DJI_FILE_PATH_SIZE_MAX];
//
//    USER_UTIL_UNUSED(arg);
//
//    returnCode = DjiUserUtil_GetCurrentFileDirPath(__FILE__, DJI_FILE_PATH_SIZE_MAX, curFileDirPath);
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", returnCode);
//        exit(1);
//    }
//    if (s_isMediaFileDirPathConfigured == true) {
//        snprintf(tempPath, DJI_FILE_PATH_SIZE_MAX, "%sPSDK_0005.h264", s_mediaFileDirPath);
//    } else {
//        snprintf(tempPath, DJI_FILE_PATH_SIZE_MAX, "%smedia_file/PSDK_0005.h264", curFileDirPath);
//    }
//
//    videoFilePath = osalHandler->Malloc(DJI_FILE_PATH_SIZE_MAX);
//    if (videoFilePath == NULL) {
//        USER_LOG_ERROR("malloc memory for video file path fail.");
//        exit(1);
//    }
//
//    transcodedFilePath = osalHandler->Malloc(DJI_FILE_PATH_SIZE_MAX);
//    if (transcodedFilePath == NULL) {
//        USER_LOG_ERROR("malloc memory for transcoded file path fail.");
//        exit(1);
//    }
//
//    frameInfo = osalHandler->Malloc(VIDEO_FRAME_MAX_COUNT * sizeof(T_TestPayloadCameraVideoFrameInfo));
//    if (frameInfo == NULL) {
//        USER_LOG_ERROR("malloc memory for frame info fail.");
//        exit(1);
//    }
//    memset(frameInfo, 0, VIDEO_FRAME_MAX_COUNT * sizeof(T_TestPayloadCameraVideoFrameInfo));
//
//    returnCode = DjiPlayback_StopPlayProcess();
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("stop playback and start liveview error: 0x%08llX.", returnCode);
//        exit(1);
//    }
//
//    while (1) {
//        osalHandler->TaskSleepMs(1000 / SEND_VIDEO_TASK_FREQ);
//
//        // response playback command
//        if (osalHandler->MutexLock(s_mediaPlayCommandBufferMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("mutex lock error");
//            continue;
//        }
//
//        bufferReadSize = UtilBuffer_Get(&s_mediaPlayCommandBufferHandler, (uint8_t *) &playbackCommand,
//                                        sizeof(T_TestPayloadCameraPlaybackCommand));
//
//        if (osalHandler->MutexUnlock(s_mediaPlayCommandBufferMutex) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("mutex unlock error");
//            continue;
//        }
//
//        if (bufferReadSize != sizeof(T_TestPayloadCameraPlaybackCommand))
//            goto send;
//
//        switch (playbackCommand.command) {
//            case TEST_PAYLOAD_CAMERA_MEDIA_PLAY_COMMAND_STOP:
//                snprintf(videoFilePath, DJI_FILE_PATH_SIZE_MAX, "%s", tempPath);
//                startTimeMs = 0;
//                sendVideoFlag = true;
//                sendOneTimeFlag = false;
//                break;
//            case TEST_PAYLOAD_CAMERA_MEDIA_PLAY_COMMAND_PAUSE:
//                sendVideoFlag = false;
//                goto send;
//            case TEST_PAYLOAD_CAMERA_MEDIA_PLAY_COMMAND_START:
//                snprintf(videoFilePath, DJI_FILE_PATH_SIZE_MAX, "%s", playbackCommand.path);
//                startTimeMs = playbackCommand.timeMs;
//                sendVideoFlag = true;
//                sendOneTimeFlag = true;
//                break;
//            default:
//                USER_LOG_ERROR("playback command invalid: %d.", playbackCommand.command);
//                sendVideoFlag = false;
//                goto send;
//        }
//
//        // video send preprocess
//        returnCode = DjiPlayback_VideoFileTranscode(videoFilePath, "h264", transcodedFilePath,
//                                                    DJI_FILE_PATH_SIZE_MAX);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("transcode video file error: 0x%08llX.", returnCode);
//            continue;
//        }
//
//        returnCode = DjiPlayback_GetFrameRateOfVideoFile(transcodedFilePath, &frameRate);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("get frame rate of video error: 0x%08llX.", returnCode);
//            continue;
//        }
//
//        returnCode = DjiPlayback_GetFrameInfoOfVideoFile(transcodedFilePath, frameInfo, VIDEO_FRAME_MAX_COUNT,
//                                                         &frameCount);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("get frame info of video error: 0x%08llX.", returnCode);
//            continue;
//        }
//
//        returnCode = DjiPlayback_GetFrameNumberByTime(frameInfo, frameCount, &frameNumber,
//                                                      startTimeMs);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("get start frame number error: 0x%08llX.", returnCode);
//            continue;
//        }
//
//        if (fpFile != NULL)
//            fclose(fpFile);
//
//        fpFile = fopen(transcodedFilePath, "rb+");
//        if (fpFile == NULL) {
//            USER_LOG_ERROR("open video file fail.");
//            continue;
//        }
//
//send:
//        if (fpFile == NULL) {
//            USER_LOG_ERROR("open video file fail.");
//            continue;
//        }
//
//        if (sendVideoFlag != true)
//            continue;
//
//        returnCode = DjiTest_CameraGetMode(&mode);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            continue;
//        }
//
//        returnCode = DjiTest_CameraGetVideoStreamType(&videoStreamType);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            continue;
//        }
//
//        if (mode == DJI_CAMERA_MODE_PLAYBACK && s_playbackInfo.isInPlayProcess == false) {
//            continue;
//        }
//
//        if (!USER_UTIL_IS_WORK_TURN(sendVideoStep++, frameRate, SEND_VIDEO_TASK_FREQ))
//            continue;
//
//        frameBufSize = frameInfo[frameNumber].size;
//        if (videoStreamType == DJI_CAMERA_VIDEO_STREAM_TYPE_H264_DJI_FORMAT) {
//            frameBufSize = frameBufSize + VIDEO_FRAME_AUD_LEN;
//        }
//
//        dataBuffer = calloc(frameBufSize, 1);
//        if (dataBuffer == NULL) {
//            USER_LOG_ERROR("malloc fail.");
//            goto free;
//        }
//
//        ret = fseek(fpFile, frameInfo[frameNumber].positionInFile, SEEK_SET);
//        if (ret != 0) {
//            USER_LOG_ERROR("fseek fail.");
//            goto free;
//        }
//
//        dataLength = fread(dataBuffer, 1, frameInfo[frameNumber].size, fpFile);
//        if (dataLength != frameInfo[frameNumber].size) {
//            USER_LOG_ERROR("read data from video file error.");
//        } else {
//            USER_LOG_DEBUG("read data from video file success, len = %d B\r\n", dataLength);
//        }
//
//        if (videoStreamType == DJI_CAMERA_VIDEO_STREAM_TYPE_H264_DJI_FORMAT) {
//            memcpy(&dataBuffer[frameInfo[frameNumber].size], s_frameAudInfo, VIDEO_FRAME_AUD_LEN);
//            dataLength = dataLength + VIDEO_FRAME_AUD_LEN;
//        }
//
//        lengthOfDataHaveBeenSent = 0;
//        while (dataLength - lengthOfDataHaveBeenSent) {
//            lengthOfDataToBeSent = USER_UTIL_MIN(DATA_SEND_FROM_VIDEO_STREAM_MAX_LEN,
//                                                 dataLength - lengthOfDataHaveBeenSent);
//            returnCode = DjiPayloadCamera_SendVideoStream((const uint8_t *) dataBuffer + lengthOfDataHaveBeenSent,
//                                                          lengthOfDataToBeSent);
//            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                USER_LOG_ERROR("send video stream error: 0x%08llX.", returnCode);
//            }
//            lengthOfDataHaveBeenSent += lengthOfDataToBeSent;
//        }
//
//        if ((frameNumber++) >= frameCount) {
//            USER_LOG_DEBUG("reach file tail.");
//            frameNumber = 0;
//
//            if (sendOneTimeFlag == true)
//                sendVideoFlag = false;
//        }
//
//        returnCode = DjiPayloadCamera_GetVideoStreamState(&videoStreamState);
//        if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_DEBUG(
//                "video stream state: realtimeBandwidthLimit: %d, realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController:%d busyState: %d.",
//                videoStreamState.realtimeBandwidthLimit, videoStreamState.realtimeBandwidthBeforeFlowController,
//                videoStreamState.realtimeBandwidthAfterFlowController,
//                videoStreamState.busyState);
//        } else {
//            USER_LOG_ERROR("get video stream state error.");
//        }
//
//free:
//        free(dataBuffer);
//    }
//}

/* Private functions definition-----------------------------------------------*/
void Application::DjiUser_SetupEnvironment()
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler osalHandler = {0};
    T_DjiHalUartHandler uartHandler = {0};
    T_DjiHalUsbBulkHandler usbBulkHandler = {0};
    T_DjiLoggerConsole printConsole;
    T_DjiLoggerConsole localRecordConsole;
    T_DjiFileSystemHandler fileSystemHandler = {0};
    T_DjiSocketHandler socketHandler {0};
    T_DjiHalNetworkHandler networkHandler = {0};

    networkHandler.NetworkInit = HalNetWork_Init;
    networkHandler.NetworkDeInit = HalNetWork_DeInit;
    networkHandler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;

    socketHandler.Socket = Osal_Socket;
    socketHandler.Bind = Osal_Bind;
    socketHandler.Close = Osal_Close;
    socketHandler.UdpSendData = Osal_UdpSendData;
    socketHandler.UdpRecvData = Osal_UdpRecvData;
    socketHandler.TcpListen = Osal_TcpListen;
    socketHandler.TcpAccept = Osal_TcpAccept;
    socketHandler.TcpConnect = Osal_TcpConnect;
    socketHandler.TcpSendData = Osal_TcpSendData;
    socketHandler.TcpRecvData = Osal_TcpRecvData;

    osalHandler.TaskCreate = Osal_TaskCreate;
    osalHandler.TaskDestroy = Osal_TaskDestroy;
    osalHandler.TaskSleepMs = Osal_TaskSleepMs;
    osalHandler.MutexCreate = Osal_MutexCreate;
    osalHandler.MutexDestroy = Osal_MutexDestroy;
    osalHandler.MutexLock = Osal_MutexLock;
    osalHandler.MutexUnlock = Osal_MutexUnlock;
    osalHandler.SemaphoreCreate = Osal_SemaphoreCreate;
    osalHandler.SemaphoreDestroy = Osal_SemaphoreDestroy;
    osalHandler.SemaphoreWait = Osal_SemaphoreWait;
    osalHandler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
    osalHandler.SemaphorePost = Osal_SemaphorePost;
    osalHandler.Malloc = Osal_Malloc;
    osalHandler.Free = Osal_Free;
    osalHandler.GetTimeMs = Osal_GetTimeMs;
    osalHandler.GetTimeUs = Osal_GetTimeUs;
    osalHandler.GetRandomNum = Osal_GetRandomNum;

    printConsole.func = DjiUser_PrintConsole;
    printConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO;
    printConsole.isSupportColor = true;

    localRecordConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG;
    localRecordConsole.func = DjiUser_LocalWrite;
    localRecordConsole.isSupportColor = false;

    uartHandler.UartInit = HalUart_Init;
    uartHandler.UartDeInit = HalUart_DeInit;
    uartHandler.UartWriteData = HalUart_WriteData;
    uartHandler.UartReadData = HalUart_ReadData;
    uartHandler.UartGetStatus = HalUart_GetStatus;

    usbBulkHandler.UsbBulkInit = HalUsbBulk_Init;
    usbBulkHandler.UsbBulkDeInit = HalUsbBulk_DeInit;
    usbBulkHandler.UsbBulkWriteData = HalUsbBulk_WriteData;
    usbBulkHandler.UsbBulkReadData = HalUsbBulk_ReadData;
    usbBulkHandler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

    fileSystemHandler.FileOpen = Osal_FileOpen,
    fileSystemHandler.FileClose = Osal_FileClose,
    fileSystemHandler.FileWrite = Osal_FileWrite,
    fileSystemHandler.FileRead = Osal_FileRead,
    fileSystemHandler.FileSync = Osal_FileSync,
    fileSystemHandler.FileSeek = Osal_FileSeek,
    fileSystemHandler.DirOpen = Osal_DirOpen,
    fileSystemHandler.DirClose = Osal_DirClose,
    fileSystemHandler.DirRead = Osal_DirRead,
    fileSystemHandler.Mkdir = Osal_Mkdir,
    fileSystemHandler.Unlink = Osal_Unlink,
    fileSystemHandler.Rename = Osal_Rename,
    fileSystemHandler.Stat = Osal_Stat,

    returnCode = DjiPlatform_RegOsalHandler(&osalHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register osal handler error.");
    }

    returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal uart handler error.");
    }

#if (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
    returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal usb bulk handler error.");
    }
#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
    returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal network handler error");
    }

//    USER_LOG_INFO("Creating video task...");
//    if (DjiPlatform_GetHalNetworkHandler() != NULL || DjiPlatform_GetHalUsbBulkHandler() != NULL) {
//        returnCode = osalHandler.TaskCreate("user_camera_media_task", s_userSendVideoThread, 2048,
//                                             NULL, &s_userSendVideoThread);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("user send video task create error.");
//            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
//        }
//	USER_LOG_INFO("Created video task.");
//    }

#elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
    /*!< Attention: Only use uart hardware connection.
     */
#endif

    //Attention: if you want to use camera stream view function, please uncomment it.
    returnCode = DjiPlatform_RegSocketHandler(&socketHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("register osal socket handler error");
    }

    returnCode = DjiPlatform_RegFileSystemHandler(&fileSystemHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register osal filesystem handler error.");
    }

    if (DjiUser_LocalWriteFsInit(DJI_LOG_PATH) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("File system init error.");
    }

    returnCode = DjiLogger_AddConsole(&printConsole);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Add printf console error.");
    }

    returnCode = DjiLogger_AddConsole(&localRecordConsole);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Add printf console error.");
    }
}

void Application::DjiUser_ApplicationStart()
{
    T_DjiUserInfo userInfo;
    T_DjiReturnCode returnCode;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    T_DjiFirmwareVersion firmwareVersion = {
        .majorVersion = 1,
        .minorVersion = 0,
        .modifyVersion = 0,
        .debugVersion = 0,
    };

    // attention: when the program is hand up ctrl-c will generate the coredump file
    signal(SIGTERM, DjiUser_NormalExitHandler);

    returnCode = DjiUser_FillInUserInfo(&userInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Fill user info error, please check user info config.");
    }

    returnCode = DjiCore_Init(&userInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Core init error.");
    }

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Get aircraft base info error.");
    }

    if (aircraftInfoBaseInfo.mountPosition != DJI_MOUNT_POSITION_EXTENSION_PORT) {
        throw std::runtime_error("Please run this sample on extension port.");
    }

    returnCode = DjiCore_SetAlias("General Computational Payload");
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set alias error.");
    }

    returnCode = DjiCore_SetFirmwareVersion(firmwareVersion);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set firmware version error.");
    }

    returnCode = DjiCore_SetSerialNumber("PSDK12345678XX");
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set serial number error");
    }

    returnCode = DjiCore_ApplicationStart();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Start sdk application error.");
    }

    USER_LOG_INFO("Application start.");
}

T_DjiReturnCode Application::DjiUser_PrintConsole(const uint8_t *data, uint16_t dataLen)
{
    printf("%s", data);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Application::DjiUser_LocalWrite(const uint8_t *data, uint16_t dataLen)
{
    int32_t realLen;

    if (s_djiLogFile == nullptr) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    realLen = fwrite(data, 1, dataLen, s_djiLogFile);
    fflush(s_djiLogFile);
    if (realLen == dataLen) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
}

T_DjiReturnCode Application::DjiUser_FillInUserInfo(T_DjiUserInfo *userInfo)
{
    memset(userInfo->appName, 0, sizeof(userInfo->appName));
    memset(userInfo->appId, 0, sizeof(userInfo->appId));
    memset(userInfo->appKey, 0, sizeof(userInfo->appKey));
    memset(userInfo->appLicense, 0, sizeof(userInfo->appLicense));
    memset(userInfo->developerAccount, 0, sizeof(userInfo->developerAccount));
    memset(userInfo->baudRate, 0, sizeof(userInfo->baudRate));

    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) > sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) > sizeof(userInfo->appKey) ||
        strlen(USER_APP_LICENSE) > sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount) ||
        strlen(USER_BAUD_RATE) > sizeof(userInfo->baudRate)) {
        USER_LOG_ERROR("Length of user information string is beyond limit. Please check.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    if (!strcmp(USER_APP_NAME, "your_app_name") ||
        !strcmp(USER_APP_ID, "your_app_id") ||
        !strcmp(USER_APP_KEY, "your_app_key") ||
        !strcmp(USER_BAUD_RATE, "your_app_license") ||
        !strcmp(USER_DEVELOPER_ACCOUNT, "your_developer_account") ||
        !strcmp(USER_BAUD_RATE, "your_baud_rate")) {
        USER_LOG_ERROR(
            "Please fill in correct user information to 'samples/sample_c++/platform/linux/manifold2/application/dji_sdk_app_info.h' file.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, USER_UTIL_MIN(sizeof(userInfo->appId), strlen(USER_APP_ID)));
    memcpy(userInfo->appKey, USER_APP_KEY, USER_UTIL_MIN(sizeof(userInfo->appKey), strlen(USER_APP_KEY)));
    memcpy(userInfo->appLicense, USER_APP_LICENSE,
           USER_UTIL_MIN(sizeof(userInfo->appLicense), strlen(USER_APP_LICENSE)));
    memcpy(userInfo->baudRate, USER_BAUD_RATE, USER_UTIL_MIN(sizeof(userInfo->baudRate), strlen(USER_BAUD_RATE)));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Application::DjiUser_LocalWriteFsInit(const char *path)
{
    T_DjiReturnCode djiReturnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    char filePath[DJI_LOG_PATH_MAX_SIZE];
    char systemCmd[DJI_SYSTEM_CMD_STR_MAX_SIZE];
    char folderName[DJI_LOG_FOLDER_NAME_MAX_SIZE];
    time_t currentTime = time(nullptr);
    struct tm *localTime = localtime(&currentTime);
    uint16_t logFileIndex = 0;
    uint16_t currentLogFileIndex;
    uint8_t ret;

    if (localTime == nullptr) {
        printf("Get local time error.\r\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (access(DJI_LOG_FOLDER_NAME, F_OK) != 0) {
        sprintf(folderName, "mkdir %s", DJI_LOG_FOLDER_NAME);
        ret = system(folderName);
        if (ret != 0) {
            printf("Create new log folder error, ret:%d.\r\n", ret);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    s_djiLogFileCnt = fopen(DJI_LOG_INDEX_FILE_NAME, "rb+");
    if (s_djiLogFileCnt == nullptr) {
        s_djiLogFileCnt = fopen(DJI_LOG_INDEX_FILE_NAME, "wb+");
        if (s_djiLogFileCnt == nullptr) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    } else {
        ret = fseek(s_djiLogFileCnt, 0, SEEK_SET);
        if (ret != 0) {
            printf("Seek log count file error, ret: %d, errno: %d.\r\n", ret, errno);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }

        ret = fread((uint16_t *) &logFileIndex, 1, sizeof(uint16_t), s_djiLogFileCnt);
        if (ret != sizeof(uint16_t)) {
            printf("Read log file index error.\r\n");
        }
    }

    currentLogFileIndex = logFileIndex;
    logFileIndex++;

    ret = fseek(s_djiLogFileCnt, 0, SEEK_SET);
    if (ret != 0) {
        printf("Seek log file error, ret: %d, errno: %d.\r\n", ret, errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    ret = fwrite((uint16_t *) &logFileIndex, 1, sizeof(uint16_t), s_djiLogFileCnt);
    if (ret != sizeof(uint16_t)) {
        printf("Write log file index error.\r\n");
        fclose(s_djiLogFileCnt);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    fclose(s_djiLogFileCnt);

    sprintf(filePath, "%s_%04d_%04d%02d%02d_%02d-%02d-%02d.log", path, currentLogFileIndex,
            localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
            localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

    s_djiLogFile = fopen(filePath, "wb+");
    if (s_djiLogFile == nullptr) {
        USER_LOG_ERROR("Open filepath time error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (logFileIndex >= DJI_LOG_MAX_COUNT) {
        sprintf(systemCmd, "rm -rf %s_%04d*.log", path, currentLogFileIndex - DJI_LOG_MAX_COUNT);
        ret = system(systemCmd);
        if (ret != 0) {
            printf("Remove file error, ret:%d.\r\n", ret);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    return djiReturnCode;
}

static void DjiUser_NormalExitHandler(int signalNum)
{
    USER_UTIL_UNUSED(signalNum);
    exit(0);
}

//T_DjiReturnCode HalNetWork_Init(const char *ipAddr, const char *netMask, T_DjiNetworkHandle *halObj)
//{
//	int32_t ret;
//	char cmdStr[LINUX_CMD_STR_MAX_SIZE];
//
//	if (ipAddr == NULL || netMask == NULL)
//	{
//		USER_LOG_ERROR("hal network config param error");
//		return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
//	}
//
//	//Attention: need root permission to config ip addr and netmask.
//	memset(cmdStr, 0, sizeof(cmdStr));
//
//	snprintf(cmdStr, sizeof(cmdStr), "ifconfig %s up", LINUX_NETWORK_DEV);
//	ret = system(cmdStr);
//	if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//	{
//		USER_LOG_ERROR("Can't open the network."
//		"Probably the program not execute with root permission."
//		"Please use the root permission to execute the program.");
//		return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
//	}
//
//	snprintf(cmdStr, sizeof(cmdStr), "ifconfig %s %s netmask %s", LINUX_NETWORK_DEV, ipAddr, netMask);
//	ret = system(cmdStr);
//	if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
//	{
//		USER_LOG_ERROR("Can't config the ip address of network."
//		"Probably the program not execute with root permission."
//		"Please use the root permission to execute the program.");
//		return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
//	}
//
//	return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
//}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
