/**
 ********************************************************************
 * @file    dji_camera_stream_decoder.cpp
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
#include "dji_camera_stream_decoder.hpp"
#include "unistd.h"
#include "pthread.h"
#include "dji_logger.h"

#include <libavutil/avutil.h>
#include <iostream>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
DJICameraStreamDecoder::DJICameraStreamDecoder()
    : initSuccess(false),
      cbThreadIsRunning(false),
      cbThreadStatus(-1),
      cb(nullptr),
      cbUserParam(nullptr),
#ifdef FFMPEG_INSTALLED
      pCodecCtx(nullptr),
      pCodec(nullptr),
      pCodecParserCtx(nullptr),
      pSwsCtx(nullptr),
      pFrameYUV(nullptr),
      pFrameRGB(nullptr),
      rgbBuf(nullptr),
#endif
      bufSize(0)
{
    pthread_mutex_init(&decodemutex, nullptr);
}

DJICameraStreamDecoder::~DJICameraStreamDecoder()
{
    pthread_mutex_destroy(&decodemutex);
    if(cb)
    {
        registerCallback(nullptr, nullptr);
    }

    cleanup();
}

bool DJICameraStreamDecoder::init()
{

    std::cout << "In stream decoder init!" << std::endl;

    pthread_mutex_lock(&decodemutex);

    if (true == initSuccess) {
        USER_LOG_INFO("Decoder already initialized.\n");
        return true;
    }

#ifdef FFMPEG_INSTALLED
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(nullptr);
    if (!pCodecCtx) {
        return false;
    }

    // *************************************************************************
    // joshua
//    pCodecCtx->time_base.num = 1;
//    pCodecCtx->time_base.den = 15;
//    pCodecCtx->time_base.den = 30;
//    pCodecCtx->time_base.den = 60;
//    pCodecCtx->width         = 1920;
//    pCodecCtx->height        = 1080;
//    pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    // Set the time per frame for the codec context

//    AVRational timebase;
//    timebase.num = 1;
//    timebase.den = 30;  // Replace with your actual frame rate (e.g., 30 frames per second)
//    if (av_opt_set_q(pCodecCtx->priv_data, "time_base", timebase, 0) < 0) {
//        // Handle the error here
//	    std::cout << "got the error unfortunately" << std::endl;
//    }
    // *************************************************************************

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    //pCodec = avcodec_find_encoder_by_name("h264_v4l2m2m"); // joshua
    //pCodec = avcodec_find_decoder_by_name("h264"); // joshua
    //pCodec = avcodec_find_decoder_by_name("h264_v4l2m2m"); // joshua
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, nullptr) < 0) {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx) {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV) {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
        return false;
    }

    pSwsCtx = nullptr;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
#endif
    initSuccess = true;
    pthread_mutex_unlock(&decodemutex);

    

    std::cout << "at end of stream decoder init!" << std::endl;
    return true;
}

void DJICameraStreamDecoder::cleanup()
{
    pthread_mutex_lock(&decodemutex);

    initSuccess = false;

#ifdef FFMPEG_INSTALLED
    if (nullptr != pSwsCtx) {
        sws_freeContext(pSwsCtx);
        pSwsCtx = nullptr;
    }

    if (nullptr != pFrameYUV) {
        av_free(pFrameYUV);
        pFrameYUV = nullptr;
    }

    if (nullptr != pCodecParserCtx) {
        av_parser_close(pCodecParserCtx);
        pCodecParserCtx = nullptr;
    }

    if (nullptr != pCodec) {
        avcodec_close(pCodecCtx);
        pCodec = nullptr;
    }

    if (nullptr != pCodecCtx) {
        av_free(pCodecCtx);
        pCodecCtx = nullptr;
    }

    if (nullptr != rgbBuf) {
        av_free(rgbBuf);
        rgbBuf = nullptr;
    }

    if (nullptr != pFrameRGB) {
        av_free(pFrameRGB);
        pFrameRGB = nullptr;
    }
#endif
    pthread_mutex_unlock(&decodemutex);
}

void *DJICameraStreamDecoder::callbackThreadEntry(void *p)
{
    //DSTATUS_PRIVATE("****** Decoder Callback Thread Start ******\n");
    usleep(50 * 1000);
    static_cast<DJICameraStreamDecoder *>(p)->callbackThreadFunc();
    return nullptr;
}

void DJICameraStreamDecoder::callbackThreadFunc()
{
    while (cbThreadIsRunning) {
        CameraRGBImage copyOfImage;
        if (!decodedImageHandler.getNewImageWithLock(copyOfImage, 1000)) {
            //DDEBUG_PRIVATE("Decoder Callback Thread: Get image time out\n");
            continue;
        }

        if (cb) {
            (*cb)(copyOfImage, cbUserParam);
        }
    }
}

void DJICameraStreamDecoder::decodeBuffer(const uint8_t *buf, int bufLen)
{
    const uint8_t *pData = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

#ifdef FFMPEG_INSTALLED
    AVPacket pkt;
    av_init_packet(&pkt);
    pthread_mutex_lock(&decodemutex);
    while (remainingLen > 0) {
        if (!pCodecParserCtx || !pCodecCtx) {
            //DSTATUS("Invalid decoder ctx.");
            break;
        }
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData += processedLen;

        if (pkt.size > 0) {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture) {
                ////DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            } else {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                ////DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if (nullptr == pSwsCtx) {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, nullptr, nullptr, nullptr);
                }

                if (nullptr == rgbBuf) {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t *) av_malloc(bufSize);
                    avpicture_fill((AVPicture *) pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if (nullptr != pSwsCtx && nullptr != rgbBuf) {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;

                    decodedImageHandler.writeNewImageWithLock(pFrameRGB->data[0], bufSize, w, h);
                }
            }
        }
    }
    pthread_mutex_unlock(&decodemutex);
    av_free_packet(&pkt);
#endif
}

bool DJICameraStreamDecoder::registerCallback(CameraImageCallback f, void *param)
{
    cb = f;
    cbUserParam = param;

    /* When users register a non-nullptr callback, we will start the callback thread. */
    if (nullptr != cb) {
        if (!cbThreadIsRunning) {
            cbThreadStatus = pthread_create(&callbackThread, nullptr, callbackThreadEntry, this);
            if (0 == cbThreadStatus) {
                //DSTATUS_PRIVATE("User callback thread created successfully!\n");
                cbThreadIsRunning = true;
                return true;
            } else {
                //DERROR_PRIVATE("User called thread creation failed!\n");
                cbThreadIsRunning = false;
                return false;
            }
        } else {
            //DERROR_PRIVATE("Callback thread already running!\n");
            return true;
        }
    } else {
        if (cbThreadStatus == 0) {
            cbThreadIsRunning = false;
            pthread_join(callbackThread, nullptr);
            cbThreadStatus = -1;
        }
        return true;
    }
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/