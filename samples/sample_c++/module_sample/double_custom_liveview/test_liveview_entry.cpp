/**
 ********************************************************************
 * @file    test_liveview_entry.cpp
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
#include <iostream>
#include <dji_logger.h>
#include "test_liveview_entry.hpp"
#include "test_liveview.hpp"

#ifdef OPEN_CV_INSTALLED

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../../../sample_c/module_sample/utils/util_misc.h"

#include <chrono>

using namespace cv;
#endif
using namespace std;
using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
const char *classNames[] = {"background", "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
                            "boat", "traffic light",
                            "fire hydrant", "background", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                            "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "background", "backpack",
                            "umbrella", "background", "background", "handbag", "tie", "suitcase", "frisbee", "skis",
                            "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
                            "surfboard", "tennis racket",
                            "bottle", "background", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
                            "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
                            "cake", "chair", "couch", "potted plant", "bed", "background", "dining table", "background",
                            "background", "toilet", "background", "tv", "laptop", "mouse", "remote", "keyboard",
                            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "background", "book",
                            "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

const size_t inWidth = 320;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float) inHeight;
static int32_t s_demoIndex = -1;
char curFileDirPath[DJI_FILE_PATH_SIZE_MAX];
char tempFileDirPath[DJI_FILE_PATH_SIZE_MAX];
char prototxtFileDirPath[DJI_FILE_PATH_SIZE_MAX];
char weightsFileDirPath[DJI_FILE_PATH_SIZE_MAX];

/* Private functions declaration ---------------------------------------------*/
static void joshua_image_callback(CameraRGBImage img, void *userData);
static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData);
static T_DjiReturnCode DjiUser_GetCurrentFileDirPath(const char *filePath, uint32_t pathBufferSize, char *dirPath);

// ****************************************************************************
apriltag_family_t *tf = nullptr;
apriltag_detector_t *td = nullptr;
// ****************************************************************************

using namespace std;

/* Exported functions definition ---------------------------------------------*/
void DjiUser_RunFileStreamViewSample(const char * filename)
{
    cout << "Starting DjiUser_RunCameraStreamViewSample" << endl;

    char cameraIndexChar = 0;
    char demoIndexChar = 0;
    char isQuit = 0;
    CameraRGBImage camImg;
    char fpvName[] = "FPV_CAM";
    char mainName[] = "MAIN_CAM";
    char viceName[] = "VICE_CAM";
    char topName[] = "TOP_CAM";
    auto *liveviewSample = new LiveviewSample();

    s_demoIndex = 0;
    //liveviewSample->joshua_start_camera_stream_main(&joshua_image_callback, &mainName);
    liveviewSample->joshua_start_file_stream(&joshua_image_callback, &mainName);
    
    cout << "Please enter the 'q' or 'Q' to quit camera stream view\n"
         << endl;
    while (true) {
        cin >> isQuit;
        if (isQuit == 'q' || isQuit == 'Q') {
            break;
        }
    }

    delete liveviewSample;
}

void DjiUser_RunCameraStreamViewSample()
{
    cout << "Starting DjiUser_RunCameraStreamViewSample" << endl;

    char cameraIndexChar = 0;
    char demoIndexChar = 0;
    char isQuit = 0;
    CameraRGBImage camImg;
    char fpvName[] = "FPV_CAM";
    char mainName[] = "MAIN_CAM";
    char viceName[] = "VICE_CAM";
    char topName[] = "TOP_CAM";
    auto *liveviewSample = new LiveviewSample();

    s_demoIndex = 0;
    //liveviewSample->joshua_start_camera_stream_main(&joshua_image_callback, &mainName);
    liveviewSample->joshua_start_file_stream(&joshua_image_callback, &mainName);
    
    cout << "Please enter the 'q' or 'Q' to quit camera stream view\n"
         << endl;
    while (true) {
        cin >> isQuit;
        if (isQuit == 'q' || isQuit == 'Q') {
            break;
        }
    }

    delete liveviewSample;
}

static void joshua_image_callback(CameraRGBImage img, void *userData)
{
//    auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
//    cout << "Joshua received image( " << millisec_since_epoch << " ): width=" << img.width << ", height=" << img.height << endl;

    string name = string(reinterpret_cast<char *>(userData));

    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);

    if (s_demoIndex == 0)
    {
        cvtColor(mat, mat, COLOR_RGB2BGR);

	// ***************************************************************************
	if( nullptr == tf )
	{
	    tf = tagCustom48h12_create();
	    td = apriltag_detector_create();
	    apriltag_detector_add_family(td, tf);

	    td->quad_decimate = 2;
	    td->quad_sigma    = 0;
	    td->nthreads      = 3;
	    td->debug         = false;
	    td->refine_edges  = false;
	}
	cv::Mat gray;
	cvtColor(mat, gray, COLOR_BGR2GRAY);
	image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
	zarray_t *detections = apriltag_detector_detect(td, &im);

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
        apriltag_detections_destroy(detections);	
	// ***************************************************************************

        imshow(name, mat);
    }
    else if (s_demoIndex == 1)
    {
        cvtColor(mat, mat, COLOR_RGB2GRAY);
	imshow(name, mat);
//        Mat mask;
//        cv::threshold(mat, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
//        imshow(name, mask);
    }

    cv::waitKey(1);
}

/* Private functions definition-----------------------------------------------*/
static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData)
{
    auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    cout << "Received image( " << millisec_since_epoch << " ): width=" << img.width << ", height=" << img.height << endl;

    string name = string(reinterpret_cast<char *>(userData));

#ifdef OPEN_CV_INSTALLED
    Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);

    if (s_demoIndex == 0) {
        cvtColor(mat, mat, COLOR_RGB2BGR);
        imshow(name, mat);
    } else if (s_demoIndex == 1) {
        cvtColor(mat, mat, COLOR_RGB2GRAY);
        Mat mask;
        cv::threshold(mat, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        imshow(name, mask);
    } else if (s_demoIndex == 2) {
        cvtColor(mat, mat, COLOR_RGB2BGR);
        snprintf(tempFileDirPath, DJI_FILE_PATH_SIZE_MAX, "%s/data/haarcascade_frontalface_alt.xml", curFileDirPath);
        auto faceDetector = cv::CascadeClassifier(tempFileDirPath);
        std::vector<Rect> faces;
        faceDetector.detectMultiScale(mat, faces, 1.1, 3, 0, Size(50, 50));

        for (int i = 0; i < faces.size(); ++i) {
            cout << "index: " << i;
            cout << "  x: " << faces[i].x;
            cout << "  y: " << faces[i].y << endl;

#ifdef OPEN_CV_VERSION_3
            cv::rectangle(mat, cvPoint(faces[i].x, faces[i].y),
                          cvPoint(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                          Scalar(0, 0, 255), 2, 1, 0);
#endif

#ifdef OPEN_CV_VERSION_4
            cv::rectangle(mat, cv::Point(faces[i].x, faces[i].y),
                          cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
                          Scalar(0, 0, 255), 2, 1, 0);
#endif
        }
        imshow(name, mat);
    } else if (s_demoIndex == 3) {
        snprintf(prototxtFileDirPath, DJI_FILE_PATH_SIZE_MAX,
                 "%s/data/tensorflow/ssd_inception_v2_coco_2017_11_17.pbtxt",
                 curFileDirPath);
        //Attention: If you want to run the Tensorflow Object detection demo, Please download the tensorflow model.
        //Download Url: http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_17.tar.gz
        snprintf(weightsFileDirPath, DJI_FILE_PATH_SIZE_MAX, "%s/data/tensorflow/frozen_inference_graph.pb",
                 curFileDirPath);

        dnn::Net net = cv::dnn::readNetFromTensorflow(weightsFileDirPath, prototxtFileDirPath);
        Size frame_size = mat.size();

        Size cropSize;
        if (frame_size.width / (float) frame_size.height > WHRatio) {
            cropSize = Size(static_cast<int>(frame_size.height * WHRatio),
                            frame_size.height);
        } else {
            cropSize = Size(frame_size.width,
                            static_cast<int>(frame_size.width / WHRatio));
        }

        Rect crop(Point((frame_size.width - cropSize.width) / 2,
                        (frame_size.height - cropSize.height) / 2),
                  cropSize);

        cv::Mat blob = cv::dnn::blobFromImage(mat, 1, Size(300, 300));
        net.setInput(blob);
        Mat output = net.forward();
        Mat detectionMat(output.size[2], output.size[3], CV_32F, output.ptr<float>());

        mat = mat(crop);
        float confidenceThreshold = 0.50;

        for (int i = 0; i < detectionMat.rows; i++) {
            float confidence = detectionMat.at<float>(i, 2);
            if (confidence > confidenceThreshold) {
                auto objectClass = (size_t) (detectionMat.at<float>(i, 1));

                int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * mat.cols);
                int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * mat.rows);
                int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * mat.cols);
                int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * mat.rows);

                ostringstream ss;
                ss << confidence;
                String conf(ss.str());

                Rect object((int) xLeftBottom, (int) yLeftBottom,
                            (int) (xRightTop - xLeftBottom),
                            (int) (yRightTop - yLeftBottom));

                rectangle(mat, object, Scalar(0, 255, 0), 2);
                String label = String(classNames[objectClass]) + ": " + conf;

                int baseLine = 0;
                Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
#ifdef OPEN_CV_VERSION_3
                rectangle(mat, Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),
                                    Size(labelSize.width, labelSize.height + baseLine)), Scalar(0, 255, 0), CV_FILLED);
#endif

#ifdef OPEN_CV_VERSION_4
                rectangle(mat, Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),
                                    Size(labelSize.width, labelSize.height + baseLine)), Scalar(0, 255, 0), cv::FILLED);
#endif
                putText(mat, label, Point(xLeftBottom, yLeftBottom), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
            }
        }
        imshow(name, mat);
    }

    cv::waitKey(1);
#endif
}

static T_DjiReturnCode DjiUser_GetCurrentFileDirPath(const char *filePath, uint32_t pathBufferSize, char *dirPath)
{
    uint32_t i = strlen(filePath) - 1;
    uint32_t dirPathLen;

    while (filePath[i] != '/') {
        i--;
    }

    dirPathLen = i + 1;

    if (dirPathLen + 1 > pathBufferSize) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    memcpy(dirPath, filePath, dirPathLen);
    dirPath[dirPathLen] = 0;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
