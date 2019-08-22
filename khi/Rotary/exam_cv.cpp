
#include <iostream>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/device/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "exam_cv.h"

#define PI 3.1415926

#define _ROI_Xmin 0.0f	
#define _ROI_Xmax 1.0f
#define _ROI_Ymin 0.5f
#define _ROI_Ymin_max 0.6f
#define _ROI_Ymax 0.9f

/* Yellow Lane Detection Threshold */
#define LY_lowH 20
#define LY_highH 50	//40	
#define LY_lowS 30	//20
#define LY_highS 255
#define LY_lowV 80
#define LY_highV 255

using namespace std;
using namespace cv;

typedef enum
{
    STRAIGHT,
    CURVELEFT,
    CURVERIGHT
}DrivingMode;

volatile int lane_Xmin, lane_Xmax,lane_Ymin, lane_Ymax;
volatile double current_Y_min = _ROI_Ymin;

IplImage* g_image = NULL;//p
IplImage* g_gray = NULL;//p
IplImage* g_binary = NULL;//p

int g_thresh = 100 ;//p
CvMemStorage* g_storage = NULL ;//p 


extern "C" {


/**
  * @brief  To load image file to the buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to load
             nw : width v of the destination buffer
             nh : height v of the destination buffer
  * @retval none
  */
void OpenCV_load_file(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    srcRGB = imread(file, CV_LOAD_IMAGE_COLOR); // rgb
    //cvtColor(srcRGB, srcRGB, CV_RGB2BGR);

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To convert format from BGR to RGB.
  * @param  inBuf: buffer pointer of BGR image
             w: width v of the buffers
             h : height v of the buffers
             outBuf : buffer pointer of RGB image
  * @retval none
  */
void OpenCV_Bgr2RgbConvert(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
{
    Mat srcRGB(h, w, CV_8UC3, inBuf);
    Mat dstRGB(h, w, CV_8UC3, outBuf);

    cvtColor(srcRGB, dstRGB, CV_BGR2RGB);
}

/**
  * @brief  Detect faces on loaded image and draw circles on the faces of the loaded image.
  * @param  file: pointer for load image file in local path
             outBuf: buffer pointer to draw circles on the detected faces
             nw : width v of the destination buffer
             nh : height v of the destination buffer
  * @retval none
  */
void OpenCV_face_detection(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    
    // Load Face cascade (.xml file)
    CascadeClassifier face_cascade;
    face_cascade.load( "haarcascade_frontalface_alt.xml" );
 
    // Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( srcRGB, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
    
    // Draw circles on the detected faces
    for( int i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( srcRGB, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
 
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To bind two images on destination buffer.
  * @param  file1: file path of first image to bind
             file2: file path of second image to bind
             outBuf : destination buffer pointer to bind
             nw : width v of the destination buffer
             nh : height v of the destination buffer
  * @retval none
  */
void OpenCV_binding_image(char* file1, char* file2, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file1, CV_LOAD_IMAGE_COLOR);
    Mat srcRGB2 = imread(file2, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cv::resize(srcRGB2, srcRGB2, cv::Size(srcRGB2.cols/1.5, srcRGB2.rows/1.5));
    cv::Point location = cv::Point(280, 220);
    for (int y = std::max(location.y, 0); y < srcRGB.rows; ++y)
    {
        int fY = y - location.y;
        if (fY >= srcRGB2.rows)
            break;
        
        for (int x = std::max(location.x, 0); x < srcRGB.cols; ++x)
        {
            int fX = x - location.x;
            if (fX >= srcRGB2.cols)
            break;
            
            double opacity = ((double)srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + 3]) / 255.;
            for (int c = 0; opacity > 0 && c < srcRGB.channels(); ++c)
            {
                unsigned char overlayPx = srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + c];
                unsigned char srcPx = srcRGB.data[y * srcRGB.step + x * srcRGB.channels() + c];
                srcRGB.data[y * srcRGB.step + srcRGB.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }
 
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  Apply canny edge algorithm and draw it on destination buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to apply canny edge
             nw : width v of destination buffer
             nh : height v of destination buffer
  * @retval none
  */
void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat srcGRAY;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cvtColor(srcRGB, srcGRAY, CV_BGR2GRAY);
     // 케니 알고리즘 적용
    cv::Mat contours;
    cv::Canny(srcGRAY, // 그레이레벨 영상
        contours, // 결과 외곽선
        125,  // 낮은 경계값
        350);  // 높은 경계값

    // 넌제로 화소로 외곽선을 표현하므로 흑백 값을 반전
    //cv::Mat contoursInv; // 반전 영상
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
    // 밝기 값이 128보다 작으면 255가 되도록 설정
 
    cvtColor(contours, contours, CV_GRAY2BGR);
    
    cv::resize(contours, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}


/**
  * @brief  Detect the hough and draw hough on destination buffer.
  * @param  srcBuf: source pointer to hough transform
             iw: width v of source buffer
             ih : height v of source buffer
             outBuf : destination pointer to hough transform
             nw : width v of destination buffer
             nh : height v of destination buffer
  * @retval none
  */
DriveLine OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int* mode)
{
    CvPoint ptv = {0,0};

    Scalar lineColor = cv::Scalar(255, 0, 0);
    Scalar yellow(131, 232, 252);

    DriveLine dLine;
    
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat compImage(ih, iw, CV_8UC3, srcBuf);

    CvSeq* contours=0;
    
    cvtColor(srcRGB, srcRGB, CV_BGR2GRAY);

    IplImage *iplImage = new IplImage(srcRGB);

    //임계값 이하:0, 임계값초과값:1 설정
    cvThreshold(iplImage, iplImage, g_thresh, 255, CV_THRESH_BINARY);

    //윤곽선 찾기
    cvFindContours
    (
            iplImage,                //입력영상
            g_storage,             //검출된 외곽선을 기록하기 위한 메모리 스토리지
            &contours,             //외곽선의 좌표들이 저장된 Sequence
            sizeof(CvContour),    
            CV_RETR_TREE           //어떤종류의 외곽선 찾을지, 어떻게 보여줄지에 대한정보
    );     

    cvZero(iplImage);

    if(contours) 
    {
            //외곽선을 찾은 정보(contour)를 이용하여 외곽선을 그림
            cvDrawContours
            (
                    iplImage,                //외곽선이 그려질 영상
                    contours,              //외곽선 트리의 루트노드
                    cvScalarAll(255),      //외부 외곽선의 색상
                    cvScalarAll(128),      //내부 외곽선의 색상
                    100                    //외곽선을 그릴때 이동할 깊이
            );                           
    
    }

    compImage = cvarrToMat(iplImage);
    
    cv::resize(compImage, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    return dLine;
}

CvPoint CalVanishPoint(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4)
{
	CvPoint ptv = {0,0};	
	
	ptv.x = 320 * (pt3.y-pt1.y) / (pt2.y-pt1.y+pt3.y-pt4.y);
	ptv.y = (pt2.y-pt1.y) * ptv.x / 320 + pt1.y;

	return ptv;
}


/**
  * @brief  Merge two source images of the same size into the output buffer.
  * @param  src1: pointer to parameter of rgb32 image buffer
             src2: pointer to parameter of bgr32 image buffer
             dstImage : pointer to parameter of rgb32 output buffer
             w : width of src and dstImage buffer
             h : height of src and dstImage buffer
  * @retval none
  */
void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dstImage, int w, int h)
{
    Mat src1AR32(h, w, CV_8UC4, src1);
    Mat src2AR32(h, w, CV_8UC4, src2);
    Mat dstAR32(h, w, CV_8UC4, dstImage);

    cvtColor(src2AR32, src2AR32, CV_BGRA2RGBA);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double opacity = ((double)(src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + 3])) / 255.;
            for (int c = 0; opacity > 0 && c < src1AR32.channels(); ++c) {
                unsigned char overlayPx = src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + c];
                unsigned char srcPx = src1AR32.data[y * src1AR32.step + x * src1AR32.channels() + c];
                src1AR32.data[y * src1AR32.step + src1AR32.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }

    memcpy(dstImage, src1AR32.data, w*h*4);
}

}

