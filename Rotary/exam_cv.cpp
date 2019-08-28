
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
#include <opencv2/legacy/compat.hpp>
#include <opencv2/nonfree/nonfree.hpp>


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
int fcount = 0;

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

const int DIM_VECTOR = 128;


float Rotary(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int* mode)
{
    CvPoint2D32f max;
    CvPoint2D32f min;
    float area;

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);   

    if (fcount == 0)
    {
    
        // SURF추출을 위한 입력 영상을 그레이스케일로 읽음
        IplImage* grayImage = new IplImage(srcRGB);

        // 결과에 키포인트를 표현하기 위해 컬러로도 읽음 

        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* imageKeypoints = 0;
        CvSeq* imageDescriptors = 0;
        CvSURFParams params = cvSURFParams(1000, 1);

        // 영상으로부터 SURF 특징 추출
        cvExtractSURF(grayImage, 0, &imageKeypoints, &imageDescriptors, storage, params);
        cout << "Image Descriptors: " << imageDescriptors->total << endl;

        float temp2;
        CvPoint temp1 ; 

        // 영상에 키포인트 표현,지름 불필요
        for (int i = 0; i < imageKeypoints->total; i++) 
        {
            int radius;
            CvPoint2D32f center;    

            CvSURFPoint* point = (CvSURFPoint*)cvGetSeqElem(imageKeypoints, i);

            center.x = cvRound(point->pt.x);
            center.y = cvRound(point->pt.y);

            radius = cvRound(point->size * 1.2 / 9.0 * 2.0);
            //cv::line(dstHSV, pt1(temp[0][0], temp[0][1]),pt2, lineColor, 1); 
            //temp1 = center; 
            //temp2 = radius ; 
            if (i == 0)
            {
                max.x = min.x = center.x;
                max.y = min.y = center.y;
            }

            if((max.x < center.x) && (max.y < center.y))
            {
                max.x = center.x;
                max.y = center.y;
            }
            else if((min.x > center.x) && (min.y > center.y))
            {
                min.x = center.x;
                min.y = center.y;
            } 

            //cv::circle(srcRGB, temp1, temp2, cvScalar(0,255,255), 1, 8, 0);
        }
        cv::rectangle(srcRGB, min, max, cvScalar(0,255,255), 1, 8, 0);

        float width = max.x - min.x;
        float height = max.y - min.y;
        area = width*height ; 

        printf("min x,y : %f, %f \n", min.x ,min.y);
        printf("max x,y : %f ,%f \n", max.x ,max.y);
        printf("area : %f \n", area);

        // 후처리 - 메모리 해제 등
        cvClearSeq(imageKeypoints);
        cvClearSeq(imageDescriptors);
        cvReleaseMemStorage(&storage);
    }

        cv::resize(srcRGB, dstRGB, cv::Size(nw,nh), 0, 0, CV_INTER_LINEAR);

        if (fcount < 20)
            fcount++;
        else
            fcount = 0;

        return area;//면적 반환 
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

