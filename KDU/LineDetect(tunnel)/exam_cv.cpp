
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
    CURVERIGHT,
	TUNNEL
}DrivingMode;

volatile int lane_Xmin, lane_Xmax,lane_Ymin, lane_Ymax;
volatile double current_Y_min = _ROI_Ymin;


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
     // \C4ɴ\CF \BE˰\ED\B8\AE\C1\F2 \C0\FB\BF\EB
    cv::Mat contours;
    cv::Canny(srcGRAY, // \B1׷\B9\C0̷\B9\BA\A7 \BF\B5\BB\F3
        contours, // \B0\E1\B0\FA \BFܰ\FB\BC\B1
        125,  // \B3\B7\C0\BA \B0\E6\B0谪
        350);  // \B3\F4\C0\BA \B0\E6\B0谪

    // \B3\CD\C1\A6\B7\CE ȭ\BCҷ\CE \BFܰ\FB\BC\B1\C0\BB ǥ\C7\F6\C7ϹǷ\CE \C8\E6\B9\E9 \B0\AA\C0\BB \B9\DD\C0\FC
    //cv::Mat contoursInv; // \B9\DD\C0\FC \BF\B5\BB\F3
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
    // \B9\E0\B1\E2 \B0\AA\C0\CC 128\BA\B8\B4\D9 \C0\DB\C0\B8\B8\E9 255\B0\A1 \B5ǵ\B5\B7\CF \BC\B3\C1\A4
 
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
    Mat srcHSV;
    Mat dstHSV;
    Mat resRGB(ih, iw, CV_8UC3);

    cvtColor(srcRGB, srcHSV, CV_BGR2HSV);

    IplImage *iplImage = new IplImage(srcHSV);
    IplImage *dstImage = new IplImage(srcHSV); //cvCreateImage(cvSize(ih, iw), IPL_DEPTH_8U, 1);

    int i, j;
	const unsigned char HUE = 0, SAT = 1, VAL = 2;
	unsigned int StopSignCnt = 0;
	lane_Xmin = iplImage->width;
	lane_Xmax = 0;
	lane_Ymin = iplImage->height;
	lane_Xmax = 0;

	int ROI_Xs = (int)(iplImage->width * _ROI_Xmin), ROI_Xd = (int)(iplImage->width * _ROI_Xmax);
	int ROI_Ys = (int)(iplImage->height * current_Y_min), ROI_Yd = (int)(iplImage->height * _ROI_Ymax);
	
	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (/* Range to detect (Lane : Yellow or White) */
				// ROI range condition
				/*(i > MAX(0, ROI_Ys) && i < MIN(iplImage->height, ROI_Yd) &&
				j > MAX(0, ROI_Xs) && j < MIN(iplImage->width, ROI_Xd))
				&&*/
				// Color condition to detect
				(
					// Yellow Lane
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] >= LY_lowH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] <= LY_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] >= LY_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] <= LY_highS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] >= LY_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] <= LY_highV)
			   ))
			{
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)255;
				if (j < lane_Xmin) lane_Xmin = j;
				if (j > lane_Xmax) lane_Xmax = j;
				if (j < lane_Ymin) lane_Ymin = j;
				if (j > lane_Ymax) lane_Ymax = j;
			}
			else // elsewhere
			{
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)0;
			}
		}
	}

    dstHSV = cvarrToMat(dstImage);

    // ĳ\B4\CF \BE˰\ED\B8\AE\C1\F2 \C0\FB\BF\EB
    cv::Mat contours;
    cv::Canny(dstHSV, contours, 125, 350);
    
    // \BC\B1 \B0\A8\C1\F6 \C0\A7\C7\D1 \C7\E3\C7\C1 \BA\AFȯ
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, PI/180, 55);  // \C5\F5ǥ(vote) \C3ִ\EB \B0\B3\BC\F6
   

    // \BC\B1 \B1׸\AE\B1\E2
    cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
    //printf("Lines detected: %d\n", lines.size());

    float temp[2][2] = {{0,0}, {0,0}};

    std::vector<cv::Vec2f>::const_iterator it=lines.begin();

    while (it!=lines.end()) 
    {
        float rho = (*it)[0];   // ù \B9\F8° \BF\E4\BCҴ\C2 rho \B0Ÿ\AE
        float theta = (*it)[1]; // \B5\CE \B9\F8° \BF\E4\BCҴ\C2 \B5\A8Ÿ \B0\A2\B5\B5

        if(theta < 1.01 || theta > 3.12 )
        {
            ++it;
            continue;
        }

        if (theta < PI/2.)
        {
            if (temp[0][1] < theta)
            {
                temp[0][0] = rho;
                temp[0][1] = theta;
            }
        }
        else
        {
            if (temp[1][1] < theta)
            {
                temp[1][0] = rho;
                temp[1][1] = theta;
            }
        }

        ++it;
    }

    dLine.leftLine.rho = temp[0][0];
    dLine.leftLine.theta = temp[0][1];
    dLine.rightLine.rho = temp[1][0];
    dLine.rightLine.theta = temp[1][1];

    if (temp[0][1] && temp[1][1])
    {
        cv::Point pt1(0, temp[0][0]/sin(temp[0][1]));
	    cv::Point pt2(result.cols, (temp[0][0]-result.cols*cos(temp[0][1]))/sin(temp[0][1]));
	    cv::line(dstHSV, pt1, pt2, lineColor, 1);

        cv::Point pt3(0, temp[1][0]/sin(temp[1][1]));
        cv::Point pt4(result.cols, (temp[1][0]-result.cols*cos(temp[1][1]))/sin(temp[1][1]));
        cv::line(dstHSV, pt3, pt4, lineColor, 1);

        ptv = CalVanishPoint(pt1, pt2, pt3, pt4);

        dLine.VanishPoint = ptv;

        *mode = 0;
    }
    else if (temp[0][1] && !temp[1][1])
    {
        cv::Point pt1(0, temp[0][0]/sin(temp[0][1]));
	    cv::Point pt2(result.cols, (temp[0][0]-result.cols*cos(temp[0][1]))/sin(temp[0][1]));
	    cv::line(srcRGB, pt1, pt2, lineColor, 1);

        *mode = 1;
    }
    else if (!temp[0][1] && temp[1][1])
    {
        cv::Point pt1(0, temp[1][0]/sin(temp[1][1]));
	    cv::Point pt2(result.cols, (temp[1][0]-result.cols*cos(temp[1][1]))/sin(temp[1][1]));
	    cv::line(srcRGB, pt1, pt2, lineColor, 1);

        *mode = 2;
    }
    else
    {
        dLine.VanishPoint = ptv;

        *mode = 3;
    }

	/* left line */
	//cv::Point pt1(0, temp[0][0]/sin(temp[0][1]));
	//cv::Point pt2(result.cols, (temp[0][0]-result.cols*cos(temp[0][1]))/sin(temp[0][1]));
	//cv::line(srcRGB, pt1, pt2, lineColor, 1);

	/* right line */
    //cv::Point pt3(0, temp[1][0]/sin(temp[1][1]));
    //cv::Point pt4(result.cols, (temp[1][0]-result.cols*cos(temp[1][1]))/sin(temp[1][1]));
    //cv::line(srcRGB, pt3, pt4, lineColor, 1);

#if 0
	cv::circle(srcRGB, pt1, 10, (0,0,255));
	cv::circle(srcRGB, pt2, 10, (0,0,255));
	cv::circle(srcRGB, pt3, 10, (0,0,255));
	cv::circle(srcRGB, pt4, 10, (0,0,255));
#endif

#if 0
	printf("pt1 : %d %d\n", pt1.x, pt1.y);
	printf("pt2 : %d %d\n", pt2.x, pt2.y);
	printf("pt3 : %d %d\n", pt3.x, pt3.y);
	printf("pt4 : %d %d\n", pt4.x, pt4.y);
#endif

    

    //printf("vanish point : %d %d\n", ptv.x, ptv.y);
    //cv::circle(srcRGB, ptv, 7, (255,255,255), 3);

    cv::resize(dstHSV, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);

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

