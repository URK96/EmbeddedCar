
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

/* Red Detection Threshold */
#define Red_lowH 170
#define Red_highH 175
#define Red_lowS 50
#define Red_highS 255
#define Red_lowV  90
#define Red_highV 255

/* Yellow Detection Threshold */
#define Yellow_lowH 25
#define Yellow_highH 30
#define Yellow_lowS 50
#define Yellow_highS 255
#define Yellow_lowV  90
#define Yellow_highV 255

/* Green Detection Threshold */
#define Green_lowH 50
#define Green_highH 80
#define Green_lowS 30
#define Green_highS 255
#define Green_lowV  30
#define Green_highV 255

#define REDa
#define YELLOWa
#define GREEN

using namespace std;
using namespace cv;

typedef enum
{
    STRAIGHT,
    CURVELEFT,
    CURVERIGHT
}DrivingMode;


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
     // �ɴ� �˰����� ����
    cv::Mat contours;
    cv::Canny(srcGRAY, // �׷��̷��� ����
        contours, // ��� �ܰ���
        125,  // ���� ��谪
        350);  // ���� ��谪

    // ������ ȭ�ҷ� �ܰ����� ǥ���ϹǷ� ��� ���� ����
    //cv::Mat contoursInv; // ���� ����
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
    // ��� ���� 128���� ������ 255�� �ǵ��� ����
 
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
    IplImage *dstImage = new IplImage(srcHSV);

    int i, j;
	const unsigned char HUE = 0, SAT = 1, VAL = 2;
	unsigned int colorCount = 0;

#ifdef RED
	
	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] >= Red_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] <= Red_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] >= Red_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] <= Red_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] >= Red_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] <= Red_highV
				)
            {
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)255;
			}
			else // elsewhere
			{
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)0;
			}
		}
	}

#endif

#ifdef YELLOW
	
	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] >= Yellow_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] <= Yellow_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] >= Yellow_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] <= Yellow_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] >= Yellow_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] <= Yellow_highV
				)
            {
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)255;
			}
			else // elsewhere
			{
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)0;
			}
		}
	}

#endif

#ifdef GREEN
	
	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] >= Green_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + HUE] <= Green_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] >= Green_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + SAT] <= Green_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] >= Green_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + VAL] <= Green_highV
				)
            {
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)255;

                colorCount++;
			}
			else // elsewhere
			{
				dstImage->imageData[i*dstImage->widthStep + 3 * j + HUE] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + SAT] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + VAL] = (unsigned char)0;
			}
		}
	}

#endif

    cout << "Green Lights : " << colorCount << endl;

    dstHSV = cvarrToMat(dstImage);

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

