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

#include "imageProcess.h"
#include "car_control.h"

using namespace std;
using namespace cv;

extern "C" {

void ConvertImageForLCD(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
	Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
	Mat dstRGB(nh, nw, CV_8UC3, outBuf);

	cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

CvPoint CalVanishPoint(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4)
{
	CvPoint ptv = {0,0};	
	
	ptv.x = 320 * (pt3.y-pt1.y) / (pt2.y-pt1.y+pt3.y-pt4.y);
	ptv.y = (pt2.y-pt1.y) * ptv.x / 320 + pt1.y;

	return ptv;
}

void FindDriveLine(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
	int i, k;
	const unsigned char H = 0, S = 1, V = 2;
	float tempLine[2][2] = {{0, 0}, {0, 0}};

	cv::Mat contours;
	std::vector<cv::Vec2f> lines;
	std::vector<cv::Vec2f>::const_iterator it;

	Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
	Mat dstRGB(nh, nw, CV_8UC3, outBuf);
	Mat srcHSV;
	Mat dstHSV;

	cvtColor(srcRGB, srcHSW, CV_BGR2HSV);

	IplImage *srcIpl = new IplImage(srcHSV);
	IplImage *dstIpl = new IplImage(srcHSV);

	for (i = 0; i < srcIpl->height; ++i)
	{
		for (k = 0; k < srcIpl->width; ++k)
		{
			if 
			(
				((unsigned char)iplImage->imageData[i*srcIpl->widthStep + 3 * j + HUE] >= LY_lowH &&
				(unsigned char)iplImage->imageData[i*srcIpl->widthStep + 3 * j + HUE] <= LY_highH &&
				(unsigned char)iplImage->imageData[i*srcIpl->widthStep + 3 * j + SAT] >= LY_lowS &&
				(unsigned char)iplImage->imageData[i*srcIpl->widthStep + 3 * j + SAT] <= LY_highS &&
				(unsigned char)iplImage->imageData[i*srcIpl->widthStep + 3 * j + VAL] >= LY_lowV &&
				(unsigned char)iplImage->imageData[i*srcIpl->widthStep + 3 * j + VAL] <= LY_highV)
			)
			{
                dstImage->imageData[i*dstIpl->widthStep + 3 * j + SAT] = (unsigned char)255;
                dstImage->imageData[i*dstIpl->widthStep + 3 * j + VAL] = (unsigned char)255;
				dstImage->imageData[i*dstIpl->widthStep + 3 * j + HUE] = (unsigned char)255;
			}
			else
			{
				dstImage->imageData[i*dstIpl->widthStep + 3 * j + HUE] = (unsigned char)0;
                dstImage->imageData[i*dstIpl->widthStep + 3 * j + SAT] = (unsigned char)0;
                dstImage->imageData[i*dstIpl->widthStep + 3 * j + VAL] = (unsigned char)0;
			}
		}
	}

	dstHSV = cvarrToMat(dstImage);

	cv::Canny(dstHSV, contours, 125, 350);
	cv::HoughLines(contours, lines, 1, PI/180, 55);

	it = lines.begin();

	while (it!=lines.end()) 
    {
        float rho = (*it)[0];
        float theta = (*it)[1];

        if(theta < 1.01 || theta > 3.12 )
        {
            ++it;
            continue;
        }

        if (theta < PI/2.)
        {
            if (tempLine[0][1] < theta)
            {
                tempLine[0][0] = rho;
                tempLine[0][1] = theta;
            }
        }
        else
        {
            if (tempLine[1][1] < theta)
            {
                tempLine[1][0] = rho;
                tempLine[1][1] = theta;
            }
        }

        ++it;
    }

	driveLine.leftLine.rho = tempLine[0][0];
    driveLine.leftLine.theta = tempLine[0][1];
    driveLine.rightLine.rho = tempLine[1][0];
    driveLine.rightLine.theta = tempLine[1][1];

    if (driveLine.leftLine.theta && driveLine.rightLine.theta)
    {
        cv::Point pt1(0, driveLine.leftLine.rho/sin(driveLine.leftLine.theta));
	    cv::Point pt2(result.cols, (driveLine.leftLine.rho-result.cols*cos(driveLine.leftLine.theta))/sin(driveLine.leftLine.theta));
	    cv::line(dstHSV, pt1, pt2, lineColor, 1);

        cv::Point pt3(0, driveLine.rightLine.rho/sin(driveLine.rightLine.theta));
        cv::Point pt4(result.cols, (driveLine.rightLine.rho-result.cols*cos(driveLine.rightLine.theta))/sin(driveLine.rightLine.theta));
        cv::line(dstHSV, pt3, pt4, lineColor, 1);

        driveLine.VanishPoint = CalVanishPoint(pt1, pt2, pt3, pt4);

        driveMode = DrivingMode.STRAIGHT;
    }
    else if (driveLine.leftLine.theta && !driveLine.rightLine.theta)
    {
        cv::Point pt1(0, driveLine.leftLine.rho/sin(driveLine.leftLine.theta));
	    cv::Point pt2(result.cols, (driveLine.leftLine.rho-result.cols*cos(driveLine.leftLine.theta))/sin(driveLine.leftLine.theta));
	    cv::line(srcRGB, pt1, pt2, lineColor, 1);

        driveMode = DrivingMode.CURVERIGHT;
    }
    else if (!driveLine.leftLine.theta && driveLine.rightLine.theta)
    {
        cv::Point pt1(0, driveLine.rightLine.rho/sin(driveLine.rightLine.theta));
	    cv::Point pt2(result.cols, (driveLine.rightLine.rho-result.cols*cos(driveLine.rightLine.theta))/sin(driveLine.rightLine.theta));
	    cv::line(srcRGB, pt1, pt2, lineColor, 1);

        driveMode = DrivingMode.CURVELEFT;
    }
    else
        driveLine.VanishPoint = new CvPoint(0, 0);

}


}