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

Mat srcRGB;
CvPoint vanishP;

void ConvertImageForLCD(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
	//Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
	Mat dstRGB(nh, nw, CV_8UC3, outBuf);

	srcRGB = Mat(ih, iw, CV_8UC3, srcBuf);

	cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/*CvPoint CalVanishPoint(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4)
{
	CvPoint ptv = {0,0};	
	
	ptv.x = 320 * (pt3.y-pt1.y) / (pt2.y-pt1.y+pt3.y-pt4.y);
	ptv.y = (pt2.y-pt1.y) * ptv.x / 320 + pt1.y;

	return ptv;
}*/

void FindDriveLine()
{
	int i, k;
	const unsigned char H = 0, S = 1, V = 2;
	float tempLine[2][2] = {{0, 0}, {0, 0}};
	float avgLine[2][2] = {{0, 0}, {0, 0}};
	int lineCount[2] = {0, 0};

	Scalar lineColor = cv::Scalar(255, 0, 0);
	cv::Mat contours;
	std::vector<cv::Vec2f> lines;
	std::vector<cv::Vec2f>::const_iterator it;
	
	Mat srcHSV;
	Mat dstHSV;

	cvtColor(srcRGB, srcHSV, CV_BGR2HSV);

	IplImage *srcIpl = new IplImage(srcHSV);
	IplImage *dstIpl = new IplImage(srcHSV);

	for (i = 0; i < srcIpl->height; ++i)
	{
		for (k = 0; k < srcIpl->width; ++k)
		{
			if (i <= 120)
			{
				dstIpl->imageData[i*dstIpl->widthStep + 3 * k + H] = (unsigned char)0;
                dstIpl->imageData[i*dstIpl->widthStep + 3 * k + S] = (unsigned char)0;
                dstIpl->imageData[i*dstIpl->widthStep + 3 * k + V] = (unsigned char)0;
				continue;
			}

			if 
			(
				((unsigned char)srcIpl->imageData[i*srcIpl->widthStep + 3 * k + H] >= LY_lowH &&
				(unsigned char)srcIpl->imageData[i*srcIpl->widthStep + 3 * k + H] <= LY_highH &&
				(unsigned char)srcIpl->imageData[i*srcIpl->widthStep + 3 * k + S] >= LY_lowS &&
				(unsigned char)srcIpl->imageData[i*srcIpl->widthStep + 3 * k + S] <= LY_highS &&
				(unsigned char)srcIpl->imageData[i*srcIpl->widthStep + 3 * k + V] >= LY_lowV &&
				(unsigned char)srcIpl->imageData[i*srcIpl->widthStep + 3 * k + V] <= LY_highV)
			)
			{
                dstIpl->imageData[i*dstIpl->widthStep + 3 * k + S] = (unsigned char)255;
                dstIpl->imageData[i*dstIpl->widthStep + 3 * k + V] = (unsigned char)255;
				dstIpl->imageData[i*dstIpl->widthStep + 3 * k + H] = (unsigned char)255;
			}
			else
			{
				dstIpl->imageData[i*dstIpl->widthStep + 3 * k + H] = (unsigned char)0;
                dstIpl->imageData[i*dstIpl->widthStep + 3 * k + S] = (unsigned char)0;
                dstIpl->imageData[i*dstIpl->widthStep + 3 * k + V] = (unsigned char)0;
			}
		}
	}

	dstHSV = cvarrToMat(dstIpl);

	cv::Canny(dstHSV, contours, 125, 350);
	cv::HoughLines(contours, lines, 2, PI/180, 45);

	cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);

	it = lines.begin();

	while (it!=lines.end()) 
    {
        float rho = (*it)[0];
        float theta = (*it)[1];

		//printf("rho : %f, theta : %f\n", rho, theta);

        if (theta < PI/2.)
        {
            if (tempLine[0][1] < theta)
            {
                tempLine[0][0] = rho;
                tempLine[0][1] = theta;
            }

			/*avgLine[0][0] += rho;
			avgLine[0][1] += theta;

			lineCount[0] += 1;*/
        }
        else
        {
            if (tempLine[1][1] < theta)
            {
                tempLine[1][0] = rho;
                tempLine[1][1] = theta;
            }

			/*avgLine[1][0] += rho;
			avgLine[1][1] += theta;

			lineCount[1] += 1;*/
        }

        ++it;
    }

	driveLine.leftLine.rho = tempLine[0][0];
    driveLine.leftLine.theta = tempLine[0][1];
    driveLine.rightLine.rho = tempLine[1][0];
    driveLine.rightLine.theta = tempLine[1][1];

	/*lineCount[0] = (lineCount[0] == 0) ? 1 : lineCount[0];
	lineCount[1] = (lineCount[1] == 0) ? 1 : lineCount[1];

	driveLine.leftLine.rho = avgLine[0][0] / lineCount[0];
	driveLine.leftLine.theta = avgLine[0][1] / lineCount[0];
	driveLine.rightLine.rho = avgLine[1][0] / lineCount[1];
	driveLine.rightLine.theta = avgLine[1][1] / lineCount[1];*/

    if (driveLine.leftLine.theta && driveLine.rightLine.theta)
    {
        cv::Point pt1(0, driveLine.leftLine.rho/sin(driveLine.leftLine.theta));
	    cv::Point pt2(result.cols, (driveLine.leftLine.rho-result.cols*cos(driveLine.leftLine.theta))/sin(driveLine.leftLine.theta));
	    //cv::line(dstHSV, pt1, pt2, lineColor, 1);

        cv::Point pt3(0, driveLine.rightLine.rho/sin(driveLine.rightLine.theta));
        cv::Point pt4(result.cols, (driveLine.rightLine.rho-result.cols*cos(driveLine.rightLine.theta))/sin(driveLine.rightLine.theta));
        //cv::line(dstHSV, pt3, pt4, lineColor, 1);

        //vanishP = CalVanishPoint(pt1, pt2, pt3, pt4);

        driveMode = STRAIGHT;
    }
    else if (driveLine.leftLine.theta && !driveLine.rightLine.theta)
    {
        cv::Point pt1(0, driveLine.leftLine.rho/sin(driveLine.leftLine.theta));
	    cv::Point pt2(result.cols, (driveLine.leftLine.rho-result.cols*cos(driveLine.leftLine.theta))/sin(driveLine.leftLine.theta));
	    //cv::line(srcRGB, pt1, pt2, lineColor, 1);

        driveMode = CURVERIGHT;
    }
    else if (!driveLine.leftLine.theta && driveLine.rightLine.theta)
    {
        cv::Point pt1(0, driveLine.rightLine.rho/sin(driveLine.rightLine.theta));
	    cv::Point pt2(result.cols, (driveLine.rightLine.rho-result.cols*cos(driveLine.rightLine.theta))/sin(driveLine.rightLine.theta));
	    //cv::line(srcRGB, pt1, pt2, lineColor, 1);

        driveMode = CURVELEFT;
    }
    else
	{
        vanishP.x = 0;
		vanishP.y = 0;
	}
}

int CheckStopSign(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
    int i, j;
	unsigned int stopSignCnt = 0;
    
    Mat srcHSV;
    Mat dstHSV;

    cvtColor(srcRGB, srcHSV, CV_BGR2HSV);

    IplImage *iplImage = new IplImage(srcHSV);
    IplImage *dstImage = new IplImage(srcHSV);

	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] >= SSR_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] <= SSR_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] >= SSR_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] <= SSR_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] >= SSR_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] <= SSR_highV
				)
            {
				dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)255;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)255;

                stopSignCnt++;
			}
			else
			{
				dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)0;
                dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)0;
			}
		}
	}

    return stopSignCnt;
}

TrafficLights FindTrafficLights()
{    
    int i, j;
	unsigned int colorCount[3] = { 0, 0, 0 };
	TrafficLights light = RED;

    Mat srcHSV;
    Mat dstHSV;

    cvtColor(srcRGB, srcHSV, CV_BGR2HSV);

    IplImage *iplImage = new IplImage(srcHSV);
    IplImage *dstImage = new IplImage(srcHSV);

	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] >= Red_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] <= Red_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] >= Red_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] <= Red_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] >= Red_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] <= Red_highV
				)
            {
				//dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)255;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)255;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)255;

				colorCount[0]++;
			}
			else
			{
				//dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)0;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)0;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)0;
			}
		}
	}

	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] >= Yellow_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] <= Yellow_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] >= Yellow_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] <= Yellow_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] >= Yellow_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] <= Yellow_highV
				)
            {
				//dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)255;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)255;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)255;
			
				colorCount[1]++;
			}
			else
			{
				//dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)0;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)0;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)0;
			}
		}
	}

	for (i = 0; i < iplImage->height; i++)
	{
		for (j = 0; j < iplImage->width; j++) 
		{
			if (
					((unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] >= Green_lowH &&
                    (unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + H] <= Green_highH &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] >= Green_lowS &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + S] <= Green_highS) &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] >= Green_lowV &&
					(unsigned char)iplImage->imageData[i*iplImage->widthStep + 3 * j + V] <= Green_highV
				)
            {
				//dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)255;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)255;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)255;

                colorCount[2]++;
			}
			else
			{
				//dstImage->imageData[i*dstImage->widthStep + 3 * j + H] = (unsigned char)0;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + S] = (unsigned char)0;
                //dstImage->imageData[i*dstImage->widthStep + 3 * j + V] = (unsigned char)0;
			}
		}
	}

	printf("Red : %d, Yellow : %d, Green: %d\n", colorCount[0], colorCount[1], colorCount[2]);

	if ((colorCount[0] > 50) && (colorCount[1] < 10) && (colorCount[2] < 10))
		light = RED;
	else if ((colorCount[1] > 50) && (colorCount[0] < 10) && (colorCount[2] < 10))
		light = YELLOW;
	else if ((colorCount[2] > 10) && (colorCount[0] < 10) && (colorCount[1] < 10))
		if (colorCount[2] < 70)
			light = GREENARROW;
		else
			light = GREEN;

    return light;
}


}