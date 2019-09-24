#ifndef _IMAGE_PROCESS_H_
#define _IMAGE_PROCESS_H_ 

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

/* Stop Sign RED Detection Threshold */
#define SSR_lowH 170
#define SSR_highH 175
#define SSR_lowS 50
#define SSR_highS 255
#define SSR_lowV  90
#define SSR_highV 255

/* Red Detection Threshold */
#define Red_lowH 170
#define Red_highH 180
#define Red_lowS 50
#define Red_highS 255
#define Red_lowV  20
#define Red_highV 255

/* Yellow Detection Threshold */
#define Yellow_lowH 25
#define Yellow_highH 30
#define Yellow_lowS 50
#define Yellow_highS 255
#define Yellow_lowV  90
#define Yellow_highV 255

/* Green Detection Threshold */
#define Green_lowH 60
#define Green_highH 80
#define Green_lowS 30
#define Green_highS 150
#define Green_lowV  30
#define Green_highV 100


#ifdef __cplusplus
extern "C" {
#endif

typedef enum TrafficLights
{
    RED,
    YELLOW,
    GREEN,
    GREENARROW
}TrafficLights;

const unsigned char H = 0, S = 1, V = 2;

void ConvertImageForLCD(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
//CvPoint CalVanishPoint(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4);
void FindDriveLine();
int CheckStopSign(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
TrafficLights FindTrafficLights();


#ifdef __cplusplus
}
#endif

#endif