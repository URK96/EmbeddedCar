#ifndef IMAGE_PROCESS_H_
#define IMAGE_PROCESS_H_ 

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


#ifdef __cplusplus
extern "C" {
#endif

typedef struct LineInfo
{
    float rho;
    float theta;
}LineInfo;

typedef struct DriveLine
{
    struct LineInfo leftLine;
    struct LineInfo rightLine;

    CvPoint VanishPoint;
}DriveLine;


CvPoint CalVanishPoint(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4);


#ifdef __cplusplus
}
#endif

#endif