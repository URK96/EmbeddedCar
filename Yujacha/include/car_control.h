#ifndef CAR_CONTROL_H_
#define CAR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    STRAIGHT,
    CURVELEFT,
    CURVERIGHT
}DrivingMode;

typedef struct LineInfo
{
    float rho;
    float theta;
}LineInfo;

typedef struct DriveLine
{
    struct LineInfo leftLine;
    struct LineInfo rightLine;
}DriveLine;


int speed, posInit, posDes;
unsigned char gain;
unsigned char enablePositionSpeed;
unsigned char checkLine;
unsigned char missionSequence = 0;

int distanceChannel[6] = { 1, 2, 3, 4, 5, 6 }, distance[6];

DriveLine driveLine;
DrivingMode driveMode;

void* positionSpeedControl(void *arg);
void speedPIDControl(int speed);
void LoopCheckDistance(int sensorIndex, int wantDistance, unsigned char isUp);
void LineStopThread();
void* ValanceThread(void *arg);


#ifdef __cplusplus
}
#endif


#endif