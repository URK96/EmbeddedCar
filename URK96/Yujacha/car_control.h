#ifndef _CAR_CONTROL_H_
#define _CAR_CONTROL_H_

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


int speed, posInit, posDes, posNow;
unsigned char gain;
unsigned char enablePositionSpeed;
unsigned char checkLine;
//unsigned char missionSequence = 0;

unsigned char enableCheckDistance;
int distance[6];

DriveLine driveLine;
DrivingMode driveMode;

void* positionSpeedControl(void *arg);
void speedPIDControl(int speed);
void* CheckDistance(void *arg);
void LoopCheckDistance(int sensorIndex, int wantDistance, unsigned char isUp);
void LineStopThread();
void ValanceCar();


#ifdef __cplusplus
}
#endif


#endif