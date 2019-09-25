#include <stdio.h>

#include "car_lib.h"
#include "car_control.h"

int distanceChannels[6] = { 1, 2, 3, 4, 5, 6 };


void* positionSpeedControl(void *arg)
{
    int position = 0, posRead = 0;

    EncoderCounter_Write(posInit);

    while (1)
    {
        if (enablePositionSpeed)
        {
            SpeedControlOnOff_Write(CONTROL);
            PositionControlOnOff_Write(CONTROL);
            
            DesireSpeed_Write(speed);
            PositionProportionPoint_Write(gain);
            
            position = position + posDes;
            DesireEncoderCount_Write(posDes);
            EncoderCounter_Write(position);
        }

        posNow = EncoderCounter_Read();

        //printf("%d\n", posNow);
        
        usleep(100000);
    }
}

void speedPIDControl(int speed)
{
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    SpeedControlOnOff_Write(CONTROL);

    SpeedPIDProportional_Write(20);
    SpeedPIDIntegral_Write(20);
    SpeedPIDDifferential_Write(20);

    DesireSpeed_Write(speed);
}

void CheckDistance()
{
    int i;

    while (1)
    {
        for (i = 0; i < 6; ++i)
            distance[i] = DistanceSensor(distanceChannels[i]);

        //printf("%d %d %d %d %d %d\n", distance[0], distance[1], distance[2], distance[3], distance[4], distance[5]);
        usleep(100000);
    }
}

void LoopCheckDistance(int sensorIndex, int wantDistance, unsigned char isUp)
{
    if (isUp)
    {
        while (1)
        {
            if (distance[sensorIndex] > wantDistance)
                break;

            usleep(5000);
        }
    }
    else
    {
        while (1)
        {
            if (distance[sensorIndex] < wantDistance)
                break;

            usleep(5000);
        }
    }
}

void LineStopThread()
{
    char sensor;
    char byte = 0x80;

    while (1)
    {
        sensor = LineSensor_Read();        // black:1, white:0

        if (sensor == 0)
        {
            enablePositionSpeed = 0;
            break;
        }

        usleep(100);
    }
}

void ValanceCar()
{
    int valanceDegree = 1530;
    int centerX = 160;

    float centerRightTheta = 2.20;
    float centerLeftTheta = 1.70;
     
    float gap = driveLine.rightLine.theta - centerRightTheta;
    float gapDiff = 0.05;
    int gapTick = gap / gapDiff;
    int sleepTick = 10000;
    int steerP = 50;

    printf("Check Drive Mode \n");
    printf("left : %f, right : %f\n", driveLine.leftLine.theta, driveLine.rightLine.theta);

    if (driveMode == STRAIGHT)
    {
        printf("Drive Mode : Straight\n");

        if (gap <= -gapDiff)
        {
            printf("left -> center\n");
            SteeringServoControl_Write(valanceDegree + gapTick * 140);
            usleep(sleepTick);
            SteeringServoControl_Write(valanceDegree - gapTick * 100);
            usleep(sleepTick);
            SteeringServoControl_Write(valanceDegree);
            
        }
        else if (gap >= gapDiff)
        {
            printf("right -> center\n");
            SteeringServoControl_Write(valanceDegree + gapTick * 140);
            usleep(sleepTick);
            SteeringServoControl_Write(valanceDegree - gapTick * 100);
            usleep(sleepTick);
            SteeringServoControl_Write(valanceDegree);
            
        }
        else
            SteeringServoControl_Write(valanceDegree);
    } 
    else if (driveMode == CURVERIGHT)
    {
        printf("Drive Mode : CurveRight\n");

        SteeringServoControl_Write(valanceDegree - 450);

    }
    else if (driveMode == CURVELEFT)
    {
        printf("Drive Mode : CurveLeft\n");

        SteeringServoControl_Write(valanceDegree + 450);

    }
}
