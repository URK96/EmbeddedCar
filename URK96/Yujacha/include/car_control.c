#include <stdio.h>

#include "car_lib.h"
#include "car_control.h"

void* positionSpeedControl(void *arg)
{
    int position = 0, posRead = 0;

    while (1)
    {
        if (enablePositionSpeed)
        {
            SpeedControlOnOff_Write(CONTROL);
            PositionControlOnOff_Write(CONTROL);
            
            DesireSpeed_Write(speed);
            PositionProportionPoint_Write(gain);
            EncoderCounter_Write(posInit);
            
            position = posInit + posDes;
            DesireEncoderCount_Write(position);
        }
        
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

void* CheckDistance(void *arg)
{
    int i;

    while (1)
    {
        for (i = 0; i < 6; ++i)
            distance[i] = DistanceSensor(distanceChannel[i]);

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

            usleep(10000);
        }
    }
    else
    {
        while (1)
        {
            if (distance[sensorIndex] < wantDistance)
                break;

            usleep(10000);
        }
    }
}

void* LineStopThread(void *arg)
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

    return NULL;
}

void* ValanceThread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    int valanceDegree = 1530;
    int centerX = 160;

    float centerRightTheta = 2.20;

    SteeringServoControl_Write(valanceDegree);

    while (1)
    {
        /*int gap = vanishP.x - centerX;
        int gapDiff = 20;
        int gapTick = gap / gapDiff;
        int sleepTick = 1000000;

        if (driveMode == STRAIGHT)
        {
            if (gap <= -gapDiff)
            {
                SteeringServoControl_Write(valanceDegree + gapTick * 40);
                usleep(sleepTick);
                SteeringServoControl_Write(valanceDegree - gapTick * 40);
                usleep(sleepTick);
                SteeringServoControl_Write(valanceDegree);
            }
            else if (gap >= gapDiff)
            {
                SteeringServoControl_Write(valanceDegree - gapTick * 40);
                usleep(sleepTick);
                SteeringServoControl_Write(valanceDegree + gapTick * 40);
                usleep(sleepTick);
                SteeringServoControl_Write(valanceDegree);
            }
            else
                SteeringServoControl_Write(valanceDegree);
        }
        else if (driveMode == CURVERIGHT)
        {
            int steerDegree = valanceDegree;
            int cameraXDegree = 1500;

            while (steerDegree <= 1900)
            {
                steerDegree += 50;
                cameraXDegree += 20;
                SteeringServoControl_Write(steerDegree);
                CameraXServoControl_Write(cameraXDegree);
            }
        }
        else if (driveMode == CURVELEFT)
        {

        }*/

        
        float gap = driveLine.rightLine.theta - centerRightTheta;
        float gapDiff = 0.05;
        int gapTick = gap / gapDiff;
        int sleepTick = 500000;
        int steerP = 50;

        printf("Check Drive Mode \n");
        printf("left theta : %f, right theta : %f\n", driveLine.leftLine.theta, driveLine.rightLine.theta);
        checkLine = 0;

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
                //usleep(100000);
            }
            else if (gap >= gapDiff)
            {
                printf("right -> center\n");
                SteeringServoControl_Write(valanceDegree + gapTick * 140);
                usleep(sleepTick);
                SteeringServoControl_Write(valanceDegree - gapTick * 100);
                usleep(sleepTick);
                SteeringServoControl_Write(valanceDegree);
                //usleep(100000);
            }
            else
                SteeringServoControl_Write(valanceDegree);
        } 
        else if (driveMode == CURVERIGHT)
        {
            printf("Drive Mode : CurveRight\n");

            //usleep(1000000);

            //SteeringServoControl_Write(valanceDegree + 100);
            //usleep(500000);
            SteeringServoControl_Write(valanceDegree - 450);

            while (1)
            {
                if (driveMode == STRAIGHT)
                {
                    SteeringServoControl_Write(valanceDegree);
                    break;
                }

                usleep(5000);
            }
        }
        else if (driveMode == CURVELEFT)
        {
            printf("Drive Mode : CurveLeft\n");

            //usleep(500000);

            //SteeringServoControl_Write(valanceDegree - 100);
            //usleep(500000);
            SteeringServoControl_Write(valanceDegree + 450);

            while (1)
            {
                if (driveMode == STRAIGHT)
                {
                    SteeringServoControl_Write(valanceDegree);
                    break;
                }

                usleep(5000);
            }
        }


        checkLine = 1;

        if (driveMode == STRAIGHT)
            usleep(5000);
    }


    return NULL;
}
