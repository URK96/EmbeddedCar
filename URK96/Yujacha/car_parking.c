#include <stdio.h>

#include "util.h"
#include "car_lib.h"
#include "car_control.h"
#include "car_parking.h"

void ParallelParking()
{
    // 1st wall dectection

    printf("Check 1st Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 500, 1);

    printf("Check 1st Wall No.3 Sensor distance...\n");

    LoopCheckDistance(2, 500, 1);

    printf("Check 1st Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 500, 0);


    // 2nd wall dectection

    printf("Check 2nd Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 500, 1);

    printf("Check 2nd Wall No.3 Sensor distance...\n");

    LoopCheckDistance(2, 500, 1);

    SteeringServoControl_Write(1900);

    printf("Check 2nd Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 300, 1);


    // Start backward movement

    usleep(200000);
    enablePositionSpeed = 0;
    printf("Stop Vehicle!\n");

    SteeringServoControl_Write(1050);

    speedPIDControl(-30);


    // Start check back wall (Entering)

    printf("Check 1st Back Wall distance...\n");

    LoopCheckDistance(3, 700, 1);

    
    SteeringServoControl_Write(1250);

    printf("Check 1st-2 Back Wall distance...\n");

    LoopCheckDistance(3, 500, 0);

    printf("Check 2nd Back Wall distance...\n");

    LoopCheckDistance(3, 500, 1);


    // Start check back wall (Handling)

    SteeringServoControl_Write(1950);

    printf("Check 3rd back distance...\n");

    LoopCheckDistance(3, 1800, 1);

    SteeringServoControl_Write(1530);

    printf("Check 4rd back distance...\n");

    LoopCheckDistance(3, 2400, 1);

    printf("Stop Vehicle!\n");


    // Start line up vehicle (Forward)

    speedPIDControl(0);

    SteeringServoControl_Write(1100);

    enablePositionSpeed = 1;

    LoopCheckDistance(0, 2000, 1);

    enablePositionSpeed = 0;


    // Start line up vehicle (Backward)

    SteeringServoControl_Write(1530);

    speedPIDControl(-10);

    LoopCheckDistance(3, 3600, 1);

    printf("Stop Vehicle!\n");

    speedPIDControl(0);

    sleep(2);


    // Start exit parking area

    SteeringServoControl_Write(2000);

    posDes = 100;
    speed = 100;
    enablePositionSpeed = 1;

    //LoopCheckDistance(0, 1400, 1);

    while (1)
    {
        if (distance[0] <= 500)
            break;

        if (distance[0] >= 1400)
        {
            usleep(500000);

            enablePositionSpeed = 0;

            SteeringServoControl_Write(1000);

            speedPIDControl(-30);

            LoopCheckDistance(3, 1800, 1);

            speedPIDControl(0);

            SteeringServoControl_Write(2000);

            enablePositionSpeed = 1;

            break;
        }

        usleep(5000);
    }

    printf("Out Parking Area\n");

    usleep(3000000);
    
    SteeringServoControl_Write(1100);

    usleep(2500000);

    posDes = 200;
    speed = 150;

    checkLine = 1;

    //SteeringServoControl_Write(1100);

    //speed = 150;


    // Other code is running with camera

}

void VerticalParking()
{
    // 1st wall dectection

    printf("Check 1st Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 500, 1);

    printf("Check 1st-2 Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 500, 0);



    // 2nd wall dectection

    printf("Check 2nd Wall No.2 Sensor distance...\n");

    LoopCheckDistance(1, 500, 1);

    SteeringServoControl_Write(2000);

    printf("Check 2nd-2 Wall No.2 Sensor distance...\n");

    sleep(1);


    // Start backward movement

    enablePositionSpeed = false;
    printf("Stop Vehicle!\n");

    SteeringServoControl_Write(1100);

    speedPIDControl(-30);


    // Start check back wall (Entering)

    printf("Check 1st Side Wall distance...\n");

    sleep(2);

    LoopCheckDistance(4, 1800, 0);

    SteeringServoControl_Write(1300);

    usleep(500000);

    SteeringServoControl_Write(1530);

    speedPIDControl(-20);

    LoopCheckDistance(3, 4000, 1);

    speedPIDControl(0);

    printf("Stop Vehicle!\n");

    sleep(2);

    enablePositionSpeed = 1;

    LoopCheckDistance(1, 500, 0);

    SteeringServoControl_Write(1100);

    sleep(3);

    SteeringServoControl_Write(1530);
}
