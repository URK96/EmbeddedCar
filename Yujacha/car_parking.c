#include <stdio.h>

#include "util.h"
#include "car_lib.h"
#include "car_control.h"


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

    propGain = 20;
    integralGain = 20;
    differentialGain = 20;

    speedPIDControl(-30);


    // Start check back wall (Entering)

    printf("Check 1st Back Wall distance...\n");

    LoopCheckDistance(3, 750, 1);

    
    SteeringServoControl_Write(1250);

    printf("Check 1st-2 Back Wall distance...\n");

    LoopCheckDistance(3, 500, 0);

    printf("Check 2nd Back Wall distance...\n");

    LoopCheckDistance(3, 600, 1);


    // Start cehck back wall (Handling)

    SteeringServoControl_Write(1950);

    printf("Check 3rd back distance...\n");

    LoopCheckDistance(3, 1800, 1);

    SteeringServoControl_Write(1530);

    printf("Check 4rd back distance...\n");

    LoopCheckDistance(3, 2000, 1);

    printf("Stop Vehicle!\n");


    // Start line up vehicle (Forward)

    speedPIDControl(0);

    SteeringServoControl_Write(1000);

    enablePositionSpeed = 1;

    LoopCheckDistance(0, 2000, 1);

    enablePositionSpeed = 0;


    // Start line up vehicle (Backward)

    SteeringServoControl_Write(1530);

    speedPIDControl(-10);

    LoopCheckDistance(3, 2500, 1);

    printf("Stop Vehicle!\n");

    speedPIDControl(0);

    sleep(2);


    // Start exit parking area

    SteeringServoControl_Write(2000);

    enablePositionSpeed = 1;

    LoopCheckDistance(3, 1000, 0);

    SteeringServoControl_Write(1100);


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

    usleep(1500000);


    // Start backward movement

    enablePositionSpeed = 0;
    printf("Stop Vehicle!\n");

    SteeringServoControl_Write(1100);

    propGain = 20;
    integralGain = 20;
    differentialGain = 20;

    speedPIDControl(-30);


    // Start check back wall (Entering)

    printf("Check 1st Side Wall distance...\n");

    sleep(2);

    LoopCheckDistance(4, 1800, 1);

    SteeringServoControl_Write(1300);

    //LoopCheckDistance(3, 500, 0);

    //LoopCheckDistance(3, 700, 1);

    usleep(500000);

    SteeringServoControl_Write(1530);

    speedPIDControl(-20);

    LoopCheckDistance(3, 4000, 1);

    speedPIDControl(0);

    printf("Stop Vehicle!\n");

    sleep(2);

    /*enablePositionSpeed = 1;

    LoopCheckDistance(1, 500, 0);

    SteeringServoControl_Write(1100);

    LoopCheckDistance(4, 500, 0);

    SteeringServoControl_Write(1400);

    usleep(500000);

    SteeringServoControl_Write(1530);*/
}
