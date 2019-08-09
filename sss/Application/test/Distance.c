#include <stdio.h>

#include "car_lib.h"

//#define DISTANCE_SENSOR
#define POSITION_CONTROL
//#define SERVO_CONTROL

void main(void)
{
    unsigned char status;
    short speed;
    unsigned char gain;
    int position, posInit, posDes, posRead;
    short angle;
    int channel;
    int data;
    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;

    CarControlInit();

#ifdef POSITION_CONTROL
     // 1. position control -------------------------------------------------------
    printf("\n\n 1. position control\n");

    //jobs to be done beforehand;
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
    speed = 30; // speed set     --> speed must be set when using position controller
    DesireSpeed_Write(speed);

    //control on/off
    status = PositionControlOnOff_Read();
    printf("PositionControlOnOff_Read() = %d\n", status);
    PositionControlOnOff_Write(CONTROL);

    //position controller gain set
    gain = PositionProportionPoint_Read();    // default value = 10, range : 1~50
    printf("PositionProportionPoint_Read() = %d\n", gain);
    gain = 30;
    PositionProportionPoint_Write(gain);

    //position write
    posInit = 0;  //initialize
    EncoderCounter_Write(posInit);
    
    //position set
    posDes = 5000;
    position = posInit+posDes;
    DesireEncoderCount_Write(position);

    position=DesireEncoderCount_Read();
    printf("DesireEncoderCount_Read() = %d\n", position);
    
    tol = 100;    // tolerance
    while(abs(posRead-position)>tol)
    {
        usleep(10000);
		posRead=EncoderCounter_Read();
		if(posRead != CHECKSUMERROR)
		{
			printf("EncoderCounter_Read() = %d\n", posRead);
		}
		else
		{
			printf("CHECKSUMERROR!, stop reading Encodercount! \n");
			break;
		}
    }
    sleep(1);
#endif

  #ifdef SERVO_CONTROL
    // 3. servo control ----------------------------------------------------------
    printf("\n\n 3. servo control\n");
    //steer servo set
    angle = SteeringServoControl_Read();
    printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1200;
    SteeringServoControl_Write(angle);

    //camera x servo set
    angle = CameraXServoControl_Read();
    printf("CameraXServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1400;
    CameraXServoControl_Write(angle);

    //camera y servo set
    angle = CameraYServoControl_Read();
    printf("CameraYServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

    angle = 1400;
    CameraYServoControl_Write(angle);
    
    sleep(1);
    angle = 1500;
    SteeringServoControl_Write(angle);
    CameraXServoControl_Write(angle);
    CameraYServoControl_Write(angle); 
#endif  

#ifdef DISTANCE_SENSOR
    printf("\n\n 4. distance sensor\n");

    for (i = 0; i < 1000; i++)
    {
        printf("please input ADC channel number\n");
        scanf("%d", &channel);

        for (j = 0; j < 50; j++)
        {
            data = DistanceSensor(channel);
            printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
            usleep(100000);
        }
    }
#endif
}

