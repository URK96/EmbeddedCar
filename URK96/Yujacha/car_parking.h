#ifndef _CAR_PARKING_
#define _CAR_PARKING_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum 
{ 
    left = -1, 
    none = 0, 
    right = 1
}SteerDirection;

void ParallelParking();
void VerticalParking();

#ifdef __cplusplus
}
#endif

#endif