#ifndef myPID_h
#define myPID_h

#include "stm32f1xx.h"

typedef struct {
    int info[8];
    int position;
    float Kp;
    float Ki;
    float Kd;
    int proportional, integrational, Derivative;
    int errordatas[10];
    int tot_error;
    int  maxspeedr;
    int maxspeedl;
    int MotorRightSpeedBase;
    int MotorLeftSpeedBase;
    int Var_ARR;
    int lastError;
    int i;
    int k;
	int motorspeed;
    int Variable;
    int active_count;
    int last_end;
    int white_count;

} MYPID;


extern MYPID MyPIDInstance;
extern MYPID *PID; // Declare a pointer to My_PID

#endif
