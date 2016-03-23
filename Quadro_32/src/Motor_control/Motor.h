/*
 * Motor.h
 *
 * Created: 24.02.2016 19:45:09
 *  Author: J
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

//#include "main.h"

/* struct PID */
typedef struct{
	float P_p;
	float I_p;
	float D_p;
	
	float P_r;
	float I_r;
	float D_r;
	
	float P_y;
	float I_y;
	float D_y;
	
}PID_terms;

void Motor_Task(void *pvParameters);
void PWM_init(void);
void PID_Calculate(Motor_Queue *position,PID_terms *pid);
void Filter_D(float * D_p,float * D_r,float* D_y);
void Motor_Update(Motor_Queue *position,PID_terms *pid);

#define PWM_FREQUENCY      480
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0


#endif /* MOTOR_H_ */