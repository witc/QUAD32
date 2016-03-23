/*
 * Motor.c
 *
 * Created: 24.02.2016 19:44:05
 *  Author: J
 */ 

#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS.h"
#include "main.h"
#include "Motor.h"

#define P_PITCH		0.05f
#define I_PITCH		0.001f
#define D_PITCH		1.01f

#define P_ROLL		0.2f
#define I_ROLL		0.001f
#define D_ROLL		1.01f


extern volatile	xQueueHandle		Queue_Motor_Task;
volatile short temp=0;

pwm_channel_t pwm_channel_0;
pwm_channel_t pwm_channel_1;
pwm_channel_t pwm_channel_2;
pwm_channel_t pwm_channel_3;

float	wanted_pitch=0;
float   wanted_roll=0;
float	wanted_yaw=0;
uint16_t	wanted_power=0;

/* D filtrace */
Filter_D(float * D_p,float* D_r,float * D_y)
{	
	#define ARRAY	20
	
	static float Sum_p=0, Sum_r=0, Sum_y=0;
	static float Pole_p[ARRAY], Pole_r[ARRAY], Pole_y[ARRAY];
	static uint16_t counter=0;
	
	Sum_p+=*D_p;
	Pole_p[counter]=*D_p;
	
	Sum_r+=*D_r;
	Pole_r[counter]=*D_r;
	
	Sum_y+=*D_y;
	Pole_y[counter]=*D_y;
	
	counter++;
	if (counter==ARRAY) counter=0;
	
	Sum_p-=Pole_p[counter];
	Sum_r-=Pole_r[counter];
	Sum_y-=Pole_y[counter];
	
	*D_p=(float)(Sum_p/ARRAY);
	*D_r=(float)(Sum_r/ARRAY);
	*D_y=(float)(Sum_y/ARRAY);
}

/* PID */
PID_Calculate(Motor_Queue *position,PID_terms *pid)
{	
	volatile float p_p, p_r, p_y;
	static float integral_p, integral_r, integral_y;
	float derivative_p, derivative_r, derivative_y;
	static float prev_error_p, prev_error_r, prev_error_y;
	static float  dt=0.001f;
	static portTickType CurrentTime=0;
	static portTickType LastTime=0;
		
	LastTime=CurrentTime;
	CurrentTime=xTaskGetTickCount();
	dt=(double)((CurrentTime-LastTime));
	dt/=1000;			
	/* P složka */
	p_p = wanted_pitch - position->pitch;
	p_r = wanted_roll - position->roll;
	p_y = wanted_yaw - position->yaw;
	
	/* I položka */
	integral_p = integral_p + p_p*dt;
	integral_r = integral_r + p_r*dt;
	integral_y = integral_y + p_y*dt;
	
	if (integral_p>5) integral_p=5;
	if (integral_r>5) integral_r=5;
	if (integral_y>5) integral_y=5;
	
		
	/* D složka */
	derivative_p = (float)((p_p - prev_error_p)/dt);
	derivative_r = (float)((p_r - prev_error_r)/dt);
	derivative_y = (float)((p_y - prev_error_y)/dt);
	
	prev_error_p=p_p;
	prev_error_r=p_r;
	prev_error_y=p_y;
	
	/* filtrace D složky pid - LP */
	Filter_D(&derivative_p,&derivative_r,&derivative_y);
	
	pid->P_p=p_p;
	pid->P_r=p_r;
	pid->P_y=p_y;
	
	pid->I_p=integral_p;
	pid->I_r=integral_r;
	pid->I_y=integral_y;
	
	pid->D_p=derivative_p;
	pid->D_r=derivative_r;
	pid->D_y=derivative_y;
	
}


void Motor_Update(Motor_Queue *position,PID_terms *pid)
{	
	float Power_M1=0;
	float Power_M2=0;
	float Power_M3=0;
	float Power_M4=0;
	
	uint16_t Final_M1=0;
	uint16_t Final_M2=0;
	uint16_t Final_M3=0;
	uint16_t Final_M4=0;
	
	/* PITCH */
 	Power_M1+=-(float)(P_PITCH*pid->P_p)+I_PITCH*pid->I_p;//+D_PITCH*pid->D_p);//+pid->I_p+pid->D_p;
 	Power_M3+=Power_M1;
 	Power_M2+=(-1)*Power_M1;
  	Power_M4+=(-1)*Power_M1;
	
	/* ROLL */
// 	Power_M1+=+(float)(P_ROLL*pid->P_r);//+I_ROLL*pid->I_r+D_PITCH*pid->D_p);//+pid->I_p+pid->D_p;
// 	Power_M2+=Power_M1;
// 	Power_M3+=-Power_M1;
// 	Power_M4+=-Power_M1;
		
	if (wanted_power>55)
	{
		Final_M1=(uint16_t)(wanted_power+Power_M1);
		Final_M2=(uint16_t)(wanted_power+Power_M2);
		Final_M3=(uint16_t)(wanted_power+Power_M3);
		Final_M4=(uint16_t)(wanted_power+Power_M4);
	
	}
	else
	{
		Final_M1=50;
		Final_M2=50;
		Final_M3=50;
		Final_M4=50;
	}
	
	if (Final_M1>95) Final_M1=95;
	if (Final_M2>95) Final_M2=95;
	if (Final_M3>95) Final_M3=95;
	if (Final_M4>95) Final_M4=95;
	
	
	pwm_channel_disable(PWM, 0);
	pwm_channel_disable(PWM, 1);
	pwm_channel_disable(PWM, 2);
	pwm_channel_disable(PWM, 3);
	
	pwm_channel_update_duty(PWM,&pwm_channel_0,Final_M1);
	pwm_channel_update_duty(PWM,&pwm_channel_1,Final_M2);
	pwm_channel_update_duty(PWM,&pwm_channel_2,Final_M3);
	pwm_channel_update_duty(PWM,&pwm_channel_3,Final_M4);
	
	pwm_channel_enable(PWM, 0);
	pwm_channel_enable(PWM, 1);
	pwm_channel_enable(PWM, 2);
	pwm_channel_enable(PWM, 3);
	
	

		
}
/* PWM init function */
void PWM_init(void)
{	
	pmc_enable_periph_clk(ID_PWM);
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	
	/* pins */
	pio_configure_pin(PWM_M1,PIO_TYPE_PIO_PERIPH_B);
	ioport_disable_pin(PWM_M1);	
	ioport_set_pin_dir(PWM_M1,IOPORT_DIR_OUTPUT);
	
	pio_configure_pin(PWM_M2,PIO_TYPE_PIO_PERIPH_A);
	ioport_disable_pin(PWM_M2);
	ioport_set_pin_dir(PWM_M2,IOPORT_DIR_OUTPUT);
	
	pio_configure_pin(PWM_M3,PIO_TYPE_PIO_PERIPH_B);
	ioport_disable_pin(PWM_M3);
	ioport_set_pin_dir(PWM_M3,IOPORT_DIR_OUTPUT);
	
	pio_configure_pin(PWM_M4,PIO_TYPE_PIO_PERIPH_B);
	ioport_disable_pin(PWM_M4);
	ioport_set_pin_dir(PWM_M4,IOPORT_DIR_OUTPUT);
	 
	/* Disable PWM channels*/
	pwm_channel_disable(PWM, 0);
	pwm_channel_disable(PWM, 1);
	pwm_channel_disable(PWM, 2);
	pwm_channel_disable(PWM, 3);
		
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_bus_hz(PWM)
	};
	pwm_init(PWM, &clock_setting);
	
	/* Period is left-aligned */
	pwm_channel_0.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_0.polarity =PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_0.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_0.ul_duty = 50;
	pwm_channel_0.channel = 0;
	pwm_channel_init(PWM, &pwm_channel_0);
	
	/* Period is left-aligned */
	pwm_channel_1.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_1.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_1.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_1.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_1.ul_duty = 95;
	pwm_channel_1.channel = 1;
	pwm_channel_init(PWM, &pwm_channel_1);
	
	/* Period is left-aligned */
	pwm_channel_2.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_2.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_2.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_2.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_2.ul_duty = 50;
	pwm_channel_2.channel = 2;
	pwm_channel_init(PWM, &pwm_channel_2);
	
	/* Period is left-aligned */
	pwm_channel_3.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_3.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_3.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_3.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_3.ul_duty = 50;
	pwm_channel_3.channel = 3;
	pwm_channel_init(PWM, &pwm_channel_3);

	/*dissable int */
	pwm_channel_disable_interrupt(PWM, 0, 0);
	pwm_channel_disable_interrupt(PWM, 1, 0);
	pwm_channel_disable_interrupt(PWM, 2, 0);
	pwm_channel_disable_interrupt(PWM, 3, 0);
	
	pwm_channel_0.channel = PWM_CHANNEL_0;
	pwm_channel_1.channel = PWM_CHANNEL_1;
	pwm_channel_2.channel = PWM_CHANNEL_2;
	pwm_channel_3.channel = PWM_CHANNEL_3;
	pwm_channel_update_duty(PWM,&pwm_channel_0,95);
	pwm_channel_update_duty(PWM,&pwm_channel_1,95);
	pwm_channel_update_duty(PWM,&pwm_channel_2,95);
	pwm_channel_update_duty(PWM,&pwm_channel_3,95);
// 	
// 	/* Enable PWM channels */
 	pwm_channel_enable(PWM, 0);
 	pwm_channel_enable(PWM, 1);
 	pwm_channel_enable(PWM, 2);
 	pwm_channel_enable(PWM, 3);
	vTaskDelay(500/portTICK_RATE_MS);
	
	pwm_channel_disable(PWM, 0);
	pwm_channel_disable(PWM, 1);
	pwm_channel_disable(PWM, 2);
	pwm_channel_disable(PWM, 3);
	
	pwm_channel_update_duty(PWM,&pwm_channel_0,50);
	pwm_channel_update_duty(PWM,&pwm_channel_1,50);
	pwm_channel_update_duty(PWM,&pwm_channel_2,50);
	pwm_channel_update_duty(PWM,&pwm_channel_3,50);
	/* Enable PWM channels */
	pwm_channel_enable(PWM, 0);
	pwm_channel_enable(PWM, 1);
	pwm_channel_enable(PWM, 2);
	pwm_channel_enable(PWM, 3);
	vTaskDelay(100/portTICK_RATE_MS);

	
}
 
 
 void Motor_Task(void *pvParameters)
 {
	Motor_Queue Position;
	PID_terms	Pid;
	uint16_t y0_output=0;
	/* PWM inicializace */ 
 	PWM_init();
// 	
// 	pwm_channel_update_duty(PWM,&pwm_channel_0,80);
// 	pwm_channel_update_duty(PWM,&pwm_channel_1,80);
// 	pwm_channel_update_duty(PWM,&pwm_channel_2,80);
// 	pwm_channel_update_duty(PWM,&pwm_channel_3,80);
	while(1)
	{
 		if(xQueueReceive(Queue_Motor_Task,&Position,portMAX_DELAY)==pdPASS)
 		{	
 			if (Position.type_of_data==FROM_SENZOR)
 			{
 				PID_Calculate(&Position,&Pid);
				Motor_Update(&Position,&Pid);
 			}
 			else if(Position.type_of_data==FROM_TX)
 			{
 				wanted_power=(uint16_t)((-0.045f*Position.TX_CH_xx[3])+185);
				 if (wanted_power>95) wanted_power=95;
				 if (wanted_power<50) wanted_power=50;
				
				wanted_pitch=(float)(Position.TX_CH_xx[2]*(-0.09f)+225);
				wanted_roll=(float)(Position.TX_CH_xx[0]*(-0.09f)+225);
				wanted_yaw=(float)(Position.TX_CH_xx[1]*0.09f-225);
				
				

 			}
 			
 			
 			
 		}
			 	
	}
 }