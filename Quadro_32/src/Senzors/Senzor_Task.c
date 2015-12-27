/*
 * Senzor_Task.c
 *
 * Created: 27.1.2014 21:45:14
 *  Author: JR
 */ 

#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS.h"
#include "main.h"
#include "Senzor_Task.h"
#include "Extern_init.h"
#include "sx1276-Hal.h"
#include "L3Gx.h"
#include "LSM303DLHC.h"
#include "Filters.h"
#include "MPU_9150.h"

#define USE_MARG_AHRS	1

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile	 xQueueHandle		Queue_Senzor_Task;
extern		Senzor_id;
extern volatile xTimerHandle MPU_Timer;

/* offset for Gyro */
volatile short offset[4]={0,0,0,0};
	
extern fourthOrderData_t fourthOrder1000Hz[6];	//MAg and accel 3 axis
 	MPU9150_Queue GL_XYZ;


void MPU_TimerCallback(xTimerHandle pxTimer)
{	
	//if(xTimerStopFromISR(MPU_Timer,pdPASS)!=pdPASS){};
		
	//ioport_set_pin_level(PERIODE_PIN,true);
	GL_XYZ.temp=MPU9150_getMotion6_fifo(&GL_XYZ.MPU_FIFO[0],offset);
	ioport_toggle_pin_level(PERIODE_PIN);
	
	//ioport_set_pin_level(PERIODE_PIN,false);
	
// 	if(xQueueSendFromISR(Queue_Senzor_Task,&XYZ,1)!=pdPASS);
// 	{
// 		
// 	}
	
	
}

void MPU9150_INT()
{	
	MPU9150_Queue XYZ;
	uint8_t data[2];
	
	//MPU9150_Queue d
	NVIC_ClearPendingIRQ(PIOB_IRQn);
	/*xTaskResumeFromISR(Senzor_id);*/

	ioport_toggle_pin_level(PERIODE_PIN);
	
	XYZ.temp=MPU9150_getMotion6_fifo(XYZ.MPU_FIFO,offset);
	data[0]=0x4;
	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_USER_CTRL );
	
	//ioport_toggle_pin_level(PERIODE_PIN);
		
 	if(xQueueSendFromISR(Queue_Senzor_Task,&XYZ,1)!=pdPASS);
 	{
 			
 	}
	
// 		
//  	if(xQueueSendFromISR(Queue_Senzor_Task,&XYZ,10)!=pdPASS);
//  	{
//  		
//  	}
}

void INT_init()
{
	pmc_enable_periph_clk(ID_PIOB);
	

	pio_handler_set(PIOB, ID_PIOB, PIO_PB0, PIO_IT_RISE_EDGE, MPU9150_INT);
	
	pio_enable_interrupt(PIOB, PIO_PB0);
	
	NVIC_EnableIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn,2);
}

void Senzor_Task(void *pvParameters)
{	
	MPU9150_Queue XYZ;
	RF_Queue Semtech;
	//EulerAngles Angles;
	portTickType LastWakeTime;
	LastWakeTime=xTaskGetTickCount();
	
	char Kalibrace=NULL;
	short temp[6];
	static float uhel[3];
		
 	short GyroXYZ[3];
 	short MagXYZ[3];
	short AccXYZ[3];
	
	
	uhel[0]=0;
	
	taskENTER_CRITICAL();
	twi_init();
	taskEXIT_CRITICAL();
	
	MPU6050_Initialize();


	
	
//	INT_init();
// 	short GyroXYZ[80][3];
// 	short AccXYZ[80][3];
	
	if (Kalibrace==NULL)
	{
		
// 		MAG.Nasobek1=0.919999999;
// 		MAG.Nasobek2=1.0973854;
// 		MAG.X_Offset=208;
// 		MAG.Y_Offset=-158;
// 		MAG.Z_Offset=-2;
			
		Kalibrace=1;
		
	}
	
	short FIFO_MPU[1024];
	short poc=0;
	
	if(xTimerStart(MPU_Timer,0)!=pdPASS){}
		
	for (;;)
	{	
		
 		if(xQueueReceive(Queue_Senzor_Task,&XYZ,portMAX_DELAY)==pdPASS)
  		{	
// 			 for (short i=0;i<1;i++)
// 			 {
// 				GyroXYZ[0]=XYZ.MPU_FIFO[i] ;
// 				GyroXYZ[1]=XYZ.MPU_FIFO[i+1] ;
// 				GyroXYZ[2]=XYZ.MPU_FIFO[i+2];
// 				
// 				//
// 				//usart_serial_write_packet(BOARD_USART,&Semtech.Buffer[0],6);
			
				
			 }
			  
		//	if(xTimerStart(MPU_Timer,0)!=pdPASS){}
			
			temp[0]=(short)(GyroXYZ[0]);
			temp[1]=(short)(GyroXYZ[1]);
			temp[2]=(short)(GyroXYZ[2]);
				
			Semtech.Buffer[0]=(uint8_t)temp[0];	//LOW
			Semtech.Buffer[1]=(uint8_t)(temp[0]>>8);		//HIGH
			Semtech.Buffer[2]=(uint8_t) temp[1];;
			Semtech.Buffer[3]=(uint8_t) (temp[1]>>8);
			Semtech.Buffer[4]=(uint8_t) temp[2];
			Semtech.Buffer[5]=(uint8_t)( temp[2]>>8);
			
			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;	
		
	}
}

// 			if(xQueueSend(Queue_RF_Task,&Semtech,1))	//pdPASS=1-
// 			{
// 					
// 			}
	//	 }
// 				
// 		ioport_toggle_pin_level(PERIODE_PIN);
// 		
		//vTaskSuspend(Sx1276_id);			
	//	MPU9150_getMotion6(&AccXYZ[0],&AccXYZ[1],&AccXYZ[2],&GyroXYZ[0],&GyroXYZ[1],&GyroXYZ[2],offset);
	//	MPU9150_getMotion6_fifo(FIFO_MPU,offset);
		
// 		do 
// 		{
// 			AccXYZ[poc]=FIFO_MPU[poc++];
// 			AccXYZ[poc]=FIFO_MPU[poc++];
// 			AccXYZ[poc]=FIFO_MPU[poc++];
// 			
// 			GyroXYZ[poc]=FIFO_MPU[poc++];
// 			GyroXYZ[poc]=FIFO_MPU[poc++];
// 			GyroXYZ[poc]=FIFO_MPU[poc++];
// 			
// 		} while (FIFO_MPU[poc]!=0);
// 		
// 		for (short jj=0;jj<(poc/6);jj++)
// 		{
		
		
		//AccXYZ[0]=LPF_Fs1000Hz((float)AccXYZ[0],&fourthOrder200Hz[0]);
		
		
// 		uhel[0]=(LPF_Fs1000Hz_50CutOff((float)AccXYZ[1],&fourthOrder1000Hz[0]));
// 		temp[0]=(short)uhel[0];
		///AccXYZ[2]=(short)LPF_Fs1000Hz((float)AccXYZ[2],&fourthOrder200Hz[2]);
// 		MPU9150_getMotion6(&AccXYZ[0],&AccXYZ[1],&AccXYZ[2],&GyroXYZ[0],&GyroXYZ[1],&GyroXYZ[2],offset);
// 		temp[2]=AccXYZ[1];
// 		uhel[0]=(LPF_Fs1000Hz_50CutOff((float)AccXYZ[1],&fourthOrder1000Hz[0]));
// 		temp[1]=(short)uhel[0];
 		//GyroXYZ[0]=LPF_Fs1000Hz_50CutOff((float)GyroXYZ[0],&fourthOrder1000Hz[3]);
// 		GyroXYZ[1]=LPF_Fs1000Hz((float)GyroXYZ[1],&fourthOrder200Hz[4]);
// 		GyroXYZ[2]=LPF_Fs1000Hz((float)GyroXYZ[2],&fourthOrder200Hz[5]);
		
		
		//temp[1]=AccXYZ[0];
		//temp[2]=AccXYZ[2];
		
// 		AccXYZ[0]=((short)(AccXYZ[0]));
// 		//GyroXYZ[1]=HPF_Fs1000Hz((float)GyroXYZ[0],&fourthOrder200Hz[0]);
 	
// 		
// 		
// 		
// 		temp[0]=((short)uhel[1]);
 		//uhel[1]=(float)(-atan2((float)AccXYZ[2],(float)AccXYZ[1])*180/M_PI)+90;;
    	//	temp[2]=(short)((0.8*uhel[0])+0.2*(uhel[1]));
		//temp[1]=(short)uhel[1];
		//uhel[0]=(float)((GyroXYZ[0]));//*0.0013*4000/65536*4000/65536
		//uhel[1]+=uhel[0]*0.0009;	//900uS
		//temp[0]=(short)uhel[0];
			
 		
 		
		// }
		//vTaskDelayUntil(&LastWakeTime,(1/portTICK_RATE_MS));	//1000HZ
		//vTaskDelayUntil(&LastWakeTime,(140));	//50HZ
		
	//}
		
		

  
  /*
  
  X== Maximum + 130 ; ==Minimum -170
  Y== Maximum + 94 ; ==Minimum -190
  
 */
 