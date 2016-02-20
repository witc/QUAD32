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
#include "MadgwickAHRS.h"


#define USE_MARG_AHRS	1

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile	 xQueueHandle		Queue_Senzor_Task;

extern volatile xTimerHandle MPU_Timer;

/* offset for Gyro */
volatile short offset[4]={0,0,0,0};
	
extern fourthOrderData_t fourthOrder1000Hz[6];	//MAg and accel 3 axis
MPU9150_Queue GL_XYZ;


#if (FIFO_MPU9150==1)

void MPU_TimerCallback(xTimerHandle pxTimer)
{	
	//if(xTimerStopFromISR(MPU_Timer,pdPASS)!=pdPASS){};
		
	ioport_set_pin_level(PERIODE_PIN,true);
	GL_XYZ.temp=MPU9150_getMotion6_fifo(&GL_XYZ.MPU_FIFO[0]);
	//ioport_toggle_pin_level(PERIODE_PIN);
	ioport_set_pin_level(PERIODE_PIN,false);
	
   	if(xQueueSendFromISR(Queue_Senzor_Task,&GL_XYZ,portMAX_DELAY)!=pdPASS);
   	{
   		
   	}
	
	
}

#endif

void MPU9150_INT(void)
{	
		
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdFALSE;	//pøerušení se dokonèí celé= pdFalse
	
	/*xTaskResumeFromISR(Senzor_id);*/
//	pio_disable_interrupt(PIOB, PIO_PB0);
	//ioport_set_pin_level(PERIODE_PIN,true);
 	MPU9150_getMotion66(GL_XYZ.MPU_FIFO,offset);
	NVIC_ClearPendingIRQ(PIOB_IRQn);
	//ioport_set_pin_level(PERIODE_PIN,false);
	
	//ioport_toggle_pin_level(PERIODE_PIN);
		
   	while(xQueueSendToBackFromISR(Queue_Senzor_Task,&GL_XYZ,&xHigherPriorityTaskWoken)!=pdTRUE); //èekám dokud se data neodešlou do fronty
  
   
   //if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();

}

void INT_init(void)
{
	pmc_enable_periph_clk(ID_PIOB);
	
	pio_handler_set(PIOB, ID_PIOB, PIO_PB0, PIO_IT_RISE_EDGE, MPU9150_INT);
	
	pio_enable_interrupt(PIOB, PIO_PB0);
	
	NVIC_EnableIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}

void Senzor_Task(void *pvParameters)
{	
	MPU9150_Queue XYZ;
	RF_Queue Semtech;
	EulerAngles Angles;
	portTickType CurrentTime;
	portTickType LastTime;
		
	char Kalibrace=NULL;
	short temp[6];
	static float uhel[3];
	float f_temp[3];
	uint16_t packet_count=0;
		
 	short GyroXYZ[3];
 	short MagXYZ[3];
	short AccXYZ[3];
	
	double dt=0;
	uint8_t data[2];
	uhel[0]=0;
	
	taskENTER_CRITICAL();
	twi_init();
	MPU6050_Initialize();
	taskEXIT_CRITICAL();

#if (RAW_MPU9150==1)
	//MPU9150_Gyro_Tempr_Bias_no_fifo(offset);
	CurrentTime=xTaskGetTickCount();
	LastTime=xTaskGetTickCount();
#elif ((RAW_INT_MPU9150==1))

		//MPU9150_Gyro_Tempr_Bias_no_fifo(offset);
		INT_init();
		  
#elif (FIFO_MPU9150==1)
		//MPU9150_Gyro_Tempr_Bias(offset);	//-47,-7,17
			if(xTimerStart(MPU_Timer,0)!=pdPASS){}
		
#else
# error "Please specify Way to get a datta from MPU9150"
#endif
		
		offset[0]=-47;
		offset[1]=-7;
		offset[2]=17;
	
	
for (;;)
{	
	  		
#if (RAW_MPU9150==1)
				
		ioport_set_pin_level(PERIODE_PIN,true);
		MPU9150_getMotion66(XYZ.MPU_FIFO,offset);
		ioport_set_pin_level(PERIODE_PIN,false);
		
		GyroXYZ[0]=(((short)(XYZ.MPU_FIFO[0]) << 8 ) | XYZ.MPU_FIFO[1])-offset[0];
		GyroXYZ[1]=(((short)(XYZ.MPU_FIFO[2]) << 8 ) | XYZ.MPU_FIFO[3])-offset[1];
		GyroXYZ[2]=(((short)(XYZ.MPU_FIFO[4]) << 8 ) | XYZ.MPU_FIFO[5])-offset[2];
		
		LastTime=CurrentTime;
		CurrentTime=xTaskGetTickCount();
		dt=(double)((CurrentTime-LastTime));
		
		f_temp[0]=(float)((GyroXYZ[0]*0.06103515f)*dt/1000);
		f_temp[1]=(float)((GyroXYZ[1]*0.06103515f)*dt/1000);
		f_temp[2]=(float)((GyroXYZ[2]*0.06103515f)*dt/1000);
		uhel[0]+=(float)f_temp[0];
		uhel[1]+=(float)f_temp[1];
		uhel[2]+=(float)f_temp[2];
				 			
#elif ((RAW_INT_MPU9150==1))		
		 
		if(xQueueReceive(Queue_Senzor_Task,&XYZ,portMAX_DELAY)==pdPASS)
		{	
			
			ioport_set_pin_level(PERIODE_PIN,true);
			AccXYZ[0]=(((short)(XYZ.MPU_FIFO[0]) << 8 ) | XYZ.MPU_FIFO[1]);
			AccXYZ[1]=(((short)(XYZ.MPU_FIFO[2]) << 8 ) | XYZ.MPU_FIFO[3]);
			AccXYZ[2]=(((short)(XYZ.MPU_FIFO[4]) << 8 ) | XYZ.MPU_FIFO[5]);
			
			GyroXYZ[0]=(((short)(XYZ.MPU_FIFO[8]) << 8 ) | XYZ.MPU_FIFO[9])-offset[0];
			GyroXYZ[1]=(((short)(XYZ.MPU_FIFO[10]) << 8 ) | XYZ.MPU_FIFO[11])-offset[1];
			GyroXYZ[2]=(((short)(XYZ.MPU_FIFO[12]) << 8 ) | XYZ.MPU_FIFO[13])-offset[2];
			
			dt=1/1000;
				
			f_temp[0]=(float)((GyroXYZ[0]*0.06103515f)*M_PI/180);
			f_temp[1]=(float)((GyroXYZ[1]*0.06103515f)*M_PI/180);
			f_temp[2]=(float)((GyroXYZ[2]*0.06103515f)*M_PI/180);
			
// 			LastTime=CurrentTime;
// 			CurrentTime=xTaskGetTickCount();
// 			dt=(double)((CurrentTime-LastTime));
			
			
			MadgwickAHRSupdateIMU(f_temp[0],f_temp[1],f_temp[2],(float)(AccXYZ[0]),(float)(AccXYZ[1]),(float)(AccXYZ[2]),0.001f,&Angles);//,0,0,0
			
 			uhel[0]=Angles.pitch;
 			uhel[1]=Angles.roll;
 			uhel[2]=Angles.yaw;
 			
// 			uhel[0]+=(float)f_temp[0];
// 			uhel[1]+=(float)f_temp[1];
// 			uhel[2]+=(float)f_temp[2];
			ioport_set_pin_level(PERIODE_PIN,false);
		}
		
#elif (FIFO_MPU9150==1)

		if(xQueueReceive(Queue_Senzor_Task,&XYZ,portMAX_DELAY)==pdPASS)
		{	
			/* nulovani indexovani bufferu FIFA */	 
			packet_count = 0;
			while (XYZ.temp>0)
			{
				XYZ.temp-=6;
				temp[0]=(((short)(XYZ.MPU_FIFO[packet_count++]) << 8 ) | XYZ.MPU_FIFO[packet_count++]);//-offset[0];
				temp[1]=(((short)(XYZ.MPU_FIFO[packet_count++]) << 8) | XYZ.MPU_FIFO[packet_count++]);//-offset[1];
				temp[2]=(((short)(XYZ.MPU_FIFO[packet_count++]) << 8) | XYZ.MPU_FIFO[packet_count++]);//-offset[2];
							  
				temp[0]-=offset[0];
				temp[1]-=offset[1];
				temp[2]-=offset[2];
							  
				f_temp[0]=(float)((temp[0]*0.06103515f)*0.001f);
				f_temp[1]=(float)((temp[1]*0.06103515f)*0.001f);
				f_temp[2]=(float)((temp[2]*0.06103515f)*0.001f);
							  
				uhel[0]+=(float)f_temp[0];
				uhel[1]+=(float)f_temp[1];
				uhel[2]+=(float)f_temp[2];
			}
		}
		
#else
# error "Please specify Way to get a data from MPU9150"
#endif
//  			 q0=0.01f;
//  			 q1=0.02f;
//  			 q2=0.03f;
//  			 q3=0.04f;
			  
// 			 temp[0]=(short)(q0*1000);
// 			 temp[1]=(short)(q1*1000);
// 			 temp[2]=(short)(q2*1000);
// 			 temp[3]=(short)(q3*1000);
			 
// 			  temp[0]=(short)(0.333f*1000);
// 			 temp[1]=(short)(0.999f*1000);
// 			 temp[2]=(short)(0.123f*1000);
// 			 temp[3]=(short)(0.111f*1000);
// 			 
    		temp[0]=(short)(uhel[0]);
           	temp[1]=(short)(uhel[1]);
           	temp[2]=(short)(uhel[2]);
// 						
//   			temp[0]	   =AccXYZ[0];
//   			temp[1]	   =AccXYZ[1];
//   			temp[2]	   =AccXYZ[2];
 				
			Semtech.Buffer[0]=(uint8_t)temp[0];	//LOW
			Semtech.Buffer[1]=(uint8_t)(temp[0]>>8);		//HIGH
			Semtech.Buffer[2]=(uint8_t) temp[1];;
			Semtech.Buffer[3]=(uint8_t) (temp[1]>>8);
			Semtech.Buffer[4]=(uint8_t) temp[2];
			Semtech.Buffer[5]=(uint8_t)( temp[2]>>8);
// 			Semtech.Buffer[6]=(uint8_t) temp[3];
// 			Semtech.Buffer[7]=(uint8_t)( temp[3]>>8);
				
			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			for (unsigned int i=0;i<6;i++)
// 			{
// 				usart_write(USART_SERIAL,&Semtech.Buffer[i]);
// 			}

			data[0]=1;
			data[2]=0;
			
		
			/* Send data to Matlab */
 			if(xQueueSend(Queue_RF_Task,&Semtech,100))	//pdPASS=1-
 			{
 						
 			}

			
	}
}

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
 