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

/** PWM frequency in Hz */

#define PWM_FREQUENCY      500
/** Period value of PWM output waveform */
#define PERIOD_VALUE       1000
/** Initial duty cycle value */

#define INIT_DUTY_VALUE    50


#define USE_MARG_AHRS	1

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile	 xQueueHandle		Queue_Senzor_Task;
extern volatile	xQueueHandle		Queue_Motor_Task;

/* timers */
extern volatile xTimerHandle MPU_Timer;

/* offset for Gyro */
volatile short offset[3]={0,0,0};
//volatile	MPU9150_Buffer XYZ;
	
extern fourthOrderData_t fourthOrder1000Hz[6];	//MAg and accel 3 axis

void Gyro_Angle(short g_x,short g_y,short g_z,EulerAngles *uhly,float dt);
void Akce_Angle(short a_x, short a_y, short a_z,EulerAngles *uhly);


#if (FIFO_MPU9150==1)

void MPU_TimerCallback(xTimerHandle pxTimer)
{	
	//if(xTimerStopFromISR(MPU_Timer,pdPASS)!=pdPASS){};
		
	ioport_set_pin_level(PERIODE_PIN,true);
	GL_XYZ.temp=MPU9150_getMotion6_fifo(&GL_XYZ.MPU_FIFO[0]);
	//ioport_toggle_pin_level(PERIODE_PIN);
	ioport_set_pin_level(PERIODE_PIN,false);
	
   	if(xQueueSendFromISR(Queue_Senzor_Task,&GL_XYZ,pdTRUE)!=pdPASS);
   	{
   		
   	}
	
	
}

#endif

void MPU9150_INT(void)
{	
	MPU9150_Queue Data_Queue_MPU;
	//MPU9150_Buffer XYZ;
	
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	
	/*xTaskResumeFromISR(Senzor_id);*/
//	pio_disable_interrupt(PIOB, PIO_PB0);
	
 //	MPU9150_getMotion66(GL_XYZ.MPU_FIFO,offset);
	//MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,GL_XYZ.MPU_FIFO,MPU6050_RA_ACCEL_XOUT_H, 14);
	
	ioport_set_pin_level(PERIODE_PIN,true);
	//ioport_set_pin_level(PERIODE_PIN,false);
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,Data_Queue_MPU.MPU_FIFO,MPU6050_RA_ACCEL_XOUT_H, 14);
	ioport_set_pin_level(PERIODE_PIN,false);			
							
	//ioport_toggle_pin_level(PERIODE_PIN);
	Data_Queue_MPU.Vycti_data=1;
	NVIC_ClearPendingIRQ(PIOB_IRQn);	
  // 	while(xQueueSendToBackFromISR(Queue_Senzor_Task,&GL_XYZ,&xHigherPriorityTaskWoken)!=pdTRUE); //èekám dokud se data neodešlou do fronty
	xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_MPU,&xHigherPriorityTaskWoken);
   
   if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();

}

void INT_init(void)
{
	//pmc_enable_periph_clk(ID_PIOB);
	pio_set_input(PIOB, MPU9150_PIN_INT, PIO_PULLUP);
	
	pio_handler_set(PIOB, ID_PIOB, PIO_PB0, PIO_IT_RISE_EDGE, MPU9150_INT);
	
	pio_enable_interrupt(PIOB, PIO_PB0);
	
	NVIC_EnableIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
}


void Gyro_Angle(short g_x,short g_y,short g_z,EulerAngles *uhly, float dt)
{
	#define GAA			 2
	
	uint8_t			data_av=0;
	static int		Suma1=0;
	static int		Suma2=0;
	static int		Suma3=0;
	
	static short		pole1[GAA];
	static short		pole2[GAA];
	static short		pole3[GAA];
	
	static unsigned int	counter=0;
	static short		Temp1 = 0;
	static short		Temp2 = 0;
	static short		Temp3 = 0;
	GYRO_XYZ			XYZ;
	
	static float	Alfa1=0;
	static float	Alfa2=0;
	
		
// 	Suma1+=x;
// 	pole1[counter]=x;
// 	Suma2+=y;
// 	pole2[counter]=y;
// 	Suma3+=z;
// 	pole3[counter]=z;
// 	counter++;
// 	
// 	if (counter==GAA) counter=0;
// 
// 	Suma1-=pole1[counter];
// 	Temp1=(Suma1/GAA);
// 	Suma2-=pole2[counter];
// 	Temp2=(Suma2/GAA);
// 	Suma3-=pole3[counter];
// 	Temp3=(Suma3/GAA);
		
	//Temp2=(Temp2/65535)*500;
	Alfa1+=(float)(g_x*0.06103515f*dt);
	Alfa2+=(float)(g_y*0.06103515f*dt);
	
	// 		Alfa1=Temp2;
	// 		Alfa2=Temp3;
	
	uhly->pitch=(float)(Alfa1);
	uhly->roll=(float)(Alfa2);
	
	//PORTC.OUTCLR=0b1000;
	

}


void Akce_Angle(short a_x, short a_y, short a_z,EulerAngles *uhly)
{
	#define AAA		25
		
	uint8_t				data_av=0;
	static int32_t		Suma1=0;
	static int32_t		Suma2=0;
	static int32_t		Suma3=0;
	
	static int16_t		pole1[AAA];
	static int16_t		pole2[AAA];
	static int16_t		pole3[AAA];
	
	static unsigned int	counter=0;
	double		Temp1 = 0;
	double		Temp2 = 0;
	double       Temp3 = 0;
	
	
	
	Suma1+=a_x;
	pole1[counter]=a_x;
	Suma2+=a_y;
	pole2[counter]=a_y;
	Suma3+=a_z;
	pole3[counter]=a_z;
	counter++;

	if (counter==AAA) counter=0;

	Suma1-=pole1[counter];
	Temp1=(Suma1/AAA);
	Suma2-=pole2[counter];
	Temp2=(Suma2/AAA);
	Suma3-=pole3[counter];
	Temp3=(Suma3/AAA);
	
	uhly->pitch=(float)((atan2((double)Temp2,(double)Temp3)*(180)/3.141f));
	//Alfa1-=90;
// 	if (Alfa1>90)  Alfa1=90;
// 	if (Alfa1<-90) Alfa1=-90;
	
	
	//uhly->pitch=(float)(Alfa1);
	
	uhly->roll=(float)((-atan2((double)Temp1,(double)Temp3)*(180)/3.141f));
	//Alfa2-=90;
// 	if (Alfa2>90)  Alfa2=90;
// 	if (Alfa2<-90) Alfa2=-90;
	
	//uhly->pitch=(float)(Alfa2);
	
	
	
}



void Senzor_Task(void *pvParameters)
{	
	MPU9150_Queue Senzor;
	//MPU9150_Buffer XYZ;
	Motor_Queue Position;
	RF_Queue Semtech;
	
	EulerAngles Angles;
	EulerAngles Angles_A;
	EulerAngles Angles_G;
	portTickType CurrentTime;
	portTickType LastTime;
	
	#define CONST_FILTER 0.998f
		
	char Kalibrace=NULL;
	short temp[6];
	static float uhel[3];
	float f_temp[3];
	uint16_t packet_count=0;
		
 	short GyroXYZ[3];
 	short MagXYZ[3];
	short AccXYZ[3];
	
	float dt=0;
	uint8_t data[2];
	uhel[0]=0;	
	uhel[1]=0;
	uhel[2]=0;
 	//pwm_channel_enable(PWM, PWM_CHANNEL_0);

	//PWM_init();
 	
 	//pwm_channel_enable(PWM, PWM_CHANNEL_0);
	//PWM_ENA=3;
 	
	
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
		CurrentTime=xTaskGetTickCount();
		LastTime=xTaskGetTickCount();  
#elif (FIFO_MPU9150==1)
		//MPU9150_Gyro_Tempr_Bias(offset);	//-47,-7,17
		if(xTimerStart(MPU_Timer,0)!=pdPASS){}
		CurrentTime=xTaskGetTickCount();
		LastTime=xTaskGetTickCount();
#else
# error "Please specify Way to get a datta from MPU9150"
#endif
		
		offset[0]=-44;
		offset[1]=-13;
		offset[2]=22;
	
	
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
		 
		if(xQueueReceive(Queue_Senzor_Task,&Senzor,portMAX_DELAY)==pdPASS)
		{	
			if (Senzor.Vycti_data==1)
			{
				
				
				//MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,XYZ.MPU_FIFO,MPU6050_RA_ACCEL_XOUT_H, 14);

				AccXYZ[0]=(((short)(Senzor.MPU_FIFO[0]) << 8 ) | Senzor.MPU_FIFO[1]);
				AccXYZ[1]=(((short)(Senzor.MPU_FIFO[2]) << 8 ) | Senzor.MPU_FIFO[3]);
				AccXYZ[2]=(((short)(Senzor.MPU_FIFO[4]) << 8 ) | Senzor.MPU_FIFO[5]);
				
				GyroXYZ[0]=(((short)(Senzor.MPU_FIFO[8]) << 8 ) | Senzor.MPU_FIFO[9])-offset[0];
				GyroXYZ[1]=(((short)(Senzor.MPU_FIFO[10]) << 8 ) | Senzor.MPU_FIFO[11])-offset[1];
				GyroXYZ[2]=(((short)(Senzor.MPU_FIFO[12]) << 8 ) | Senzor.MPU_FIFO[13])-offset[2];
				
								
				Akce_Angle(AccXYZ[0],AccXYZ[1],AccXYZ[2],&Angles_A);
// 				LastTime=CurrentTime;
// 				CurrentTime=xTaskGetTickCount();
// 				dt=(double)((CurrentTime-LastTime)/1000);
				dt=0.001;
				uhel[0] =CONST_FILTER*(uhel[0]+(float)(GyroXYZ[0]*0.06103515f*dt)) + (1-CONST_FILTER)*Angles_A.pitch;
				uhel[1] =CONST_FILTER*(uhel[1]+(float)(GyroXYZ[1]*0.06103515f*dt)) + (1-CONST_FILTER)*Angles_A.roll;
				uhel[2] +=(float)(GyroXYZ[2]*0.06103515f*dt);
// 				
 				//uhel[2]=(float)(4*AccXYZ[0]/65535);
				//uhel[0]=uhel[0]+(AccXYZ[0]*dt);
				//uhel[1]=(float)AccXYZ[0];//uhel[1]+(uhel[0]*dt);
				
				
			}
			
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
 		
// 			for (unsigned int i=0;i<6;i++)
// 			{
// 				usart_write(USART_SERIAL,&Semtech.Buffer[i]);
// 			}

#if (RX_NEW_CMD==1)

#elif (TX_TO_MATLAB==1)
			
			temp[0]=(short)(uhel[0]);
			temp[1]=(short)(uhel[1]);
			temp[2]=(short)(uhel[2]);
			
			Semtech.Buffer[0]=(uint8_t)temp[0];	//LOW
			Semtech.Buffer[1]=(uint8_t)(temp[0]>>8);		//HIGH
			Semtech.Buffer[2]=(uint8_t) temp[1];
			Semtech.Buffer[3]=(uint8_t) (temp[1]>>8);
			Semtech.Buffer[4]=(uint8_t) temp[2];
			Semtech.Buffer[5]=(uint8_t)( temp[2]>>8);
			Semtech.Buffer[6]=(uint8_t) temp[3];
			Semtech.Buffer[7]=(uint8_t)( temp[3]>>8);
			
			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;
			/* Send data to Matlab */
			if(xQueueSend(Queue_RF_Task,&Semtech,1))	//pdPASS=1-
			{
			
			}
#else
	# error "TX or RX?"
#endif

// 						 
			/* Send new position to motor task */
			Position.pitch=uhel[0];
			Position.roll=uhel[1];
			Position.yaw=uhel[2];
				
			Position.type_of_data=FROM_SENZOR;		
			if(xQueueSend(Queue_Motor_Task,&Position,1))	//pdPASS=1-
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
 