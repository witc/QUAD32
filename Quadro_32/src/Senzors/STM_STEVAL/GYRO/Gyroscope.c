/*
 * Gyro.c
 *
 * Created: 8/25/2013 2:58:45 PM
 *  Author: U2
 */

#include <asf.h>
#include "Gyroscope.h"
#include "gyroTempCalibration.h"
#include "Akce.h"
#include "L3Gx.h"
#include "Senzor_Task.h"

uint8_t gyroCalibrating;

float gyroRTBias[3];

float gyroTemperature;
/********************************************************/
void Gyro_init()
{
	MY_GYRO_CONF_STRUCT Gyro_set;

	 Gyro_set.CTRL_1=0x0F; //Frequency 760 Hz + Cut off=100 +Enable axis
	 Gyro_set.CTRL_2=0x09; // HP filter
	 Gyro_set.CTRL_3=0;		//no interrupts
	 Gyro_set.CTRL_4=FS_250DPS;	//+- 250 dps
	 Gyro_set.CTRL_5=HP_ENABLE;

	uint8_t info;
	
	delay_ms(10);	 
	Gyro_send(L3G_CTRL_REG1,&Gyro_set.CTRL_1,1);
	Gyro_send(L3G_CTRL_REG2,&Gyro_set.CTRL_2,1);
	Gyro_send(L3G_CTRL_REG3,&Gyro_set.CTRL_3,1);
	Gyro_send(L3G_CTRL_REG4,&Gyro_set.CTRL_4,1);
	Gyro_send(L3G_CTRL_REG5,&Gyro_set.CTRL_5,1);
	
	Gyro_read(L3G_WHO_AM_I,&info,1);
	
	
	delay_ms(20);
}

/**********************************************************/
void Gyro_get_g(GYRO_XYZ *XYZ)
{	
	uint8_t Temp=0;
	do
	{
		Gyro_read(0x27,&Temp,1);
		
	} while ((Temp&0x8)!=0x8);	//Data ready

	//Gyro_read(L3G_OUT_X_L,XYZ,6);
	Gyro_read(L3G_OUT_X_L,&XYZ->X_L,1);
	Gyro_read(L3G_OUT_X_H,&XYZ->X_H,1);


	Gyro_read(L3G_OUT_Y_L,&XYZ->Y_L,1);
	Gyro_read(L3G_OUT_Y_H,&XYZ->Y_H,1);


	Gyro_read(L3G_OUT_Z_L,&XYZ->Z_L,1);
	Gyro_read(L3G_OUT_Z_H,&XYZ->Z_H,1);



}
/**********************************************************/

void Gyro_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{

	twi_package_t packet_write = {
		.chip         = GYRO_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length,  // transfer data size (bytes)

	};

	 while(twi_master_write(TWI0, &packet_write) != TWI_SUCCESS);
	//twi_master_write(&TWIC, &packet_write);

}
/*********************************************************/

void Gyro_read(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	taskENTER_CRITICAL();
	twi_package_t packet_read = {
		.chip         = GYRO_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length,  // transfer data size (bytes)

	};
	// Perform a multi-byte read access then check the result.
	twi_master_read(TWI0, &packet_read);
	taskEXIT_CRITICAL();
	
}

/********************************************************/
///////////////////////////////////////////////////////////////////////////////
// Compute Gyro Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeGyroTCBias(void)
{
// 	gyroTemperature = (float) (rawGyroTemperature.value + 13200) / 280.0f + 35.0f;
// 
// 	gyroTCBias[ROLL] = sensorConfig.gyroTCBiasSlope[ROLL] * gyroTemperature + sensorConfig.gyroTCBiasIntercept[ROLL];
// 	gyroTCBias[PITCH] = sensorConfig.gyroTCBiasSlope[PITCH] * gyroTemperature + sensorConfig.gyroTCBiasIntercept[PITCH];
// 	gyroTCBias[YAW] = sensorConfig.gyroTCBiasSlope[YAW] * gyroTemperature + sensorConfig.gyroTCBiasIntercept[YAW];
}


///////////////////////////////////////////////////////////////////////////////
// Compute Gyro Runtime Bias
///////////////////////////////////////////////////////////////////////////////

void computeGyroRTBias(void)
{
	uint8_t axis;
	uint16_t samples;
	short NO_OF_Samples = 1000;
	float gyroSum[3] = { 0.0f, 0.0f, 0.0f };
	short buffer[3];
	
	for (samples = 0; samples < NO_OF_Samples; samples++)
	{
		L3gxReadRawData(buffer);

		computeGyroTCBias();

		gyroSum[ROLL] += buffer[ROLL] ;//- gyroTCBias[ROLL];
		gyroSum[PITCH] += buffer[PITCH];// - gyroTCBias[PITCH];
		gyroSum[YAW] += buffer[YAW];//- gyroTCBias[YAW];

		delay_ms(1);
	}

	for (axis = ROLL; axis < 3; axis++) {
		gyroRTBias[axis] = (float) gyroSum[axis] / NO_OF_Samples;

	}

}

///////////////////////////////////////////////////////////////////////////////
// Gyro Initialization
///////////////////////////////////////////////////////////////////////////////

void initGyro(void)
{
	L3GInit L3GInitStructure;
	L3GFifoInit L3G_Fifo_Init;
	uint8_t data=0;
	
	/* Fill the gyro structure */
	L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
	L3GInitStructure.xOutputDataRate =   L3G_ODR_760_HZ_CUTOFF_30;
	L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
	L3GInitStructure.xFullScale = L3G_FS_2000_DPS;
	L3GInitStructure.xDataUpdate = L3G_BLOCK_UPDATE;
	L3GInitStructure.xEndianness = L3G_BIG_ENDIAN;

	/* Configure the gyro main parameters */
	L3gd20Config(&L3GInitStructure);
	
	delay_ms(10);
	
	/*FIfo init to Stream mode */
	L3G_Fifo_Init.cWtm=15;
	L3G_Fifo_Init.xFifoMode=L3G_STREAM_MODE;
	L3gd20FifoInit(&L3G_Fifo_Init);
	
	//HP filter set  + INT FIFO watermark */
	data=(1<<4)|(1<<0)|(1<<2)|(1<<3);
	Gyro_send(0x22,&data,1);
	
	/* Hp enable */
	Gyro_read(0x24,&data,1);
	data|=0x71;
	Gyro_send(0x24,&data,1);
	//computeGyroRTBias();
}


