/*
 * AKce.c
 *
 * Created: 8/25/2013 2:58:45 PM
 *  Author: U2
 */

#include <asf.h>
#include <arm_math.h>
#include "Akce.h"
#include "Senzor_Task.h"
#include "Akce-Hal.h"
#include "LSM303DLHC.h"

float accelRTBias[3] = { 0.0f, 0.0f, 0.0f };
	
/********************************************************/
void Akce_init()
{
	MY_AKCE_CONF_STRUCT Akce_set;

	 Akce_set.CTRL_1=(7<<4)|(7<<0); // 400Hz + X,y,Z enabled
	 Akce_set.CTRL_2=00; //filtry + fifo
	 Akce_set.CTRL_3=0;	//preruseni
	 Akce_set.CTRL_4=(FS_2G<<4)|(HR_ENABLE<<0);//|(1<<4); // Hig reolution  + citlivost +-2g
	 Akce_set.CTRL_5=0;	//fifo en...
	 Akce_set.CTRL_6=0;

	delay_ms(10);
	Akce_send(CTRL_REG1_A,&Akce_set.CTRL_1,1);
	Akce_send(CTRL_REG2_A,&Akce_set.CTRL_2,1);
	Akce_send(CTRL_REG3_A,&Akce_set.CTRL_3,1);
	Akce_send(CTRL_REG4_A,&Akce_set.CTRL_4,1);
	Akce_send(CTRL_REG5_A,&Akce_set.CTRL_5,1);
	Akce_send(CTRL_REG6_A,&Akce_set.CTRL_6,1);
	

}

/**********************************************************/
void Akce_get_g(AKCE_XYZ *XYZ)
{

	//Akce_read(OUT_X_L_A,&XYZ,6);
	uint8_t Temp=0;
//  	do 
//  	{
//  		Akce_read(0x27,&Temp,1);
//  		
//  	} while ((Temp&0x8)!=0x8);	//Data ready

	//Akce_read(OUT_X_L_A,XYZ,6);
 	Akce_read(OUT_X_L_A,&XYZ->X_L,1);
 	Akce_read(OUT_X_H_A,&XYZ->X_H,1);
// 
// 
 	Akce_read(OUT_Y_L_A,&XYZ->Y_L,1);
 	Akce_read(OUT_Y_H_A,&XYZ->Y_H,1);
// 	
 	Akce_read(OUT_Z_L_A,&XYZ->Z_L,1);
 	Akce_read(OUT_Z_H_A,&XYZ->Z_H,1);
	
}


///////////////////////////////////////////////////////////////////////////////
// Compute Accel Runtime Bias
///////////////////////////////////////////////////////////////////////////////

void computeAccelRTBias(void)
{
	uint16_t samples;
	short NO_OF_Samples = 2000;
	short buffer[3];
	float accelSum[3] = { 0, 0, 0 };
	
	uint8_t ctrlx[2];
	float LSM_Acc_Sensitivity;
	
	/* Read the register content */
	Lsm303dlhcAccI2CBufferRead(ctrlx, LSM_A_CTRL4_REG_ADDR,2);


	if(ctrlx[1]&0x40){
		/* FIFO mode */
		LSM_Acc_Sensitivity = 0.25;
	}
	else
	{
		/* normal mode */
		/* switch the sensitivity value set in the CRTL4*/
		switch(ctrlx[0] & 0x30)
		{
			case LSM_FS_2G:
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
			break;
			case LSM_FS_4G:
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
			break;
			case LSM_FS_8G:
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
			break;
			case LSM_FS_16G:
			LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
			break;
		}
	}


	for (samples = 0; samples <  NO_OF_Samples; samples++) {
		/* Read the raw data */
		Lsm303dlhcAccReadRawData(buffer);
		accelSum[XAXIS] += buffer[XAXIS];
		accelSum[YAXIS] += buffer[YAXIS];
		accelSum[ZAXIS] += buffer[ZAXIS];

		//delay_ms(1);
	}

	accelRTBias[XAXIS] = accelSum[XAXIS] /  NO_OF_Samples;
	accelRTBias[YAXIS] = accelSum[YAXIS] /  NO_OF_Samples;
	accelRTBias[ZAXIS] = (accelSum[ZAXIS] /  NO_OF_Samples) - (9.8056f / fabs(LSM_Acc_Sensitivity));

	
}

///////////////////////////////////////////////////////////////////////////////
// Accel Initialization
///////////////////////////////////////////////////////////////////////////////

void initAccel(void)
{	
	
	LSMAccInit LSMAccInitStructure;
	LSMAccFilterInit LSMAccFilterInitStructure;
	
	LSMAccInitStructure.xPowerMode = LSM_NORMAL_MODE;
	LSMAccInitStructure.xOutputDataRate = LSM_ODR_400_HZ;
	LSMAccInitStructure.xEnabledAxes= LSM_ALL_AXES_EN;
	LSMAccInitStructure.xFullScale = LSM_FS_4G;
	LSMAccInitStructure.xDataUpdate = LSM_CONTINUOS_UPDATE;
	LSMAccInitStructure.xEndianness=LSM_BIG_ENDIAN;
	LSMAccInitStructure.xHighResolution=LSM_ENABLE;

	/* Fill the accelerometer LPF structure */
	LSMAccFilterInitStructure.xHPF=LSM_DISABLE;
	LSMAccFilterInitStructure.xHPF_Mode=LSM_HPFM_NORMAL;
	LSMAccFilterInitStructure.cHPFReference=0x00;
	LSMAccFilterInitStructure.xHPFCutOff=LSM_HPCF_16;
	LSMAccFilterInitStructure.xHPFClick=LSM_DISABLE;
	LSMAccFilterInitStructure.xHPFAOI2=LSM_DISABLE;
	LSMAccFilterInitStructure.xHPFAOI1=LSM_DISABLE;
	
	
	/* Configure the accelerometer main parameters */
	Lsm303dlhcAccConfig(&LSMAccInitStructure);

	/* Configure the accelerometer LPF main parameters */
	Lsm303dlhcAccFilterConfig(&LSMAccFilterInitStructure);
		
	computeAccelRTBias();
	/*
	accelRTBias[XAXIS] = 0.0f;
	accelRTBias[YAXIS] = 0.0f;
	accelRTBias[ZAXIS] = 0.0f;
	*/
}