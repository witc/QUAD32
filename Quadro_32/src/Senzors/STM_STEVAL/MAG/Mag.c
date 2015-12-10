/*
 * Comp.c
 *
 * Created: 16.7.2014 16:05:41
 *  Author: JR
 */ 

#include <asf.h>
#include <math.h>
#include "Mag.h"
#include "Mag-Hal.h"
#include "Main.h"

void Mag_init(void)
{
	MY_MAG_CONF_STRUCT Mag_set;
	
	Mag_set.bCRA_REG=(7<<CRA_REG_M_OFFSET);	//frekvence - 220hz
	Mag_set.bCRB_REG=(5<<CRB_REG_M_OFFSET);	//citlivost +-4,7 gaus
	Mag_set.bMR_REG	=0;	//continous conversion
	
		delay_ms(10);
	Mag_send(ADRS_CRA_REG_M,&Mag_set.bCRA_REG,1);
	Mag_send(ADRS_CRB_REG_M,&Mag_set.bCRB_REG,1);
	Mag_send(ADRS_MR_REG_M,&Mag_set.bMR_REG,1);
	

}


/**********************************************************/

void Mag_get_b(MAG_XYZ *XYZ)
{
	
	uint8_t temp=0;
	
	do
	{
		Mag_read(SR_REG_M,&temp,1);
		
	} while ((temp&0b1)!=0b1);	//data ready
	
	
	Mag_read(OUT_XLM,&XYZ->X_L,6);
// 	Mag_read(OUT_XLM,&XYZ->X_L,1);
// 	
// 	Mag_read(OUT_YHM,&XYZ->Y_H,1);
// 	Mag_read(OUT_YLM,&XYZ->Y_L,1);
// 	
// 	Mag_read(OUT_ZHM,&XYZ->Z_H,1);
// 	Mag_read(OUT_ZLM,&XYZ->Z_L,1);

	
}

void Calibrate_Comp(MAG_XYZ * COMPAS)
 {		
	
	short MAX_X=-20000;
	short MAX_Y=-20000;
	short MAX_Z=-20000;
	
	short MIN_X=20000;
	short MIN_Y=20000;
	short MIN_Z=20000;
	 
	long long Kvadranty=0;
    LCD_Queue LCD;
	 
			 
	do 
	{
		 Mag_get_b(COMPAS);
	 
		 COMPAS->X=(short)((COMPAS->X_H<<8) | COMPAS->X_L);
		 COMPAS->Y=(short)((COMPAS->Y_H<<8) | COMPAS->Y_L);
		 COMPAS->Z=(short)((COMPAS->Z_H<<8) | COMPAS->Z_L);
		
		if (COMPAS->X==0)
   		{
   			if (COMPAS->Y>0)
   			{
   				COMPAS->fHeadingx=90;
   			}else
   			{
   				COMPAS->fHeadingx=270;
   			}
   		}
   		else
   		{	
			COMPAS->fHeadingx=(atan2f(COMPAS->Y,COMPAS->X)*(180/PI));//*(180/PI);
 		}
		
		 if (COMPAS->fHeadingx<0)
		 {
			 COMPAS->fHeadingx+=360;
		 }
		 		 		 
				 
		if (MAX_Y<COMPAS->Y)MAX_Y=COMPAS->Y;						//MIN_X=-146
		if (MAX_X<COMPAS->X)MAX_X=COMPAS->X;
		if (MAX_Z<COMPAS->Z)MAX_Z=COMPAS->Z;
	  
		if (MIN_X>COMPAS->X)MIN_X=COMPAS->X;
		if (MIN_Y>COMPAS->Y)MIN_Y=COMPAS->Y;
		if (MIN_Z>COMPAS->Z)MIN_Z=COMPAS->Z;
	  Kvadranty++;
	  delay_ms(1);
				  
	} while (Kvadranty<15000);//0b111111111111111111111111111111111111
	 
	COMPAS->Nasobek1=(double)(abs(MAX_X)+abs(MIN_X))/(abs(MAX_Y)+abs(MIN_Y));
	COMPAS->Nasobek2=(double)(abs(MAX_X)+abs(MIN_X))/(abs(MAX_Z)+abs(MIN_Z));
	
	 COMPAS->X_Offset=MIN_X+MAX_X;	//posunuti
	 COMPAS->Y_Offset=MIN_Y+MAX_Y;
	 COMPAS->Z_Offset=MIN_Z+MAX_Z;
	 
	 COMPAS->X_Offset-=(COMPAS->X_Offset)/2;
	 COMPAS->Y_Offset-=(COMPAS->Y_Offset)/2;
	 COMPAS->Z_Offset-=(COMPAS->Z_Offset)/2;
		
 }
 
 