/*
 * Akce.h
 *
 * Created: 8/25/2013 2:58:56 PM
 *  Author: U2
 */ 


#ifndef AKCE_H_
#define AKCE_H_

 
#define FS_2G	0
#define FS_4G	1
#define FS_8G	2
#define FS_16G	3

#define HR_ENABLE	1
#define HR_DISABLE	0


 
 typedef struct{
	 unsigned char CTRL_1;
	 unsigned char CTRL_2;
	 unsigned char CTRL_3;
	 unsigned char CTRL_4;
	 unsigned char CTRL_5;
	 unsigned char CTRL_6;
	
 } MY_AKCE_CONF_STRUCT;
 
 
 typedef struct{
	 unsigned char X_L;
	 unsigned char X_H;
	 unsigned char Y_L;
	 unsigned char Y_H;
	 unsigned char Z_L;
	 unsigned char Z_H; 
	short X;
	short Y;
	short Z;
	
	float f_X;
	float f_Y;
	float f_Z;
	
	 short X_Offset;
	 short Y_Offset;
	 short Z_Offset;
	 
	 double Nasobek1;
	 double Nasobek2;
	 
	 float	fHead_x;
	 float	fHead_y;
	 
 }AKCE_XYZ;


void Akce_init(void);
void Akce_get_g(AKCE_XYZ *XYZ);
void computeAccelRTBias(void);
void initAccel(void);

#endif /* AKCE_H_ */