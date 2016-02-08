/*
 * Comp.h
 *
 * Created: 16.7.2014 16:05:59
 *  Author: JR
 */ 


#ifndef COMP_H_
#define COMP_H_


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
	
	float fHeadingx;
	float fHeadingy;
	float fHeadingz;
	
	short X_Offset;
	short Y_Offset;
	short Z_Offset;
	
	 double Nasobek1;
	 double Nasobek2;
	 
}MAG_XYZ;

void Mag_get_b(MAG_XYZ *XYZ);
void Calibrate_Comp(MAG_XYZ * COMP);




#endif /* COMP_H_ */