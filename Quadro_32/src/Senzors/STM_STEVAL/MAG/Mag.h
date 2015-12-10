/*
 * Comp.h
 *
 * Created: 16.7.2014 16:05:59
 *  Author: JR
 */ 


#ifndef COMP_H_
#define COMP_H_


typedef struct{
	char X_L;
	char X_H;
	char Y_L;
	char Y_H;
	char Z_L;
	char Z_H;
	
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


#define PI					3.14159265359


#endif /* COMP_H_ */