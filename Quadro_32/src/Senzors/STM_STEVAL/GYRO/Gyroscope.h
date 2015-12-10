/*
 * Gyroscope.h
 *
 * Created: 8/29/2013 7:08:20 PM
 *  Author: U2
 */


#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#define	GYRO_ADRESS				0b1101011

#define L3G_WHO_AM_I			0x0F
#define L3G_CTRL_REG1			0x20
#define L3G_CTRL_REG2			0x21
#define L3G_CTRL_REG3			0x22
#define L3G_CTRL_REG4			0x23
#define L3G_CTRL_REG5			0x24
#define L3G_OUT_TEMP			0x26
#define L3G_STATUS_REG			0x27
#define L3G_OUT_X_L				0x28
#define L3G_OUT_X_H				0x29
#define L3G_OUT_Y_L				0x2A
#define L3G_OUT_Y_H				0x2B
#define L3G_OUT_Z_L				0x2C
#define L3G_OUT_Z_H				0x2D
#define L3G_FIFO_CTRL_REG		0x2E
#define L3G_FIFO_SRC_REG		0x2F
#define L3G_INT1_CFG			0x30
#define L3G_INT1_SRC			0x31
#define L3G_INT1_TSH_XH			0x32
#define L3G_INT1_TSH_XL			0x33
#define L3G_INT1_TSH_YH			0x34
#define L3G_INT1_TSH_YL			0x35
#define L3G_INT1_TSH_ZH			0x36
#define L3G_INT1_TSH_ZL			0x37
#define L3G_INT1_DURATION		0x38

#define FS_250DPS	0<<4
#define FS_500DPS	1<<4
#define FS_2000DPS	2<<4

#define HP_ENABLE	1<<4

 typedef struct{
	 unsigned char CTRL_1;
	 unsigned char CTRL_2;
	 unsigned char CTRL_3;
	 unsigned char CTRL_4;
	 unsigned char CTRL_5;
	 unsigned char CTRL_6;

 } MY_GYRO_CONF_STRUCT;


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
	 //float Temp;
 }GYRO_XYZ;


void Gyro_init(void);
void Gyro_send(unsigned char Adress, unsigned char *Data, unsigned char Length);
void Gyro_read(unsigned char Adress, unsigned char *Data , unsigned char Length);
void Gyro_get_g(GYRO_XYZ *XYZ);
void computeGyroTCBias(void);
void initGyro(void);
void computeGyroRTBias(void);




#endif /* GYROSCOPE_H_ */