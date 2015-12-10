/*
 * Akce_Hal.h
 *
 * Created: 16.7.2014 15:59:41
 *  Author: JR
 * Hardware Abstraction Layer - Hal
 
 */ 


#ifndef AKCE_HAL_H_
#define AKCE_HAL_H_

void Akce_send(unsigned char Adress, unsigned char *Data, unsigned char Length);
void Akce_read(unsigned char Adress, unsigned char *Data , unsigned char Length);
void Lsm303dlhcAccI2CByteRead_f(unsigned char *Data,unsigned char Adress);
void Lsm303dlhcAccI2CByteWrite_f(unsigned char *Data,unsigned char Adress);
void Lsm303dlhcAccI2CBufferRead_f(unsigned char *Data,unsigned char Adress, unsigned char size);
void Lsm303dlhcAccI2CBufferWrite_f(unsigned char *Data,unsigned char Adress, unsigned char size);

void Lsm303dlhcMAgI2CByteRead_f(unsigned char *Data,unsigned char Adress);
void Lsm303dlhcMAgI2CByteWrite_f(unsigned char *Data,unsigned char Adress);
void Lsm303dlhcMagI2CBufferRead_f(unsigned char *Data,unsigned char Adress, unsigned char size);
void Lsm303dlhcMagI2CBufferWrite_f(unsigned char *Data,unsigned char Adress, unsigned char size);


#define	AKCE_ADRESS				0x19

#define    CTRL_REG1_A				0x20
#define	CTRL_REG2_A				0x21
#define	CTRL_REG3_A				0x22
#define	CTRL_REG4_A				0x23
#define	CTRL_REG5_A				0x24
#define	CTRL_REG6_A				0x25
#define	REFERENCE_A				0x26
#define	STATUS_REG_A			0x27

#define	OUT_X_L_A				0x28
#define	OUT_X_H_A				0x29

#define	OUT_Y_L_A				0x2A
#define	OUT_Y_H_A				0x2B

#define	OUT_Z_L_A				0x2C
#define	OUT_Z_H_A				0x2D

#define	FIFO_CTRL_REG_A			0x2E
#define	FIFO_SRC_REG_A			0x2F
#define	INT1_CFG_A				0x30
#define	INT1_SOURCE_A			0x31
#define	INT1_THS_A				0x32
#define	INT1_DURATION_A			0x33
#define	INT2_CFG_A				0x34

#define	INT2_SOURCE_A			0x35
#endif /* AKCE-HAL_H_ */