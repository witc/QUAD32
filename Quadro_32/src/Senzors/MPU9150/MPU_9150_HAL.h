/*
 * MPU_9150_HAL.h
 *
 * Created: 02.12.2015 21:52:08
 *  Author: uzivatel
 */ 


#ifndef MPU_9150_HAL_H_
#define MPU_9150_HAL_H_


void MPU_9150_send(unsigned char Adress, unsigned char *Data, unsigned char Length);
void MPU_9150_read(unsigned char Adress, unsigned char *Data, unsigned char Length);



#endif /* MPU_9150_HAL_H_ */