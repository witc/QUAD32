/*
 * MPU_9150_HAL.c
 *
 * Created: 02.12.2015 21:52:00
 *  Author: uzivatel
 */ 


#define MPU_ACC_ADR

#include <asf.h>
#include "MPU_9150_HAL.h"
#include "MPU_9150.h"

void MPU_9150_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	taskENTER_CRITICAL();

	twi_package_t packet_write = {
		.chip         =  MPU6050_DEFAULT_ADDRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (bytes)

	};
	
	while(twi_master_write(TWI0, &packet_write) != TWI_SUCCESS);
	//twi_master_write(&TWIC, &packet_write);
	taskEXIT_CRITICAL();

}
/**************************************/

void MPU_9150_read(unsigned char Adress, unsigned char *Data, short Length)
{	
	taskENTER_CRITICAL();

	twi_package_t packet_read = {
		.chip         =  MPU6050_DEFAULT_ADDRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (bytes)

	};
	// Perform a multi-byte read access then check the result.
	//twi_master_read(TWI0, &packet_read);
	twi_master_read(TWI0, &packet_read);
	//twi

	taskEXIT_CRITICAL();
}

void MPU_9150_init(void)
{
	
}