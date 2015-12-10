/*
 * Akce_hal.c
 *
 * Created: 16.7.2014 15:57:16
 *  Author: JR
  * Hardware Abstraction Layer - Hal
 */ 


#include <asf.h>
#include "Akce-Hal.h"

#define LIS_ADRESS				0x1E

void Akce_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{

	twi_package_t packet_write = {
		.chip         = AKCE_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (bytes)

	};

	while(twi_master_write(TWI0, &packet_write) != TWI_SUCCESS);
	//twi_master_write(&TWIC, &packet_write);

}
/*********************************************************/

void Akce_read(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	//taskENTER_CRITICAL();
	
	twi_package_t packet_read = {
		.chip         = AKCE_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (bytes)

	};
	// Perform a multi-byte read access then check the result.
	//twi_master_read(TWI0, &packet_read);
	twi_master_read(TWI0, &packet_read);
	//twi
	
	//taskEXIT_CRITICAL();
	
}
/********************************************************/

void Lsm303dlhcAccI2CByteRead_f(unsigned char *Data,unsigned char Adress)
{

	//taskENTER_CRITICAL();

	twi_package_t packet_read = {
		.chip         = AKCE_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = 1  // transfer data size (bytes)

	};
	// Perform a multi-byte read access then check the result.
	//twi_master_read(TWI0, &packet_read);
	twi_master_read(TWI0, &packet_read);
	//twi

}


void Lsm303dlhcAccI2CByteWrite_f(unsigned char *Data,unsigned char Adress)
{
	twi_package_t packet_write = {
		.chip         = AKCE_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = 1  // transfer data size (bytes)

	};

	while(twi_master_write(TWI0, &packet_write) != TWI_SUCCESS);
}


void Lsm303dlhcAccI2CBufferRead_f(unsigned char *Data,unsigned char Adress, unsigned char size)
{
	//taskENTER_CRITICAL();
	
	for (unsigned char i=0;i<size;i++)
	{
	
		twi_package_t packet_read = {
			.chip         = AKCE_ADRESS,                        // TWI slave bus address
			.addr         = (Adress+i),								// TWI slave memory address data
			.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
			.buffer       = &Data[i],                        // transfer data destination buffer
			.length       = 1  // transfer data size (bytes)

		};
		// Perform a multi-byte read access then check the result.
		//twi_master_read(TWI0, &packet_read);
		twi_master_read(TWI0, &packet_read);
	}
}



void Lsm303dlhcAccI2CBufferWrite_f(unsigned char *Data,unsigned char Adress, unsigned char size)
{
	//taskENTER_CRITICAL();
	
	for (unsigned char i=0;i<size;i++)
	{
		
		twi_package_t packet_read = {
			.chip         = AKCE_ADRESS,                        // TWI slave bus address
			.addr         = (Adress+i),								// TWI slave memory address data
			.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
			.buffer       = &Data[i],                        // transfer data destination buffer
			.length       = 1  // transfer data size (bytes)

		};
		// Perform a multi-byte read access then check the result.
		//twi_master_read(TWI0, &packet_read);
		twi_master_write(TWI0, &packet_read);
	}
}



/*****************************************************************************/
			
		

void Lsm303dlhcMagI2CByteRead_f(unsigned char *Data,unsigned char Adress)
{

	//taskENTER_CRITICAL();

	twi_package_t packet_read = {
		.chip         = LIS_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = 1  // transfer data size (bytes)

	};
	// Perform a multi-byte read access then check the result.
	//twi_master_read(TWI0, &packet_read);
	twi_master_read(TWI0, &packet_read);
	//twi

}


void Lsm303dlhcMagI2CByteWrite_f(unsigned char *Data,unsigned char Adress)
{
	twi_package_t packet_write = {
		.chip         = LIS_ADRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = 1  // transfer data size (bytes)

	};

	while(twi_master_write(TWI0, &packet_write) != TWI_SUCCESS);
}


void Lsm303dlhcMagI2CBufferRead_f(unsigned char *Data,unsigned char Adress, unsigned char size)
{
	//taskENTER_CRITICAL();
	
	for (unsigned char i=0;i<size;i++)
	{
		
		twi_package_t packet_read = {
			.chip         = LIS_ADRESS,                        // TWI slave bus address
			.addr         = (Adress+i),								// TWI slave memory address data
			.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
			.buffer       = &Data[i],                        // transfer data destination buffer
			.length       = 1  // transfer data size (bytes)

		};
		// Perform a multi-byte read access then check the result.
		//twi_master_read(TWI0, &packet_read);
		twi_master_read(TWI0, &packet_read);
	}
}



void Lsm303dlhcMagI2CBufferWrite_f(unsigned char *Data,unsigned char Adress, unsigned char size)
{
	//taskENTER_CRITICAL();
	
	for (unsigned char i=0;i<size;i++)
	{
		
		twi_package_t packet_read = {
			.chip         = LIS_ADRESS,                        // TWI slave bus address
			.addr         = (Adress+i),								// TWI slave memory address data
			.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
			.buffer       = &Data[i],                        // transfer data destination buffer
			.length       = 1  // transfer data size (bytes)

		};
		// Perform a multi-byte read access then check the result.
		//twi_master_read(TWI0, &packet_read);
		twi_master_write(TWI0, &packet_read);
	}
}