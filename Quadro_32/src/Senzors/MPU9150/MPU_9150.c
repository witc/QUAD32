/*
 * MPU_9150.c
 *
 * Created: 02.12.2015 21:57:32
 *  Author: uzivatel
 */ 

#include <asf.h>
#include "MPU_9150.h"
#include "MPU_9150_HAL.h"
#include "L3gx.h"

/* Hardware registers needed by driver. */
struct gyro_reg_s {
	unsigned char who_am_i;
	unsigned char rate_div;
	unsigned char lpf;
	unsigned char prod_id;
	unsigned char user_ctrl;
	unsigned char fifo_en;
	unsigned char gyro_cfg;
	unsigned char accel_cfg;
	unsigned char accel_cfg2;
	unsigned char lp_accel_odr;
	unsigned char motion_thr;
	unsigned char motion_dur;
	unsigned char fifo_count_h;
	unsigned char fifo_r_w;
	unsigned char raw_gyro;
	unsigned char raw_accel;
	unsigned char temp;
	unsigned char int_enable;
	unsigned char dmp_int_status;
	unsigned char int_status;
	unsigned char accel_intel;
	unsigned char pwr_mgmt_1;
	unsigned char pwr_mgmt_2;
	unsigned char int_pin_cfg;
	unsigned char mem_r_w;
	unsigned char accel_offs;
	unsigned char i2c_mst;
	unsigned char bank_sel;
	unsigned char mem_start_addr;
	unsigned char prgm_start_h;
	#if defined AK89xx_SECONDARY
	unsigned char s0_addr;
	unsigned char s0_reg;
	unsigned char s0_ctrl;
	unsigned char s1_addr;
	unsigned char s1_reg;
	unsigned char s1_ctrl;
	unsigned char s4_ctrl;
	unsigned char s0_do;
	unsigned char s1_do;
	unsigned char i2c_delay_ctrl;
	unsigned char raw_compass;
	/* The I2C_MST_VDDIO bit is in this register. */
	unsigned char yg_offs_tc;
	#endif
};



/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 4g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize()
{	
	
	unsigned char data[6];
	
    /* Reset device. */
    data[0] = BIT_RESET;
	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 );
	
	vTaskDelay(30/portTICK_RATE_MS);
	
	while(!MPU6050_TestConnection())
	{
		/*MPU6050_GetDeviceID();*/
	}
	
	
	 /* Wake up chip. */
	 data[0] = 0x00;
	 MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 );
	 
	/*set clock */
	 MPU6050_SetClockSource(MPU6050_CLOCK_PLL_ZGYRO );	// 
	 
	/*set LPF and Fs to 8khz - internal */
	data[0]=4;	/* 4=21Hz cut off*/
	MPU6050_I2C_ByteWrite(10,&data[0], MPU6050_RA_CONFIG);
	  
	 /* set sample rate */
	data[0]=1;
	MPU6050_I2C_ByteWrite(10,data,MPU6050_RA_SMPLRT_DIV); // 1khz / (1 + 3) =
			
	/* Set citlivost */
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_4);
	
// 	 /* Wake up chip. */
 	 data[0] = 0x00;
 	 MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 );
// 	 	   	
	/* Fifo enable + FIFO reset */
 	data[0] =0x44;
 	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_USER_CTRL );
	
	/* Enable Fifo, GYRO, Temp and Acc*/
 	data[0]=0xF8;
 	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_FIFO_EN);

	/* Data ready interrupts enabled*/
//  	data[0]=0x10;
//  	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_INT_ENABLE);
	
	
	/* Fifo reset */
	data[0]=0x4;	
 	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_USER_CTRL );
 	
	
// 	while (temp!=0)
// 	{
// 		FIFO_MPU[i++] =(((short)buffer[0]) << 8) | buffer[1];
// 		FIFO_MPU[i++] =(((short)buffer[2]) << 8) | buffer[3];
// 		FIFO_MPU[i++] =(((short)buffer[4]) << 8) | buffer[5];
// 		temp-=6;
// 	}
	
// 	/* Fifo enable + FIFO reset */
// 	data[0] =0x44;
// 	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_USER_CTRL );
// 	
// 	/* Enable Fifo, GYRO */
// 	data[0]=0xF8;
// 	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_FIFO_EN);
// 
// 	/* Data ready interrupts enabled*/
// 	data[0]=0x1;
// 	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_INT_ENABLE);
// setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection()
{
	return MPU6050_GetDeviceID() == 0x34 ? true : false; //0b110100; 8-bit representation in hex = 0x34
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct), but after masking 0x34
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp;
}


/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus()
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    return tmp == 0x00 ? false : true;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(uint8_t NewState)
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccelGyro(short* AccelGyro)
{
    uint8_t tmpBuffer[14];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((short) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((short) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
     tmp &= mask;
     tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    *data = tmp & (1 << bitNum);
}



/**
 * @brief  Writes one byte to the  MPU6050.
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
{
    // ENTR_CRT_SECTION();
	
	MPU_9150_send(writeAddr,pBuffer,1);

}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
void MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, short NumByteToRead)
{
    // ENTR_CRT_SECTION();
 //	for (short i=0;i<NumByteToRead;i++)
 //	{
		MPU_9150_read(readAddr,&pBuffer[0],NumByteToRead);	
	//}
	
    
}
/**
 * @}
 *//* end of group MPU6050_Library */
 
 
void MPU9150_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
 {

    //get accel and gyro
    //MPU9150_getMotion6(ax, ay, az, gx, gy, gz);

    //read mag
//     MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
//    // delay_ms(10);
//     MPU6050_I2C_ByteWrite(MPU6050_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
//      //delay_ms(10);
//      MPU6050_I2C_BufferRead(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer);
//     *mx = (((int16_t)buffer[0]) << 8) | buffer[1];
//     *my = (((int16_t)buffer[2]) << 8) | buffer[3];
//     *mz = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9150_RA_ACCEL_XOUT_H
 */
void MPU9150_getMotion6(short* ax, short* ay, short* az, short* gx, short* gy, short* gz,short *offset) 
{	
	uint8_t buffer[14];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_ACCEL_XOUT_H, 12);
    *ax = (((short)buffer[0]) << 8) | buffer[1];
    *ay = (((short)buffer[2]) << 8) | buffer[3];
    *az = (((short)buffer[4]) << 8) | buffer[5];
    *gx = ((((short)buffer[8]) << 8) | buffer[9])-offset[0];
    *gy = ((((short)buffer[10]) << 8) | buffer[11])-offset[1];
    *gz = ((((short)buffer[12]) << 8) | buffer[13])-offset[2];
}

/* FIFO DATA */
short MPU9150_getMotion6_fifo(short* FIFO_MPU,short *offset)
{	
	
	short temp;
//	static uint8_t buffer[1024];
	short i=0;
	
// 	buffer[0]=0;
// 	MPU6050_I2C_ByteWrite(10,buffer,MPU6050_RA_FIFO_EN);
	/* read count of Fifo */
	MPU6050_I2C_BufferRead(10,FIFO_MPU,MPU6050_RA_FIFO_COUNTH,2);
	temp=(((short)FIFO_MPU[0]) << 8) | FIFO_MPU[1];
 	
	  /* Wake up chip. */
	  FIFO_MPU[0] = 0x00;
	  MPU6050_I2C_ByteWrite(10,&FIFO_MPU[0],MPU6050_RA_PWR_MGMT_1 );
	  
	MPU6050_I2C_BufferRead(10,FIFO_MPU,MPU6050_RA_FIFO_R_W,temp);
	
// 	while (temp!=0)
// 	{
// 		 FIFO_MPU[i++] =(((short)buffer[0]) << 8) | buffer[1];
// 		 FIFO_MPU[i++] =(((short)buffer[2]) << 8) | buffer[3];
// 		 FIFO_MPU[i++] =(((short)buffer[4]) << 8) | buffer[5];
// 		 temp-=6;
// 	}
		 
		
// 		 
//  		FIFO_MPU[i++] = (((short)buffer[0]) << 8) | buffer[1];
//  		FIFO_MPU[i++] = (((short)buffer[2]) << 8) | buffer[3];
//  		FIFO_MPU[i++] = (((short)buffer[4]) << 8) | buffer[5];
//  		FIFO_MPU[i++]= ((((short)buffer[8]) << 8) | buffer[9])-offset[0];
//  		FIFO_MPU[i++] = ((((short)buffer[10]) << 8) | buffer[11])-offset[1];
//  		FIFO_MPU[i++] = ((((short)buffer[12]) << 8) | buffer[13])-offset[2
 	
	//	FIFO_MPU[i]=0; /* ukonèení*/
	return temp;
}


/************************************************************************/
/* Compute Tempreature bias Gyro                                        */
/************************************************************************/
void MPU9150_Gyro_Tempr_Bias(short *offset)
{
	float Sum[4];	//32 bits
	static short Temp;
	static short Temperature;
	static short Pre_Temp=0;
	uint8_t buffer[8];
	#define NO_OF_SAMPLES 1000
	
	for (short i=0;i<NO_OF_SAMPLES;i++)
	{
		MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_GYRO_XOUT_H  , 6);  
// 		  
// 		Temp=(short)((buffer[0] << 8) | buffer[1]);
// 		Sum[3]+=(float)Temp;
		Temp=(short)((buffer[0] << 8) | buffer[1]);
		Sum[0]+=(float)Temp;
		Temp=(short)((buffer[2] << 8) | buffer[3]);
		Sum[1]+=(float)Temp;
		Temp=(short)((buffer[4] << 8) | buffer[5]);
		Sum[2]+=(float)Temp;
		delay_ms(2);
	}
	
	/* Temp */
//	Temperature=(short)(((Sum[3]/NO_OF_SAMPLES)/340)+36.53);
	/*X,Y,Z gyro offset */
	offset[0]=(short)(Sum[0]/NO_OF_SAMPLES);
	offset[1]=(short)(Sum[1]/NO_OF_SAMPLES);
	offset[2]=(short)(Sum[2]/NO_OF_SAMPLES);
	
	
	/* waiting for incriasing tempreature */
	//delay_ms(10000);
		
// 	for (short i=0;i<NO_OF_SAMPLES;i++)
// 	{
// 		MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_TEMP_OUT_H , 8);
// 		
// 		Temp=(short)((buffer[0] << 8) | buffer[1]);
// 		Sum[3]+=(float)Temp;
// 		Temp=(short)((buffer[2] << 8) | buffer[3]);
// 		Sum[0]+=(float)Temp;
// 		Temp=(short)((buffer[4] << 8) | buffer[5]);
// 		Sum[1]+=(float)Temp;
// 		Temp=(short)((buffer[6] << 8) | buffer[7]);
// 		Sum[2]+=(float)Temp;
// 		
// 	}
// 	
	/* Temp */
	Temperature=(short)((Sum[3]/NO_OF_SAMPLES)/340+36.53);
// 	/*X,Y,Z gyro offset */
// 	offset[0]=(short)(Sum[0]/NO_OF_SAMPLES);
// 	offset[1]=(short)(Sum[1]/NO_OF_SAMPLES);
// 	offset[2]=(short)(Sum[2]/NO_OF_SAMPLES	
// 	/* Read actual temprreature */
// 	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_TEMP_OUT_H , 2);    
// 	Temp=(((short)buffer[0]) << 8) | buffer[1];
     
}


// setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!


