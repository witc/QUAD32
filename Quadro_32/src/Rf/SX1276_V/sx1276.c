

#include <asf.h>
#include "sx1276.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "main.h"

/*!
 * SX1276 registers variable
 */
 extern tSX1276LR SX1276LR;

struct spi_device device_rf = {
	.id = 0,
};

#define CONF_ILI9341_CLOCK_SPEED   1000000UL


void SX1276Init( void )
{
  
	spi_flags_t spi_flags = SPI_MODE_0;
	//board_spi_select_id_t spi_select_id = 0;
	spi_master_init(SPI);
	//	spi_set_master_mode(CONF_ILI9341_SPI);
	spi_master_setup_device(SPI, &device_rf, spi_flags,	CONF_ILI9341_CLOCK_SPEED/4, 0);

	//spi_set_peripheral_chip_select_value(CONF_ILI9341_SPI, 1);
	spi_enable(SPI);
	
    SX1276InitIo( );
	SX1276Reset( );
	
	
	
#ifdef	LORA
	SX1276SetLoRaOn();
	SX1276LoRaInit( );
#else
	SX1276SetLoRaOFF();
	SX1276FskInit( );
#endif
    


}

void SX1276Reset( void )
{
    
    
	ioport_set_pin_dir(SX1276_RESET_PIN,IOPORT_DIR_INPUT);
	delay_ms(10);
	
	ioport_set_pin_dir(SX1276_RESET_PIN,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SX1276_RESET_PIN, false);
	delay_ms(2);
	
	ioport_set_pin_dir(SX1276_RESET_PIN,IOPORT_DIR_INPUT);
	delay_ms(6);

	
}

void SX1276SetLoRaOn(void)
{
    SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
        
    SX1276LR.RegOpMode = ( SX1276LR.RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;	
	SX1276LR.RegOpMode&=~0x8;	// vynuluju bit pro LF
    SX1276Write( REG_LR_OPMODE, SX1276LR.RegOpMode );
        
    SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
                                    // RxDone               RxTimeout                   FhssChangeChannel           CadDone
   SX1276LR.RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 |
   RFLR_DIOMAPPING1_DIO1_00 ; 	//RXTimeout
   // RFLR_DIOMAPPING1_DIO2_00 |
   // RFLR_DIOMAPPING1_DIO3_10; //CRC error
   // CadDetected               ModeReady
   SX1276LR.RegDioMapping2 =0;// RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;

                                    // CadDetected          ModeReady
    SX1276LR.RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
	
    SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR.RegDioMapping1, 2 );
   
   
}


void SX1276SetLoRaOFF(void)
{
	 SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
	 
	 SX1276LR.RegOpMode = ( SX1276LR.RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
	 SX1276Write( REG_LR_OPMODE, SX1276LR.RegOpMode );
	 
	 SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
	 
	// SX1276ReadBuffer( REG_OPMODE, SX1276Regs + 1, 0x70 - 1 );
}


void SX1276SetOpMode( uint8_t opMode )
{
     SX1276LoRaSetOpMode( opMode );
 }

uint8_t SX1276GetOpMode( void )
{
   
   return SX1276LoRaGetOpMode( );
  
}

double SX1276ReadRssi( void )
{
    return SX1276LoRaReadRssi( );
}

uint8_t SX1276ReadRxGain( void )
{
     return SX1276LoRaReadRxGain( );
    
}

uint8_t SX1276GetPacketRxGain( void )
{
    return SX1276LoRaGetPacketRxGain(  );
    
}

int8_t SX1276GetPacketSnr( void )
{
    return SX1276LoRaGetPacketSnr(  );
    
}

double SX1276GetPacketRssi( void )
{
   return SX1276LoRaGetPacketRssi( );
    
}

uint32_t SX1276GetPacketAfc( void )
{
   return 0;
}

void SX1276StartRx( void )
{
    SX1276LoRaSetRFState( RFLR_STATE_RX_INIT );
 
}

void SX1276GetRxPacket( void *buffer, uint16_t *size )
{
     SX1276LoRaGetRxPacket( buffer, size );
    
}

void SX1276SetTxPacket( const void *buffer, uint16_t size )
{
   SX1276LoRaSetTxPacket( buffer, size );
    
}

uint8_t SX1276GetRFState( void )
{
    return SX1276LoRaGetRFState( );
    
}

void SX1276SetRFState( uint8_t state )
{
   SX1276LoRaSetRFState( state );
    
}

uint32_t SX1276Process( void )
{
    return SX1276LoRaProcess( );
}

