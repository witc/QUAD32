/*
 * Extern_init.h
 *
 * Created: 27.1.2014 21:44:15
 *  Author: JR
 */ 


#ifndef EXTERN_INIT_H_
#define EXTERN_INIT_H_

void twi_init(void);
void spi_init(void);
void eic_setup(void);
void Semtech_IRQ1(void);
void Semtech_IRQ0(void); //external inerrupt
void Semtech_IRQ2(void);


//NIRQ - SEMTECH RF
#define  RF_NIRQ0_LINE		3	//as EXTINT3 ? => 3
#define  RF_NIRQ1_LINE		2	//as EXTINT2 ? => 2


//GPS
/** USART Interface */
#define CONF_UART              USART1
/** Baudrate setting */
#define CONF_UART_BAUDRATE     9600
/** Character length setting */
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT
/** Parity setting */
#define CONF_UART_PARITY       US_MR_PAR_NO
/** Stop bits setting */
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT

#define PDCA_GPS_Done			USART1_Handler
#define GPS_IRQ                 USART1_IRQn
#define RX_BUFF					0x1000
#define ALL_INTERRUPT_MASK  0xffffffff

#define GPS_RX_USART          1	//Musi byt 1 - pro USART RX

#endif /* EXTERN_INIT_H_ */