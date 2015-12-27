/*
 * Extern_init.c
 *
 * Created: 27.1.2014 21:33:39
 *  Author: JR
 */ 

#include <asf.h>
#include <conf_board.h>
#include "Extern_init.h"
#include "Mag-Hal.h"
#include "RF_Task.h"




#define TWI_SPEED				100000//100KHZ default


/********/
void twi_init(void)
{

	vTaskDelay(10/portTICK_RATE_MS);
		
	 twi_master_options_t opt = {
		 .speed = TWI_SPEED,
		 .chip  = 0x19,
	 };
	 
	twi_master_setup(TWI0, &opt);
	vTaskDelay(10/portTICK_RATE_MS);
 //	twi_master_init(&TWI0, &opt);
 	twi_master_enable(TWI0);
	 vTaskDelay(200/portTICK_RATE_MS);
}



 void eic_setup(void)	//external inerrupt
 {	
	 
	pmc_enable_periph_clk(ID_PIOA);
	
	//pio_set_input(PIOA, PIO_PA17, PIO_PULLUP);
	pio_set_input(PIOA, PIO_PA18, PIO_PULLUP);
	//pio_set_input(PIOA, PIO_PA24, PIO_PULLUP);
	
	pio_handler_set(PIOA, ID_PIOA, PIO_PA18, PIO_IT_RISE_EDGE, Semtech_IRQ0);
	//pio_handler_set(PIOA, ID_PIOA, PIO_PA17, PIO_IT_RISE_EDGE, Semtech_IRQ1);
	//pio_handler_set(PIOA, ID_PIOA, PIO_PA24, PIO_IT_RISE_EDGE, Semtech_IRQ2);
	
	//pio_enable_interrupt(PIOA, PIO_PA17);
	pio_enable_interrupt(PIOA, PIO_PA18);
	//pio_enable_interrupt(PIOA, PIO_PA24);
	
	NVIC_EnableIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn,0);

 	
 }