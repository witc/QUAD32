/**
 * \file
 *
 * \brief SAM4S Xplained Pro board initialization
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <board.h>
#include <gpio.h>
#include <ioport.h>
#include <wdt.h>

/**
 * \addtogroup sam4s_xplained_pro_group
 * @{
 */

void board_init(void)
{
#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
	wdt_disable(WDT);
#endif

	/* GPIO has been deprecated, the old code just keeps it for compatibility.
	 * In new designs IOPORT is used instead.
	 * Here IOPORT must be initialized for others to use before setting up IO.
	 */
	ioport_init();

	//Compass
	ioport_set_pin_dir(I2C_SCL_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(I2C_SDA_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(I2C_SCL_PIN,false);
	ioport_set_pin_level(I2C_SDA_PIN,false);
	delay_ms(15);
	gpio_configure_pin(I2C_SCL_PIN, TWI0_CLK_FLAGS);
	gpio_configure_pin(I2C_SDA_PIN, TWI0_DATA_FLAGS);
	
	/* Measure pins */
	ioport_set_pin_dir(PERIODE_PIN,1);
	ioport_set_pin_dir(PERIODE_PIN_INT,1);
	
	/* usart*/
	ioport_set_pin_dir(USART_TXD_PIN, IOPORT_DIR_OUTPUT);
	
	/* LED output */
	ioport_set_pin_dir(LEDW,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LEDR,IOPORT_DIR_OUTPUT);
//	ioport_set_pin_dir(LEDG,IOPORT_DIR_OUTPUT);
	
	
	/* motors PWM*/	
 	///ioport_set_pin_dir(PWM_M1, IOPORT_DIR_OUTPUT);

	//(PIO_OUTPUT_1 | PIO_DEFAULT)
	
// 	pio_configure_pin(PWM_M2,PIO_TYPE_PIO_PERIPH_A);
// 	pio_configure_pin(PWM_M3,PIO_TYPE_PIO_PERIPH_A);
// 	pio_configure_pin(PWM_M4,PIO_TYPE_PIO_PERIPH_B);
// 	
// 	ioport_set_pin_dir(PWM_M1, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(PWM_M2, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(PWM_M3, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(PWM_M4, IOPORT_DIR_OUTPUT);
}

/** @} */
