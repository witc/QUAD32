/**
 * \file
 *
 * \brief SAM4S Xplained Pro board configuration
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

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

//Semtech - RF
#  define SX1276_CS_PIN          PIO_PA11_IDX // OUT
#  define SX1276_SCK_PIN         PIO_PA14_IDX // OUT
#  define SX1276_MOSI_PIN        PIO_PA13_IDX // OUT
#  define SX1276_MISO_PIN	     PIO_PA12_IDX // OUT
#  define SX1276_RESET_PIN       PIO_PA23_IDX // OUT
#  define SX1276_RxTx_PIN        PIO_PA19_IDX // OUT
#  define SX1276_NIRQ0_PIN       PIO_PA18_IDX // IN
#  define SX1276_NIRQ1_PIN       PIO_PA17_IDX // IN

/*  Compass */
#define I2C_SCL_PIN				PIO_PA4_IDX
#define I2C_SDA_PIN				PIO_PA3_IDX
#define EXT1_TWI_SDA_MUX		1
#define EXT1_TWI_SCL_MUX        1

//Measure Periode
#define PERIODE_PIN				PIO_PA17_IDX

#define USART_TXD_PIN            IOPORT_CREATE_PIN(PIOA, 6)

#endif /* CONF_BOARD_H */
