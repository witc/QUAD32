

#include <asf.h>
#include <pio.h>
#include "main.h"
#include "RF_Task.h"
#include "Senzor_Task.h"
#include "Extern_init.h"
#include "string.h"
xTaskHandle		Sx1276_id;
xTaskHandle		Senzor_id;

volatile	xQueueHandle		Queue_RF_Task;
volatile	xQueueHandle		Queue_Senzor_Task;

volatile xTimerHandle MPU_Timer;

/*NIRQ0 - From RF Semtech - RX Done*/
void Semtech_IRQ0(void)
{
	RF_Queue	Semtech;
	
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	
	
	Semtech.Stat.Cmd=STAY_IN_STATE;
	Semtech.Stat.Data_State=Check_status(0);
	xTaskResumeFromISR(Sx1276_id);
// 	if(xQueueSendFromISR(Queue_RF_Task,&Semtech,3000)!=pdPASS);
// 	{
// 		
// 	}
	
}

/*NIRQ1 - From RF Semtech - Timeout*/
void Semtech_IRQ1(void)
{
	RF_Queue	Semtech;
	
	NVIC_ClearPendingIRQ(PIOA_IRQn);

	Semtech.Stat.Cmd=STAY_IN_STATE;
	Semtech.Stat.Data_State=Check_status(1);
	
	if(xQueueSendFromISR(Queue_RF_Task,&Semtech,3000)!=pdPASS);
	{
		
	}
	
	
}

/*NIRQ1 - From RF Semtech - CR error*/
void Semtech_IRQ2(void)
{
	RF_Queue	Semtech;
	
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	
	Semtech.Stat.Cmd=STAY_IN_STATE;
	Semtech.Stat.Data_State=Check_status(2);
	
	if(xQueueSendFromISR(Queue_RF_Task,&Semtech,3000)!=pdPASS);
	{
		
	}
	
}

/**
 * \brief Configure USART in normal (serial rs232) mode, asynchronous,
 * 8 bits, 1 stop bit, no parity, 115200 bauds and enable its transmitter
 * and receiver.
 */
static void configure_usart(void)
{
	const sam_usart_opt_t usart_console_settings = {
		BOARD_USART_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		/* This field is only used in IrDA mode. */
		0
	};

	/* Enable the peripheral clock in the PMC. */
	sysclk_enable_peripheral_clock(BOARD_ID_USART);

	/* Configure USART in serial mode. */
	usart_init_rs232(BOARD_USART, &usart_console_settings,
			sysclk_get_cpu_hz());

	/* Disable all the interrupts. */
	usart_disable_interrupt(BOARD_USART, ALL_INTERRUPT_MASK);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(BOARD_USART);
	

	
}


/**
 *  Configure UART for debug message output.
 */
// static void configure_console(void)
// {
// 	const usart_serial_options_t uart_serial_options = {
// 		.baudrate = CONF_UART_BAUDRATE,
// 		.paritytype = CONF_UART_PARITY
// 	};
// 
// 	/* Configure console UART. */
// 	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
// 	stdio_serial_init(CONF_UART, &uart_serial_options);
// }


int main (void)
{	
	sysclk_init();
	board_init();
	
	/* Configure UART for debug message output. */
	//configure_console();

	/* Output example information. */
	//puts(STRING_HEADER);

	/* Configure USART. */
	//configure_usart();

		
//  	static usart_serial_options_t usart_options = {
//  		.baudrate = USART_SERIAL_BAUDRATE,
//  		.charlength = USART_SERIAL_CHAR_LENGTH,
//  		.paritytype = USART_SERIAL_PARITY,
//  		.stopbits = USART_SERIAL_STOP_BIT
//  	};
//  	usart_serial_init(USART_SERIAL, &usart_options);
// 		

	/* Create timer */
	
	Queue_RF_Task=xQueueCreate(3,sizeof(RF_Queue));
	
#if (RAW_MPU9150==1)

#elif ((RAW_INT_MPU9150==1))
	Queue_Senzor_Task=xQueueCreate(10,sizeof(MPU9150_Queue));	
#elif (FIFO_MPU9150==1)
	Queue_Senzor_Task=xQueueCreate(2,sizeof(MPU9150_Queue));
	
#else
# error "Please specifyWay to get a datta from MPU9150"
#endif
	
	/*Create Compass Task*/
	xTaskCreate(Senzor_Task,"Senzor",configMINIMAL_STACK_SIZE+400,NULL, 1,&Senzor_id);	
	/*Create Semtech Task*/
	xTaskCreate(RF_Task,"sx1276",configMINIMAL_STACK_SIZE+400,NULL, 1,&Sx1276_id);
	
	
	
	vTaskStartScheduler();
		
	while (1) {
		
	}
}


//	if No task to execute	*/
void vApplicationIdleHook()
{	
	
	while(1){

		
	}
}
	
	
/* FreeRTOS stack overflow hook */
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
    
	(void) pxTask;
   // (void) pcTaskName;
    char pole[100];
		
	for (short i=0;i<strlen(pcTaskName);i++)
	{
		pole[i]=pcTaskName[i];
	}
	
	   	
	__ASM volatile("BKPT #01");	
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
	
    for (;; ) {}
}		


 void HardFault_Handler(void)
 {	
 	static char zprava1[80];
 	static char zprava2[80];
 		
 	sprintf(zprava1, "SCB->HFSR = 0x%08x\n", SCB->HFSR);
 	//nastal Hardfault
 	if ((SCB->HFSR & (1 << 30)) != 0) {
 	
 	sprintf(zprava2, "SCB->CFSR = 0x%08x\n", SCB->CFSR );
 	
 
 }	__ASM volatile("BKPT #01");
 	while(1);
 }