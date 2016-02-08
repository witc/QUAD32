

#include <asf.h>
#include <pio.h>
#include "main.h"
#include "RF_Task.h"
#include "Senzor_Task.h"
#include "Extern_init.h"
#include "string.h"

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);
void System_TimerCallback(xTimerHandle pxTimer);


xTaskHandle		Sx1276_id;
xTaskHandle		Senzor_id;

volatile	xQueueHandle		Queue_RF_Task;
volatile	xQueueHandle		Queue_Senzor_Task;

volatile xTimerHandle MPU_Timer;
volatile xTimerHandle System_Timer;

/*NIRQ0 - From RF Semtech - RX Done*/
void Semtech_IRQ0(void)
{
	//RF_Queue	Semtech;
	
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	
	
	//Semtech.Stat.Cmd=STAY_IN_STATE;
	//Semtech.Stat.Data_State=
	Check_status(0);
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
	
	if(xQueueSendFromISR(Queue_RF_Task,&Semtech,(long)3000)!=pdPASS);
	{
		
	}
	
}

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,

		.charlength = 8,
		.paritytype = CONF_UART_PARITY,

		.stopbits = 1,

	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	pio_configure_pin_group(CONF_UART_PIO, CONF_PINS_UART,
			CONF_PINS_UART_FLAGS);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	
		usart_write_line((Usart*)UART_SERIAL,"pokus");
		
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
	configure_console();

		
//  	static usart_serial_options_t usart_options = {
//  		.baudrate = USART_SERIAL_BAUDRATE,
//  		.charlength = USART_SERIAL_CHAR_LENGTH,
//  		.paritytype = USART_SERIAL_PARITY,
//  		.stopbits = USART_SERIAL_STOP_BIT
//  	};
//  	usart_serial_init(USART_SERIAL, &usart_options);
// 		
	
	/* Initialize trace library before using any FreeRTOS APIs if enabled */
	//Trace_Init();
	/* Start tracing */
	//uiTraceStart();
	
	Queue_RF_Task=xQueueCreate(3,sizeof(RF_Queue));
	
#if (RAW_MPU9150==1)

#elif ((RAW_INT_MPU9150==1))
	Queue_Senzor_Task=xQueueCreate(8,sizeof(MPU9150_Queue));	
#elif (FIFO_MPU9150==1)
	Queue_Senzor_Task=xQueueCreate(2,sizeof(MPU9150_Queue));
	MPU_Timer=xTimerCreate("Timer_MPU",(20/portTICK_RATE_MS),pdTRUE,0,MPU_TimerCallback);
#else
# error "Please specifyWay to get a datta from MPU9150"
#endif
	
	
	System_Timer=xTimerCreate("Timer_MPU",(20/portTICK_RATE_MS),pdTRUE,0,System_TimerCallback);
	if(xTimerStart(System_Timer,0)!=pdPASS){}
		
	/*Create Compass Task*/
	xTaskCreate(Senzor_Task,(const signed char * const) "Senzor",configMINIMAL_STACK_SIZE+500,NULL, 1,&Senzor_id);	
	/*Create Semtech Task*/
	xTaskCreate(RF_Task,(const signed char * const) "Sx1276",configMINIMAL_STACK_SIZE+300,NULL, 1,&Sx1276_id);
		
	
	vTaskStartScheduler();
		
	while (1) {
		
	}
}

void System_TimerCallback(xTimerHandle pxTimer)
{	
	static uint8_t temp=0;
	
	temp++;
	if (temp>15)
	{
		ioport_set_pin_level(LEDG,true);
		temp=0;
	}else
	{
		ioport_set_pin_level(LEDG,false);
		
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
//     char pole[100];
// 		
// 	for (short i=0;i<strlen(pcTaskName);i++)
// 	{
// 		pole[i]=pcTaskName[i];
// 	}
	
	   	
	__ASM volatile("BKPT #01");	
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
	
    for (;; ) {}
}		


 void HardFault_Handler(void)
 {	
//  	static char zprava1[80];
//  	static char zprava2[80];
//  		
//  	sprintf(zprava1, "SCB->HFSR = 0x%08x\n", SCB->HFSR);
//  	//nastal Hardfault
//  	if ((SCB->HFSR & (1 << 30)) != 0) {
//  	
//  	sprintf(zprava2, "SCB->CFSR = 0x%08x\n", SCB->CFSR );
 	
 
 //}	
 __ASM volatile("BKPT #01");
 	while(1);
 }