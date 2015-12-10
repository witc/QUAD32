/*
 * RF_Task.c
 *
 * Created: 14.2.2014 16:41:00
 *  Author: JR
 */ 


#include <asf.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "RF_Task.h"
#include "main.h"

#include "sx1276-Hal.h"
#include "sx1276.h"
#include "sx1276-Fchp.h"
#include "sx1276-LoRa.h"
#include "sx1276-Fsk.h"
#include "Extern_init.h"

/* Define a type of modulation*/
#define LORA	1

#define RSSI_OFFSET                                 -137.0
#define NOISE_ABSOLUTE_ZERO                         -174.0
#define NOISE_FIGURE                                6.0

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile		xSemaphoreHandle	Lights_RF_Busy;
extern xTaskHandle		Sx1276_id;
 

uint8_t pole[]={1,2,3,4,5,6};

// Default settings
extern tLoRaSettings LoRaSettings;
extern tSX1276LR SX1276LR;
extern tSX1276 SX1276;



/****************************************************************************/
void RX_done_LR(RF_Queue *Semtech,short *crc)
{	
	static short RxPacketSize = 0;
	static int8_t RxPacketSnrEstimate;
	static double RxPacketRssiValue;
	static char RxGain = 1;
	static long RxTimeoutTimer = 0;
	static char RFBuffer[RF_BUFFER_SIZE];
	
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		
	SX1276Read( REG_LR_IRQFLAGS, &SX1276LR.RegIrqFlags );
	
	if( ( SX1276LR.RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
	{
		// Clear Irq
		 SX1276Write( REG_LR_IRQFLAGS,RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK  );
		gpio_set_pin_high(LED0);
		//Semtech->State = RFLR_STATE_RX_INIT;
		*crc=CRC_FALSE;
		//Semtech->Rssi=SX1276LoRaReadRssi();
				
	}
	else if ((SX1276LR.RegIrqFlags & RFLR_IRQFLAGS_RXDONE ) == RFLR_IRQFLAGS_RXDONE)
	{
		*crc=CRC_OK;
		SX1276Write( REG_LR_IRQFLAGS,RFLR_IRQFLAGS_RXDONE_MASK );
		gpio_toggle_pin(LED0);
				
		RxPacketRssiValue=SX1276LoRaReadRssi();
		
		if( LoRaSettings.RxSingleOn == true ) // Rx single mode
		{	
		
			SX1276LR.RegFifoAddrPtr =SX1276LR.RegFifoRxBaseAddr;;
			SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );

			if( LoRaSettings.ImplicitHeaderOn == true )
			{	
				RxPacketSize = SX1276LR.RegPayloadLength;
				SX1276ReadFifo( RFBuffer,RxPacketSize); //SX1276LR->RegPayloadLength 
			}
			else
			{
				SX1276Read( REG_LR_NBRXBYTES, &SX1276LR.RegNbRxBytes );	//Nuber of recieved bytes
				RxPacketSize = SX1276LR.RegNbRxBytes;
				SX1276ReadFifo( RFBuffer, RxPacketSize); //
			}
		}
		else // Rx continuous mode
		{	
			SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR.RegFifoRxCurrentAddr );

			if( LoRaSettings.ImplicitHeaderOn == true )
			{
				RxPacketSize = SX1276LR.RegPayloadLength;
				SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoRxCurrentAddr - SX1276LR.RegPayloadLength;
				SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
				SX1276ReadFifo( RFBuffer, SX1276LR.RegPayloadLength );
			}
			else
			{
				SX1276Read( REG_LR_NBRXBYTES, &SX1276LR.RegNbRxBytes );
				RxPacketSize = SX1276LR.RegNbRxBytes;
				SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoRxCurrentAddr - SX1276LR.RegNbRxBytes;
				SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
				SX1276ReadFifo( RFBuffer, SX1276LR.RegNbRxBytes );
			}
		}
	
	
		RFBuffer[RxPacketSize+1]=0;
		
		Semtech->Rssi=RxPacketRssiValue;
		
	
	//Clear all irqs in SEMTECH
	}else
	{
		SX1276Write( REG_LR_IRQFLAGS, 0xFF );
		//Semtech->State = RFLR_STATE_RX_INIT;
	//	RxPacketSize = SX1276LR.RegPayloadLength;
	//	SX1276ReadFifo( RFBuffer,RxPacketSize); //SX1276LR->RegPayloadLength 
			
	}
	
	SX1276Read(0xC,&Semtech->AGC);
	Semtech->AGC>>=5;
		
	for (char i=0;i<255;i++)
	{
		Semtech->Buffer[i]=RFBuffer[i];
	}
	
	Semtech->Stat.Data_State=RFLR_STATE_RX_INIT;
	Semtech->Stat.Cmd=STAY_IN_STATE;
	if(xQueueSend(Queue_RF_Task,&Semtech,5000)!=pdPASS)
	{
		
	}
	
//	NVIC_EnableIRQ(GPS_IRQ);
	
	
}
/************************************************************************/

void Start_RX_LR(void)
{	
	RF_Queue Semtech;
	
	static uint8_t RFBuffer[RF_BUFFER_SIZE];
	//RF_STAT Semtech;
	
	/* Clear all Flags IRQ */
	SX1276Write( REG_LR_IRQFLAGS, 0xFF );


	SX1276LR.RegIrqFlagsMask = 	
	RFLR_IRQFLAGS_RXTIMEOUT |
	//RFLR_IRQFLAGS_RXDONE|
	//RFLR_IRQFLAGS_PAYLOADCRCERROR |
	RFLR_IRQFLAGS_VALIDHEADER |
	RFLR_IRQFLAGS_TXDONE |
	RFLR_IRQFLAGS_CADDONE |
	RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
	RFLR_IRQFLAGS_CADDETECTED;
	 
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR.RegIrqFlagsMask );

	SX1276LR.RegHopPeriod = 0;	//Datasheet says 0 ... nebo stara verze 255?
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR.RegHopPeriod );
	 								// RxDone                    RxTimeout                   FhssChangeChannel           CadDone
	SX1276LR.RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00;// | 
								//RFLR_DIOMAPPING1_DIO1_00;// | 	//RXTimeout
								// RFLR_DIOMAPPING1_DIO2_00 | 
								// RFLR_DIOMAPPING1_DIO3_10; //CRC error
								// CadDetected               ModeReady
	SX1276LR.RegDioMapping2 =0;// RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR.RegDioMapping1, 2 );

// 	 // see errata note
 	//SX1276Write( 0x2F, 0x14 );
	SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength,&SX1276LR );	//
	 
	 if( LoRaSettings.RxSingleOn == true ) // Rx single mode
	 {
		 SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
	 }
	 else // Rx continuous mode
	 {
		 SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoRxBaseAddr;
		 SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
		 
		 SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
	 }
	 
	
// 	 PacketTimeout = LoRaSettings.RxPacketTimeout;
// 	 RxTimeoutTimer = TickCounter;//GET_TICK_COUNT( );
	
		
}

/************************************************************************/
void Send_data_LR(uint8_t *data,uint8_t Length)
{
	uint8_t Temp=0;
	uint16_t Timeout=2600;	// u kzadeho oboju musi byt jinak ?!?!?
				
	// see errata note
	//SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
	
// 	SX1276Read(REG_LR_IRQFLAGSMASK,&Temp);
 //	SX1276Write( REG_LR_IRQFLAGS, 0xFF  );
	
	SX1276LR.RegIrqFlagsMask =
	RFLR_IRQFLAGS_RXTIMEOUT |
	RFLR_IRQFLAGS_RXDONE |
	RFLR_IRQFLAGS_PAYLOADCRCERROR |
	RFLR_IRQFLAGS_VALIDHEADER |
	//RFLR_IRQFLAGS_TXDONE |
	RFLR_IRQFLAGS_CADDONE |
	RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL
	|RFLR_IRQFLAGS_CADDETECTED
	;
	
	SX1276LR.RegHopPeriod = 0;
	
	SX1276Write(REG_LR_HOPPERIOD, SX1276LR.RegHopPeriod );	//0x1C
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR.RegIrqFlagsMask );

	// Initializes the payload size
	SX1276LR.RegPayloadLength = Length;
	SX1276Write(REG_LR_PAYLOADLENGTH, SX1276LR.RegPayloadLength );	//0x17
	
	SX1276LR.RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
	SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR.RegFifoTxBaseAddr );

	SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoTxBaseAddr;
	SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
	
	// Write payload buffer to LORA modem
	SX1276WriteFifo(data,Length);
	
	///TX done						//CAD DONE							//Detected CAD
	SX1276LR.RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01;// | RFLR_DIOMAPPING1_DIO0_10 |RFLR_DIOMAPPING1_DIO1_10 ;//| RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
	// PllLock              Mode Ready
	SX1276LR.RegDioMapping2 = 0;//RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR.RegDioMapping1, 2 );

	
	SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );
	

}

/**************************************************************************/
void Send_data_FSK(uint8_t *data,uint8_t TxPacketSize)
{	
	uint8_t DataChunkSize =0;
  // Packet DIO mapping setup
  //                           PacketSent,               FifoLevel,              FifoFull,               TxReady
  SX1276.RegDioMapping1 = RF_DIOMAPPING1_DIO0_00;// | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_00 | RF_DIOMAPPING1_DIO3_01;
  //                           LowBat,                   Data
  SX1276.RegDioMapping2 =0;// RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_10;
  SX1276WriteBuffer( REG_DIOMAPPING1, &SX1276.RegDioMapping1, 2 );

  SX1276.RegFifoThresh = RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY;// | 0x18; // 24 bytes of data
  SX1276Write( REG_FIFOTHRESH, SX1276.RegFifoThresh );
	
  delay_ms(1);
  SX1276FskSetOpMode( RF_OPMODE_TRANSMITTER );
 
 // if( DIO3 == 1 )    // TxReady
	//delay_ms(800);
	
// 	if( ( SX1276.RegPacketConfig1 & RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) == RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE )
// 	{
// 		SX1276WriteFifo( ( uint8_t* )&TxPacketSize, 1 );
// 	}
	
	if( ( TxPacketSize > 0 ) && ( TxPacketSize <= 64 ) )
	{
		DataChunkSize = TxPacketSize;
	}
	else
	{
		DataChunkSize = 32;
	}
	
	SX1276WriteFifo( data, DataChunkSize );
	
	
}
/**************************************************************************/

uint8_t Check_status(char Line)
{
	uint8_t Temp;
	RF_Queue Semtech;
		
		
		if (Line==0)
		{
			//SX1276Write( 0x12, RFLR_IRQFLAGS_RXDONE_MASK);
			return	RFLR_STATE_TX_DONE;
			 
		}else if(Line==1)
		{
			 SX1276Write( 0x12, RFLR_IRQFLAGS_RXTIMEOUT_MASK);//
			 return	RFLR_STATE_RX_TIMEOUT;
			 
		}else if(Line==2)
		{
			SX1276Write( 0x12,RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK);
			return RFLR_STATE_CRC_ERROR;
		}else
		{
		 	//SX1276Write( 0x12, 0);
		 	return 0x66;
		}
	
}

void Rf_mode(RF_Queue *Sem_in)
{	
	RF_Queue Semtech;
	short Valid_packet=0;
	static uint8_t TX_READY=1;

	switch (Sem_in->Stat.Data_State)
	{
		
		case RFLR_STATE_IDLE:
			

			break;
		
		case RFLR_STATE_ERROR:
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;			
			xQueueSend(Queue_RF_Task,&Semtech,3000);
		
			break;
		
		case RFLR_STATE_RX_INIT:
			#ifdef LORA
				Start_RX_LR();
				
			#else
				Start_RX_FSK();
			#endif
			
			break;
		
		case RFLR_STATE_RX_RUNNING:
		
			break;
		
		case RFLR_STATE_RX_DONE:
			
			#ifdef LORA
					RX_done_LR(&Semtech,&Valid_packet);
			#else
					RX_done_FSK(&Semtech,&Valid_packet);
			#endif
			
			if (Valid_packet==CRC_OK)
			{
											
			}
		
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;
			if(xQueueSend(Queue_RF_Task,&Semtech,10000)!=pdPASS)
			{
				
			}
		
			break;
		
		case RFLR_STATE_RX_TIMEOUT:
		
			Semtech.Stat.Cmd=STAY_IN_STATE;
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			if(xQueueSend(Queue_RF_Task,&Semtech,10000)!=pdPASS)
			{
				
			}
			
			break;
		
		case RFLR_STATE_CRC_ERROR:	//nepoužito
		
			Semtech.Stat.Cmd=STAY_IN_STATE;
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			if(xQueueSend(Queue_RF_Task,&Semtech,10000)!=pdPASS)
			{
				
			}
			break;
		
		case RFLR_STATE_TX_INIT:
			
			#ifdef LORA
				Send_data_LR(Sem_in->Buffer,LoRaSettings.PayloadLength);
			
			#elif FSK
				Send_data_FSK(Sem_in->Buffer,6);
				
			#else
				#error "NO LORA or FSK Defined"
				
			#endif
			
			vTaskSuspend(Sx1276_id);
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );	
			// optimize the power consumption by switching off the transmitter as soon as the packet has been sent
			SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY);
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
			//SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP);
			
// 			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
// 			Semtech.Buffer[0]=1;
// 			Semtech.Buffer[1]=2;
// 			Semtech.Buffer[2]=3;
// 			Semtech.Buffer[3]=4;
// 			Semtech.Buffer[4]=5;
// 			Semtech.Buffer[5]=6;
// 			if(xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY)!=pdPASS)
// 			{
// 				
// 			}
			
			//Run task again
			//vTaskResume(Sx1276_id);
			
			break;
		
		case RFLR_STATE_TX_RUNNING:
		
			break;
		
		case RFLR_STATE_TX_DONE:
		//	TX_READY=1;
// 			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
// 			if(xQueueSend(Queue_RF_Task,&Semtech,10000)!=pdPASS)
// 			{
// 				
// 			}
			
			break;
			
		
		
		default:
			break;
	}
}

/**************************************************************************/
void RF_Task(void *pvParameters)
{
	RF_Queue Semtech;
	MANAGER_TASK Manage_data;
	
//  	Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
//  	Semtech.Stat.Cmd=STAY_IN_STATE;
//  	xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY);
 	
	portTickType LastWakeTime;
	LastWakeTime=xTaskGetTickCount();
	 
	taskENTER_CRITICAL();
	SX1276Init();
 	delay_ms(1);
	taskEXIT_CRITICAL();
	
	eic_setup();		
// 	vSemaphoreCreateBinary(Lights_RF_Busy);
// 	xSemaphoreTake(Lights_RF_Busy,0);
	cpu_irq_enable();
	Semtech.Stat.Cmd=STAY_IN_STATE;
	Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
	
	/* Fill a buffer */
	Semtech.Buffer[0]=0xAA;
	Semtech.Buffer[1]=0xAB;
	Semtech.Buffer[1]=0xAB;
	Semtech.Buffer[2]=0xAD;
	Semtech.Buffer[3]=0xAE;
	Semtech.Buffer[5]=0xAF;
	
// 	uint8_t Pole[10];
// 	Semtech.Buffer=Pole;
		//xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY);
	for (;;)
	{	
// 		Semtech.Stat.Cmd=STAY_IN_STATE;
// 		Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
// 		
// 		//Semtech.Buffer="1234567890";
// 		xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY);
		
		//delay_ms(2000);
// 		gpio_toggle_pin(LED0_GPIO);
// 		vTaskDelay(500/portTICK_RATE_MS);
		
		
  		if(xQueueReceive(Queue_RF_Task,&Semtech,portMAX_DELAY)==pdPASS)
		{
								
			if (Semtech.Stat.Cmd==CHANGE_STATE)
			{
				if (Semtech.Stat.Data_State==STATE_OFF)
				{
					SX1276Write( REG_LR_IRQFLAGS, 0xFF );
					SX1276LoRaSetOpMode(RFLR_OPMODE_SLEEP);
// 					Manage_data.Task=RF_i;
// 					Manage_data.State_RDY=RDY_TO_SLEEP;
// 					xQueueSend(Queue_Manage,&Manage_data,portMAX_DELAY);
					//xSemaphoreGive(Lights_Distance);
				}
				else if(Semtech.Stat.Data_State==STATE_ON)
				{
					taskENTER_CRITICAL();
					SX1276Init();
					delay_ms(1);
					taskEXIT_CRITICAL();
				}
				
			}
			else if(Semtech.Stat.Cmd==STAY_IN_STATE)
			{
				Rf_mode(&Semtech);
			}
		
		
		//fronta prazdna po xx ticich
		}else
		{
// 			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
// 			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			if(xQueueSend(Queue_RF_Task,&Semtech,5000)!=pdPASS)
// 			{
// 				
// 			}
		}
		
	
	}
}

/*
 *Mutex:

 Is a key to a toilet. One person can have the key - occupy the toilet - at the time. When finished, the person gives (frees) the key to the next person in the queue.

 Officially: "Mutexes are typically used to serialise access to a section of re-entrant code that cannot be executed concurrently by more than one thread. A mutex object only allows one thread into a controlled section, forcing other threads which attempt to gain access to that section to wait until the first thread has exited from that section." Ref: Symbian Developer Library

 (A mutex is really a semaphore with value 1.)

 Semaphore:

 Is the number of free identical toilet keys. Example, say we have four toilets with identical locks and keys. The semaphore count - the count of keys - is set to 4 at beginning (all four toilets are free), then the count value is decremented as people are coming in. If all toilets are full, ie. there are no free keys left, the semaphore count is 0. Now, when eq. one person leaves the toilet, semaphore is increased to 1 (one free key), and given to the next person in the queue.

 Officially: "A semaphore restricts the number of simultaneous users of a shared resource up to a maximum number. Threads can request access to the resource (decrementing the semaphore), and can signal that they have finished using the resource (incrementing the semaphore)." Ref: Symbian Developer Library
 */