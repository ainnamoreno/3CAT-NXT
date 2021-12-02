#include <comms.h>

#include <radio.h>
#include <sx126x.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

//#include "sx126x-hal.h"

static RadioEvents_t RadioEvents;	//SHOULD THIS BE IN MAIN??? IS TO HANDLE IRQ???

//HOW TO USE GLOBAL VARIABLE NOT-STATIC???? => IT DO NOT COMPILE
uint32_t air_time;
uint8_t Buffer[BUFFER_SIZE];
//uint64_t BufferWindow[WINDOW_SIZE];	//If there is RAM memory problems => delete
uint8_t count_packet;	//To count how many packets have been sent (maximum WINDOW_SIZE)
uint8_t count_window;	//To count the window number
uint64_t ack;	//Information rx in the ACK (FER DESPLAÇAMENTS DSBM)
bool nack;	//True when retransmition necessary

//uint8_t Buffer[BUFFER_SIZE];
//bool PacketReceived = false;
//bool RxTimeoutTimerIrqFlag = false;

void configuration(void){
	/*
	//i) Initialize power in the antenna and the transceiver => OBC??????????????
	if( SX126xGetOperatingMode() != MODE_STDBY_RC ) //ii) If we are not in standby, change state to standby
	{
		SX126xSetStandby(MODE_STDBY_RC);
	}
	SX126xSetPacketType(PACKET_TYPE_LORA); 	 //iii) Set the packet type to LoRa
	SX126xSetRfFrequency(RF_FREQUENCY); //iv) Set the RF frequency
	//SX126xSetPaConfig(0x04, 0x07, 0x00, 0x01); // Optimal config. for SX1262 at 22 dBm. No fa falta crec, es fa dins de SetTxParams
	//SX126xSetPaSelect(); dins de SetTxParams hi ha un if que depen d'aixo, pero no trobo aquesta funció a cap lloc
	SX126xSetTxParams(TX_OUTPUT_POWER, RADIO_RAMP_200_US); //v) Set SX1262 TX parameters (power, ramp time) -- RAMP TIME he posat el que hi ha a radio.c, no tinc ni idea de si cal canviar-ho
	SX126xSetBufferBaseAddress(0x00,0x88); //vi) Set buffer address (Tx and Rx)
	SX126xSetModulationParams(); //vii) Set modulation parameters
	SX126xSetPacketParams(); //viii) Set packet parameters

	//PLEASE, revise the following line (obtained from exampled web)
	SX126xSetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_RADIO_NONE, IRQ_RADIO_NONE ); //ix) Set
	*/
	Radio.Init( &RadioEvents );	//SHOULD THIS BE IN MAIN???

	Radio.SetChannel( RF_FREQUENCY );

	Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
								   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
								   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
								   true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE );	//In the original example it was 3000


	//SHALL WE CARE ABOUT THE RX TIMEOUT VALUE??? IF YES, CHANGE IT IN SetRx FUNCTION
	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
								   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
								   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
								   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );



	//Air time calculus
	air_time = Radio.TimeOnAir( MODEM_LORA , PACKET_LENGTH );

	count_packet = 0;
	count_window = 0;
	ack = 0xFFFFFFFFFFFFFFFF;
	nack = false;

};

//CAD: CHANNEL ACTIVITY DETECTED

void tx_function(void){
	//configuration();
	packaging(); //Start the TX by packaging all the data that will be transmitted
	//SX126xSetPayload(); //Aquesta fa el writebuffer, sha de posar direccions com a la pag 48 del datasheet

    Radio.Send( Buffer, BUFFER_SIZE );

};

void rx_function(void){
	Radio.Rx( RX_TIMEOUT_VALUE );

};

void packaging(void){
	//NACK packets at the beginnig of the next window

	//i) If nack = true	=>

	//ii)

};

