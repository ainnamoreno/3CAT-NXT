/*
 * comms.h
 *
 *  Created on: Nov 17, 2021
 *      Author: Daniel Herencia Ruiz
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_


/*This are COMMS definitions*/
#define RF_FREQUENCY 						868000000  	// 868 MHz
#define TX_OUTPUT_POWER 					22          // 22 dBm
#define LORA_SPREADING_FACTOR 				9           // [0x09 --> SF9]
#define LORA_BANDWIDTH 						0           // Radio.h changes it	Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 }			[0x04 --> 125 kHz]
#define LORA_CODINGRATE 					1           // [0x01 --> CR=4/5]
//#define LORA_LOW_DATA_RATE_OPTIMIZE 		0      		// [0x00 --> LowDataRateOptimize OFF]
#define LORA_PREAMBLE_LENGTH				8           // CSS modulations usually have 8 preamble symbols
//#define HEADER_TYPE 						0           // [0 --> Explicit header]
//#define PAYLOAD_LENGTH 					38          // [38 bytes of payload information]
//#define CRC_TYPE 							0           // [0 --> CRC OFF]	/*ARE YOU SURE??? MAYBE 4/5 0x01*/
//#define INVERT_IQ 						0           // [0 --> Standard IQ setup]
// Check CAD
// BufferBaseAddress
// LoRaSymbNumTimeout
#define TX_TIMEOUT_VALUE 					340         // Air time Tx
#define PACKET_LENGTH 						59          // Packet Size
/*End COMMS definitions*/

#define TX_WINDOW_TIMEOUT					13900

#define LORA_SYMBOL_TIMEOUT               	0       	// ??????????
#define LORA_FIX_LENGTH_PAYLOAD_ON         	false
#define LORA_FIX_LENGTH_PAYLOAD_LEN         38			//???????????
#define LORA_IQ_INVERSION_ON  				false		//?????


#define RX_TIMEOUT_VALUE                 	0			//IF NEEDED
#define BUFFER_SIZE                         38 			// Define the payload size here
#define WINDOW_SIZE							40


/* MAYBE IT COULD BE INTERESTING TO USE A STATE MACHINE IN COMMS*/
/* OR MAYBE THIS SHOULD BE HANDLED IN MAIN, OR COMMS IS A SINGLE FUNCTION WITH STATES */
/*
typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
    START_CAD,
}States_t;
*/




void configuration(void);

void tx_function(void);

void rx_function(void);

void packaging(void);



#endif /* INC_COMMS_H_ */
