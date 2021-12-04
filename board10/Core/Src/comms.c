
/*!
 * \file      comms.c
 *
 * \brief     Comms subsytem functions
 *
 *
 *
 * \code
 *
 * 				 _______    ______    ____    ____    ____    ____     ______
 * 				/ ______)  /  __  \  |    \  /    |  |    \  /    |   / _____)
 * 			   / /         | |  | |  |  \  \/  /  |  |  \  \/  /  |  ( (____
 *            ( (          | |  | |  |  |\    /|  |  |  |\    /|  |   \____ \
 *             \ \______   | |__| |  |  | \__/ |  |  |  | \__/ |  |   _____) )
 *              \_______)  \______/  |__|      |__|  |__|      |__|  (______/
 *
 *
 * \endcode
 *
 * \author    Daniel Herencia
 *
 * \author    Robert Molina
 */

#include <comms.h>

/*------IMPORTANT----------*/
/*
 * STATE CONTINGENCY ONLY RX
 * CHECK SLEEP MODE OF SX1262 WHILE NOT TX OR RX
 * MULTY THREAD
 * MATRIX OF READ-SALOMON => FADING IN SEVERAL PACKETS
 * SF AND CRC STORED IN MEMORY AND CHANGED BY TELECOMMANDS
 * send sf and crc at telemetry packet
 * THINK TELEMETRY PACKET!!!!!!!!!!
 * If stop sending in the middle of a window => set to zero count_packet[]
 */

//#include "sx126x-hal.h"

static RadioEvents_t RadioEvents;	//SHOULD THIS BE IN MAIN??? IS TO HANDLE IRQ???

uint32_t air_time;
uint8_t Buffer[BUFFER_SIZE];

/*
 * To avoid the variables being erased if a reset occurs, we have to store them in the Flash memory
 * Therefore, they have to be declared as a single-element array
 */
uint8_t count_packet[] = {0};	//To count how many packets have been sent (maximum WINDOW_SIZE)
uint8_t count_window[] = {0};	//To count the window number
uint8_t count_rtx[] = {0};		//To count the number of retransmitted packets

uint64_t ack;					//Information rx in the ACK (FER DESPLAÇAMENTS DSBM)
uint8_t nack_number;			//Number of the current packet to retransmit
bool nack;						//True when retransmition necessary
bool full_window;				//Stop & wait => to know when we reach the limit packet of the window

//uint8_t Buffer[BUFFER_SIZE];
//bool PacketReceived = false;
//bool RxTimeoutTimerIrqFlag = false;


/*
 * STATE MACHINE VARIABLES
 */
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

typedef enum
{
    CAD_FAIL,
    CAD_SUCCESS,
    PENDING,
}CadRx_t;

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

CadRx_t CadRx = CAD_FAIL;
bool PacketReceived = false;
bool RxTimeoutTimerIrqFlag = false;
int16_t RssiMoy = 0;
int8_t SnrMoy = 0;
uint16_t RxCorrectCnt = 0;
uint16_t BufferSize = BUFFER_SIZE;


TimerEvent_t CADTimeoutTimer;
TimerEvent_t RxAppTimeoutTimer;

/*
 *  ---------------- END ----------------
 */

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

	Flash_Read_Data( COMMS_VARIABLE , count_packet , sizeof(count_packet) );		//Read from Flash count_packet
	Flash_Read_Data( COMMS_VARIABLE + 0x1 , count_window , sizeof(count_window) ); 	//Read from Flash count_window
	Flash_Read_Data( COMMS_VARIABLE + 0x2 , count_rtx , sizeof(count_rtx) ); 		//Read from Flash count_rtx
	ack = 0xFFFFFFFFFFFFFFFF;
	nack = false;

};

//CAD: CHANNEL ACTIVITY DETECTED

void tx_function(void){
	//configuration();
	if (!full_window)
	{
		packaging(); //Start the TX by packaging all the data that will be transmitted
		//SX126xSetPayload(); //Aquesta fa el writebuffer, sha de posar direccions com a la pag 48 del datasheet
		Radio.Send( Buffer, BUFFER_SIZE );
	}
};

void rx_function(void){
	Radio.Rx( RX_TIMEOUT_VALUE );

};

void packaging(void){
	//NACK packets at the beginnig of the next window

	if (nack)
	{
		//Here a function to obtain the packets to retx
		nack_number = 1/*ACK*/;	//Delete the 1 and put a function in ack to obtain the number
		//Packet from last window => count_window - 1
		Flash_Read_Data( PHOTO_ADDR + (count_window[0]-1)*WINDOW_SIZE*BUFFER_SIZE + (nack_number)*BUFFER_SIZE , Buffer , sizeof(Buffer) );	//Direction in HEX
		count_rtx[0]++;
	}
	else
	{
		Flash_Read_Data( PHOTO_ADDR + count_window[0]*WINDOW_SIZE*BUFFER_SIZE + (count_packet[0]-count_rtx[0])*BUFFER_SIZE , Buffer , sizeof(Buffer) );	//Direction in HEX
		if (count_packet[0] < WINDOW_SIZE - 1)
		{
			count_packet[0]++;
		}
		else
		{
			count_packet[0] = 0;
			count_window[0]++;
			full_window = true;
		}
	}
	//ii)


};

// count_packet[] = {};	//To count how many packets have been sent (maximum WINDOW_SIZE)
// count_window[] = {};	//To count the window number

/*This function is called when a new photo is stored in the last photo position*/
void resetCommsParams(void){
	count_packet[0] = 0;
	count_window[0] = 0;
	count_rtx[0] 	= 0;
}


void stateMachine(void){
    uint16_t PacketCnt = 0;

    switch( State )
    {
        case RX_TIMEOUT:
        {
			#if(FULL_DBG)
            	printf( "RX Timeout\r\n");
			#endif
            //RxTimeoutCnt++;
            State = START_CAD;
            break;
        }
        case RX_ERROR:
        {
			#if(FULL_DBG)
            	printf( "RX Error\r\n");
			#endif
            //RxErrorCnt++;
            PacketReceived = false;
            State = START_CAD;
        break;
        }
        case RX:
        {
            if( PacketReceived == true )
            {
                PacketReceived = false;     // Reset flag
                if((Buffer[0]=='C') && (Buffer[1]=='A') && (Buffer[2]=='D'))
                {
                    PacketCnt = (Buffer[4] << 8) + Buffer[5];   // ID packet
                    RxCorrectCnt++;         // Update RX counter
					#if(FULL_DBG)
                    	printf( "Rx Packet n %d\r\n", PacketCnt );
					#endif
                }
            State = START_CAD;
            }
            else
            {
                if (CadRx == CAD_SUCCESS)
                {
                    //channelActivityDetectedCnt++;   // Update counter
					#if(FULL_DBG)
                    	printf( "Rxing\r\n");
					#endif
                    RxTimeoutTimerIrqFlag = false;
                    TimerReset(&RxAppTimeoutTimer);	// Start the Rx's's Timer
                    Radio.Rx( RX_TIMEOUT_VALUE );   // CAD is detected, Start RX
                }
                else
                {
                    TimerStart(&CADTimeoutTimer);   // Start the CAD's Timer
                }
                State = LOWPOWER;
            }
            break;
        }
        case TX:
        {
            printf("Send Packet n %d \r\n",PacketCnt);

            // Send the next frame
            Buffer[0] = 'C';
            Buffer[1] = 'A';
            Buffer[2] = 'D';
            Buffer[3] = '0';
            Buffer[4] = PacketCnt>>8;
            Buffer[5] = (uint8_t)PacketCnt ;

            if( PacketCnt == 0xFFFF)
            {
                PacketCnt = 0;
            }
            else
            {
                PacketCnt ++;
            }
            //Send Frame
            DelayMs( 1 );
            Radio.Send( Buffer, 6 );

            State = LOWPOWER;
            break;
        }
        case TX_TIMEOUT:
        {
            State = LOWPOWER;
            break;
        }
        case START_CAD:
        {
            //i++;    // Update NbTryCnt
            TimerStop(&RxAppTimeoutTimer);  // Stop the Rx's Timer
            // Trace for debug
            if(CadRx == CAD_FAIL)
            {
				#if(FULL_DBG)
            		printf("No CAD detected\r\n");
				#endif
            }
            CadRx = CAD_FAIL;           // Reset CAD flag
            DelayMs(randr(10,500));     //Add a random delay for the PER test
			#if(FULL_DBG)
            	printf("CAD %d\r\n",i);
			#endif
            Radio.StartCad( );          //StartCad Again
            State = LOWPOWER;
        break;
        }
        case LOWPOWER:
        default:
            // Set low power
            break;
    }
}




/*
 * FUNCTIONS OBTAINED FROM EXAMPLE MAIN.C
 */


void OnTxDone( void )
{
    Radio.Standby( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    PacketReceived = true;
    RssiMoy = (((RssiMoy * RxCorrectCnt) + RssiValue) / (RxCorrectCnt + 1));
    SnrMoy = (((SnrMoy * RxCorrectCnt) + SnrValue) / (RxCorrectCnt + 1));
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Standby( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Standby( );
    if( RxTimeoutTimerIrqFlag )
    {
        State = RX_TIMEOUT;
    }
    else
    {
		#if(FULL_DBG)
        	printf(".");
		#endif
        Radio.Rx( RX_TIMEOUT_VALUE );   //  Restart Rx
        //SymbTimeoutCnt++;               //  if we pass here because of Symbol Timeout
        State = LOWPOWER;
    }
}

void OnRxError( void )
{
    Radio.Standby( );
    State = RX_ERROR;
}

void OnCadDone( bool channelActivityDetected)
{
    Radio.Standby( );

    if( channelActivityDetected == true )
    {
        CadRx = CAD_SUCCESS;
    }
    else
    {
        CadRx = CAD_FAIL;
    }
    State = RX;
}

static void CADTimeoutTimeoutIrq( void )
{
    Radio.Standby( );
    State = START_CAD;
}

static void RxTimeoutTimerIrq( void )
{
    RxTimeoutTimerIrqFlag = true;
}


