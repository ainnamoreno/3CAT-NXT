/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

#include "radio.h"

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void RadioInit( RadioEvents_t *events );

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioGetStatus( void );

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioSetModem( RadioModems_t modem );

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
void RadioSetChannel( uint32_t freq );

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] freq       Channel RF frequency
 * \param [IN] rssiThresh RSSI threshold
 * \param [IN] maxCarrierSenseTime Max time while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioIsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime );

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t RadioRandom( void );

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                          uint32_t datarate, uint8_t coderate,
                          uint32_t bandwidthAfc, uint16_t preambleLen,
                          uint16_t symbTimeout, bool fixLen,
                          uint8_t payloadLen,
                          bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                          bool iqInverted, bool rxContinuous );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                          uint32_t bandwidth, uint32_t datarate,
                          uint8_t coderate, uint16_t preambleLen,
                          bool fixLen, bool crcOn, bool FreqHopOn,
                          uint8_t HopPeriod, bool iqInverted, uint32_t timeout );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool RadioCheckRfFrequency( uint32_t frequency );

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] pktLen     Packet payload length
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioTimeOnAir( RadioModems_t modem, uint8_t pktLen );

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void RadioSend( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the radio in sleep mode
 */
void RadioSleep( void );

/*!
 * \brief Sets the radio in standby mode
 */
void RadioStandby( void );

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRx( uint32_t timeout );

/*!
 * \brief Start a Channel Activity Detection
 */
void RadioStartCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioRssi( RadioModems_t modem );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void RadioWrite( uint16_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t RadioRead( uint16_t addr );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void RadioWriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void RadioReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void RadioSetPublicNetwork( bool enable );

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioGetWakeupTime( void );

/*!
 * \brief Process radio irq
 */
void RadioIrqProcess( void );

/*!
 * \brief Sets the radio in reception mode with Max LNA gain for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRxBoosted( uint32_t timeout );

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    RadioInit,
    RadioGetStatus,
    RadioSetModem,
    RadioSetChannel,
    RadioIsChannelFree,
    RadioRandom,
    RadioSetRxConfig,
    RadioSetTxConfig,
    RadioCheckRfFrequency,
    RadioTimeOnAir,
    RadioSend,
    RadioSleep,
    RadioStandby,
    RadioRx,
    RadioStartCad,
    RadioSetTxContinuousWave,
    RadioRssi,
    RadioWrite,
    RadioRead,
    RadioWriteBuffer,
    RadioReadBuffer,
    RadioSetMaxPayloadLength,
    RadioSetPublicNetwork,
    RadioGetWakeupTime,
    RadioIrqProcess,
    // Available on SX126x only
    RadioRxBoosted,
    RadioSetRxDutyCycle
};

/*
 * Local types definition
 */


 /*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};

const sx126x_lora_bw_t Bandwidths[] = { SX126X_LORA_BW_125, SX126X_LORA_BW_250, SX126X_LORA_BW_500 };

//                                          SF12    SF11    SF10    SF9    SF8    SF7
static double RadioLoRaSymbTime[3][6] = {{ 32.768, 16.384, 8.192, 4.096, 2.048, 1.024 },  // 125 KHz
                                         { 16.384, 8.192,  4.096, 2.048, 1.024, 0.512 },  // 250 KHz
                                         { 8.192,  4.096,  2.048, 1.024, 0.512, 0.256 }}; // 500 KHz

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;


sx126x_pkt_status_lora_t RadioPktStatus;
uint8_t RadioRxPayload[255];

bool IrqFired = false;

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq( void );

/*!
 * \brief Tx timeout timer callback
 */
void RadioOnTxTimeoutIrq( void );

/*!
 * \brief Rx timeout timer callback
 */
void RadioOnRxTimeoutIrq( void );

/*
 * Private global variables
 */


/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
}RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
sx126x_status_t SX126x;
sx126x_mod_params_lora_t mod_params;
sx126x_pkt_params_lora_t pkt_params;
sx126x_pkt_status_lora_t pkt_status;
sx126x_pkt_type_t pkt_type;
uint8_t buff;
sx126x_chip_status_t status;
int16_t rssi = 0;


/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
    /*uint8_t i;

    if( bandwidth == 0 )
    {
        return( 0x1F );
    }

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bw ) && ( bandwidth < FskBandwidths[i + 1].bw ) )
        {
            return FskBandwidths[i+1].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );*/
}

void RadioInit( RadioEvents_t *events )
{
    RadioEvents = events;

    SX126xInit( RadioOnDioIrq );		//CHECK THIS LINE!!!!!!!!!!!!!!!!
    sx126x_set_standby( &SX126x, SX126X_STANDBY_CFG_RC );
    sx126x_set_reg_mode( &SX126x, SX126X_REG_MODE_DCDC );

    sx126x_set_buffer_base_address( &SX126x , 0x00, 0x00 );
    sx126x_set_tx_params(&SX126x, 0, SX126X_RAMP_200_US );
    sx126x_set_dio_irq_params( &SX126x, SX126X_IRQ_ALL, SX126X_IRQ_ALL, SX126X_IRQ_NONE, SX126X_IRQ_NONE );

    // Initialize driver timeout timers
    TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
    TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

    IrqFired = false;
}

RadioState_t RadioGetStatus( void )
{
    switch( sx126x_get_status( &SX126x , &status.chip_mode) )		//CHECK THIS POINTER
    {
        case SX126X_CHIP_MODE_TX:
            return RF_TX_RUNNING;
        case SX126X_CHIP_MODE_RX:
            return RF_RX_RUNNING;
        case RF_CAD:
            return RF_CAD;
        default:
            return RF_IDLE;
    }
}

void RadioSetModem( RadioModems_t modem )
{
    switch( modem )
    {
    default:
    case MODEM_FSK:
    	break;
    case MODEM_LORA:
        sx126x_set_pkt_type( &SX126x,  SX126X_PKT_TYPE_LORA );
        // Public/Private network register is reset when switching modems
        if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous )
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork( RadioPublicNetwork.Current );
        }
        break;
    }
}

void RadioSetChannel( uint32_t freq )
{
	sx126x_set_rf_freq( &SX126x , freq );
}

bool RadioIsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    uint32_t carrierSenseTime = 0;

    RadioSetModem( modem );

    RadioSetChannel( freq );

    RadioRx( 0 );

    DelayMs( 1 );

    carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = RadioRssi( modem );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    RadioSleep( );
    return status;
}

uint32_t RadioRandom( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    RadioSetModem( MODEM_LORA );

    // Set radio in continuous reception
    sx126x_set_rx( &SX126x , 0 );

    for( i = 0; i < 32; i++ )
    {
        DelayMs( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )sx126x_get_rssi_inst( &SX126x , &rssi ) & 0x01 ) << i;
    }

    RadioSleep( );

    return rnd;
}

void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{

    RxContinuous = rxContinuous;

    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
            //SX126xSetStopRxTimerOnPreambleDetect( false );
        	sx126x_stop_timer_on_preamble( &SX126x , true );    // For long preambule, prevent rxtimeout
        	sx126x_set_lora_symb_nb_timeout( &SX126x , symbTimeout );
            pkt_type = SX126X_PKT_TYPE_LORA;
            mod_params.sf = ( sx126x_lora_sf_t )datarate;
            mod_params.bw = Bandwidths[bandwidth];
            mod_params.cr = ( sx126x_lora_cr_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                mod_params.ldro = 0x01;
            }
            else
            {
                mod_params.ldro = 0x00;
            }

            pkt_type = SX126X_PKT_TYPE_LORA;

            if( ( mod_params.sf == SX126X_LORA_SF5 ) ||
                ( mod_params.sf == SX126X_LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    pkt_params.preamble_len_in_symb = 12;
                }
                else
                {
                    pkt_params.preamble_len_in_symb = preambleLen;
                }
            }
            else
            {
                pkt_params.preamble_len_in_symb = preambleLen;
            }

            pkt_params.header_type = ( sx126x_lora_pkt_len_modes_t )fixLen;

            pkt_params.pld_len_in_bytes = MaxPayloadLength;
            pkt_params.crc_is_on = ( bool )crcOn;
            pkt_params.invert_iq_is_on = ( bool )iqInverted;

            RadioSetModem( MODEM_LORA );
            sx126x_set_lora_mod_params( &SX126x, &mod_params );
            sx126x_set_lora_pkt_params( &SX126x, &pkt_params );

            // Timeout Max, Timeout handled directly in SetRx function
            RxTimeout = 0xFFFF;

            break;
    }
}

void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{

    switch( modem )
    {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
        	pkt_type = SX126X_PKT_TYPE_LORA;
            mod_params.sf = ( sx126x_lora_sf_t ) datarate;
            mod_params.bw =  Bandwidths[bandwidth];
            mod_params.cr= ( sx126x_lora_cr_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                mod_params.ldro = 0x01;
            }
            else
            {
                mod_params.ldro = 0x00;
            }

            pkt_type = SX126X_PKT_TYPE_LORA;

            if( ( mod_params.sf == SX126X_LORA_SF5 ) ||
                ( mod_params.sf == SX126X_LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    pkt_params.preamble_len_in_symb = 12;
                }
                else
                {
                    pkt_params.preamble_len_in_symb = preambleLen;
                }
            }
            else
            {
                pkt_params.preamble_len_in_symb = preambleLen;
            }

            pkt_params.header_type = ( sx126x_lora_pkt_len_modes_t )fixLen;
            pkt_params.pld_len_in_bytes = MaxPayloadLength;
            pkt_params.crc_is_on = ( bool )crcOn;
            pkt_params.invert_iq_is_on = ( bool )iqInverted;

            RadioStandby( );
            sx126x_set_pkt_type( &SX126x , SX126X_PKT_TYPE_LORA);
            sx126x_set_lora_mod_params( &SX126x, &mod_params );
            sx126x_set_lora_pkt_params( &SX126x, &pkt_params );
            break;
    }
    sx126x_set_tx_params(&SX126x, power, SX126X_RAMP_200_US );
    TxTimeout = timeout;
}

bool RadioCheckRfFrequency( uint32_t frequency )
{
    return true;
}

uint32_t RadioTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        {
            double ts = RadioLoRaSymbTime[mod_params.bw - 4][12 - mod_params.sf];
            // time of preamble
            double tPreamble = ( pkt_params.preamble_len_in_symb + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * mod_params.sf +
                                 28 + 16 * pkt_params.crc_is_on -
                                 ( ( pkt_params.header_type == SX126X_LORA_PKT_IMPLICIT ) ? 20 : 0 ) ) /
                                 ( double )( 4 * ( mod_params.sf -
                                 ( ( mod_params.ldro > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( ( mod_params.cr % 4 ) + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return milli seconds
            airTime = floor( tOnAir + 0.999 );
        }
        break;
    }
    return airTime;
}

void RadioSend( uint8_t *buffer, uint8_t size )
{
    sx126x_set_dio_irq_params( &SX126x, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                           SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
                           SX126X_IRQ_NONE,
                           SX126X_IRQ_NONE );

    if( pkt_type ==  SX126X_PKT_TYPE_LORA )
    {
        pkt_params.pld_len_in_bytes = size;
    }
    else
    {
    }
    sx126x_set_lora_pkt_params( &SX126x, &pkt_params );

    sx126x_write_buffer( &SX126x , 0 , buffer , size );	//CHECK THE SECOND PARAMETER EQUAL TO 0
    //SX126xSendPayload( buffer, size, 0 );
    TimerSetValue( &TxTimeoutTimer, TxTimeout );
    TimerStart( &TxTimeoutTimer );
}

void RadioSleep( void )
{
	sx126x_sleep_cfgs_t params = { 0 };

    params = 1;
    sx126x_set_sleep( &SX126x , params );

    DelayMs( 2 );
}

void RadioStandby( void )
{
    sx126x_set_standby( &SX126x, SX126X_STANDBY_CFG_RC );
}

void RadioRx( uint32_t timeout )
{
    sx126x_set_dio_irq_params( &SX126x, SX126X_IRQ_ALL, //SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
                           SX126X_IRQ_ALL, //SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
                           SX126X_IRQ_NONE,
                           SX126X_IRQ_NONE );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( RxContinuous == true )
    {
    	sx126x_set_rx( &SX126x , 0xFFFFFF ); // Rx Continuous
    }
    else
    {
    	sx126x_set_rx( &SX126x , RxTimeout << 6 );
    }
}

void RadioRxBoosted( uint32_t timeout )
{
    sx126x_set_dio_irq_params( &SX126x, SX126X_IRQ_ALL, //SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
                           SX126X_IRQ_ALL, //SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
                           SX126X_IRQ_NONE,
                           SX126X_IRQ_NONE );

    if( timeout != 0 )
    {    
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( RxContinuous == true )
    {
    	sx126x_set_rx_with_timeout_in_rtc_step( &SX126x, SX126X_RX_CONTINUOUS ); // Rx Continuous
    }
    else
    {
    	sx126x_set_rx_with_timeout_in_rtc_step( &SX126x, SX126X_RX_SINGLE_MODE );
    }
}

void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
	sx126x_set_rx_duty_cycle( &SX126x, rxTime, sleepTime );
}

void RadioStartCad( void )
{
	sx126x_set_cad( &SX126x );
}

void RadioTx( uint32_t timeout )
{
	sx126x_set_tx( &SX126x, timeout << 6 );
}

void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    sx126x_set_rf_freq( &SX126x,freq );
    sx126x_set_tx_params(&SX126x, power, SX126X_RAMP_200_US );
    sx126x_set_tx_cw( &SX126x );

    TimerSetValue( &RxTimeoutTimer, time  * 1e3 );
    TimerStart( &RxTimeoutTimer );
}

int16_t RadioRssi( RadioModems_t modem )
{
    return sx126x_get_rssi_inst( &SX126x , &rssi );
}

void RadioWrite( uint16_t addr, uint8_t data )
{
    sx126x_write_register( &SX126x , addr, data , sizeof(data));
}

uint8_t RadioRead( uint16_t addr )
{
    return sx126x_read_register( &SX126x , addr , buff , sizeof(buff));
}

void RadioWriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    sx126x_write_register( &SX126x , addr, buffer, size );
}

void RadioReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    sx126x_read_register( &SX126x , addr, buffer, size );
}

void RadioWriteFifo( uint8_t *buffer, uint8_t size )
{
    sx126x_write_buffer( &SX126x , 0 , buffer, size );
}

void RadioReadFifo( uint8_t *buffer, uint8_t size )
{
    sx126x_read_buffer( &SX126x , 0, buffer, size );
}

void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    if( modem == MODEM_LORA )
    {
        pkt_params.pld_len_in_bytes = MaxPayloadLength = max;
        sx126x_set_lora_pkt_params( &SX126x, &pkt_params );
    }
    else
    {

    }
}

void RadioSetPublicNetwork( bool enable )
{
    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    RadioSetModem( MODEM_LORA );
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        sx126x_write_register( &SX126x, SX126X_REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF , sizeof(( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF) );
        sx126x_write_register( &SX126x, SX126X_REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF , sizeof(LORA_MAC_PUBLIC_SYNCWORD & 0xFF) );
    }
    else
    {
        // Change LoRa modem SyncWord
        sx126x_write_register( &SX126x, SX126X_REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF , sizeof(( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF) );
        sx126x_write_register( &SX126x,SX126X_REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF , sizeof(LORA_MAC_PRIVATE_SYNCWORD & 0xFF) );
    }
}

uint32_t RadioGetWakeupTime( void )
{
    return( RADIO_TCXO_SETUP_TIME + RADIO_WAKEUP_TIME );
}

void RadioOnTxTimeoutIrq( void )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
    {
        RadioEvents->TxTimeout( );
    }
}

void RadioOnRxTimeoutIrq( void )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
    {
        RadioEvents->RxTimeout( );
    }
}

void RadioOnDioIrq( void )
{
    IrqFired = true;
}

void RadioIrqProcess( void )
{
    if( IrqFired == true )
    {
    	//CHECK THIS ERRORS WITH OBC BoardDisableIrq( );
        IrqFired = false;
        //CHECK THIS ERRORS WITH OBC BoardEnableIrq( );
        uint16_t irqRegs = 0;	//DELETE THIS LINE
        //CHECK THIS ERRORS WITH OBC uint16_t irqRegs = sx126x_get_irq_status( &SX126x ); //CHECK THIS LINE WITH STM32F4XX_HAL_EXTI.H

        sx126x_clear_irq_status( &SX126x , SX126X_IRQ_ALL );

        if( ( irqRegs & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
        {
            TimerStop( &TxTimeoutTimer );
            if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
            {
                RadioEvents->TxDone( );
            }
        }

        if( ( irqRegs & SX126X_IRQ_RX_DONE ) == SX126X_IRQ_RX_DONE )
        {
            uint8_t size;

            TimerStop( &RxTimeoutTimer );
            sx126x_read_buffer( &SX126x , 0 , RadioRxPayload, &size); //CHECK THE 2ND PARAMETER EQUAL TO 0
            //SX126xGetPayload( RadioRxPayload, &size , 255 );
            sx126x_get_lora_pkt_status( &SX126x , &RadioPktStatus );
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
            {
                RadioEvents->RxDone( RadioRxPayload, size, pkt_status.signal_rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db );
            }
        }

        if( ( irqRegs & SX126X_IRQ_CRC_ERROR ) == SX126X_IRQ_CRC_ERROR )
        {
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxError ) )
            {
                RadioEvents->RxError( );
            }
        }

        if( ( irqRegs & SX126X_IRQ_CAD_DONE ) == SX126X_IRQ_CAD_DONE )
        {
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( ( ( irqRegs & SX126X_IRQ_CAD_DETECTED ) == SX126X_IRQ_CAD_DETECTED ) );
            }
        }

        if( ( irqRegs & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT )
        {
            if( sx126x_get_status( &SX126x , &status.chip_mode) == SX126X_CHIP_MODE_TX )
            {
                TimerStop( &TxTimeoutTimer );
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
                {
                    RadioEvents->TxTimeout( );
                }
            }
            else if( sx126x_get_status( &SX126x , &status.chip_mode) == SX126X_CHIP_MODE_RX )
            {
                TimerStop( &RxTimeoutTimer );
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
            }
        }

        if( ( irqRegs & SX126X_IRQ_PREAMBLE_DETECTED ) == SX126X_IRQ_PREAMBLE_DETECTED )
        {
            //__NOP( );
        }

        if( ( irqRegs & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
        {
            TimerStop( &RxTimeoutTimer );
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
            {
                RadioEvents->RxTimeout( );
            }
        }
    }
}
