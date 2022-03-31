///*
// * telecommands.c
// *
// *  Created on: Dec 11, 2021
// *      Author: psimon
// */
//
//#include "telecommands.h"
//
//
//
///**************************************************************************************
// *                                                                                    *
// * Function:  process_telecommand                                                     *
// * --------------------                                                               *
// * processes the information contained in the packet depending on the telecommand     *
// * received																	          *
// *                                                                                    *
// *  header: number of telecommand			                                          *
// *  info: information contained in the received packet								  *
// *                                                                                    *
// *  returns: nothing									                              *
// *                                                                                    *
// **************************************************************************************/
//void process_telecommand(uint8_t header, uint8_t info) {
//	switch(header) {
//	case RESET2:
//		/*Segons el drive s'ha de fer el reset si val 1 el bit, així que potser
//		 * s'hauria de posar un if*/
//		HAL_NVIC_SystemReset();
//		break;
//	case NOMINAL:
//		Write_Flash(NOMINAL_ADDR, &info, 1);
//		break;
//	case LOW:
//		Write_Flash(LOW_ADDR, &info, 1);
//		break;
//	case CRITICAL:
//		Write_Flash(CRITICAL_ADDR, &info, 1);
//		break;
//	case EXIT_LOW_POWER:
//		Write_Flash(EXIT_LOW_POWER_FLAG_ADDR, &info, 1);
//		Write_Flash(EXIT_LOW_ADDR, TRUE, 1);
//		break;
//	case SET_TIME:
//		setTime();
//		break;
//	case SET_CONSTANT_KP:
//		Write_Flash(KP_ADDR, &info, 1);
//		break;
//	case TLE:
//		tle();
//		break;
//	case SET_GYRO_RES:
//		/*4 possibles estats,rebrem 00/01/10/11*/
//		Write_Flash(GYRO_RES_ADDR, &info, 1);
//		break;
//	case SENDDATA:
//		sendData();
//		break;
//	case SENDTELEMETRY:
//		sendtelemtry();
//		break;
//	case STOPSENDINGDATA:
//		stopsendingData();
//		break;
//	case ACKDATA:
//		ackData();
//		//translate the ACK/NACK into an understandable format for the transmission function, so that when the packaging function is called, it sends the packets from the last window that needs to be retransmitted. After changing the ACK format, switches the states to TX.
//		break;
//	case SET_SF_CR:  //semicolon added in order to be able to declare SF here
//						//	/*4 cases (4/5, 4/6, 4/7,1/2), so we will receive and store 0, 1, 2 or 3*/
//		uint8_t SF;
//		if (info == 0) {SF = 7};
//		else if (info == 1) {SF = 8;}
//		else if (info == 2) {SF = 9;}
//		else if (info == 3) {SF = 10;}
//		else if (info == 4) {SF = 11;}
//		else if (info == 5) {SF = 12;}
//		Write_Flash(SF_ADDR, &SF, 1);
//		Write_Flash(CRC_ADDR, &Buffer[2], 1);
//		break;
//	case SEND_CALIBRATION:
//		sendcalibration();
//		break;
//	case TAKEPHOTO:
//		Write_Flash(PAYLOAD_STATE_ADDR, TRUE, 1);
//		Write_Flash(PL_TIME_ADDR, &info, 8);
//		Write_Flash(PHOTO_RESOL_ADDR, &Buffer[5], 1);
//		Write_Flash(PHOTO_COMPRESSION_ADDRY, &Buffer[6], 1);
//		break;
//	case TAKERF:
//		Write_Flash(PAYLOAD_STATE_ADDR, TRUE, 1);
//		Write_Flash(PL_TIME_ADDR, &info, 8);
//		Write_Flash(PHOTO_RESOL_ADDR, &Buffer[5], 1);
//		Write_Flash(F_MIN_ADDR, &Buffer[9], 1);
//		Write_Flash(F_MAX_ADDR, &Buffer[10], 1);
//		Write_Flash(DELTA_F_ADDR, &Buffer[11], 1);
//		Write_Flash(INTEGRATION_TIME_ADDR, &Buffer[12], 1);
//		break;
//	case SEND_CONFIG: ; //semicolon added in order to be able to declare SF here
//		uint8_t config[CONFIG_SIZE];
//		Read_Flash(CONFIG_ADDR, &config, CONFIG_SIZE);
//		Radio.Send(config, CONFIG_SIZE);
//		break;
//	}
//	//ha arribat telecomand (a obc), adressa o flag
//}
//void sendTelemetry(){
//	if(!contingency){
//		send_telemetry=false;
//		num_telemetry=(uint8_t) 34+BUFFER_SIZE + 1; //cast to integer to erase the decimal part
//		State = TX;
//	}
//	Write_Flash(TELEMETRY_ADDR, TRUE, 1);
//	break;
//}
//void sendData() {
//	if (!contingency){ //com entrar estat contingency
//	State = TX;
//	send_data = true;
//	}
//	break;
//}
//void stopsendingData(){
//	send_data = false;
//	count_packet[0] = 0;
//	break;
//}
//void sendcalibration(){
//	uint8_t calib[UPLINK_BUFFER_SIZE-1]; //RX
//	for (i=1; i<UPLINK_BUFFER_SIZE; i++){
//		calib[k-1]=Buffer[k];
//	}
//	Write_Flash(CALIBRATION_ADDR, &calib, sizeof(calib));
//	calib_packets = calib_packets + 1;
//	uint8_t integer_part = (uint8_t) 138/UPLINK_BUFFER_SIZE;
//	if(calib_packets == integer_part+1){
//	calib_packets = 0;
//	}
//	break;
//}
//void ackData(){
//	ack = ack & Buffer[1];
//	for(j=2; j<ACK_PAYLOAD_LENGTH; j++){
//	ack = (ack << 8*j) & Buffer[j];
//	}
//	count_window[0] = 0;
//	full_window = false;
//	if (ack != 0xFFFFFFFFFFFFFFFF){
//	nack = true;
//	}
//	State = TX;
//	break;
//}
//void setTime(){
//	uint8_t time[4];  //4bytes
//			for(n=0; n<<4; n++){
//				time[n]=Buffer(n+1);
//			}
//			Write_Flash(TEMP_ADDR, &time, sizeof(time));
//}
//void tle(){
//	/*Aquí si es reben els TLEs en diferents paquets doncs s'han d'anar
//			 * escrivint poc a poc o anar emmagatzemant la info i quan es tinguin
//			 * tots els bytes junts s'emmagatzemen (suposo que els paquets arriben
//			 * seguits)*/
//			uint8_t tle[UPLINK_BUFFER_SIZE-1];
//			for (k=1; k<UPLINK_BUFFER_SIZE; k++){
//			tle[k-1]=Buffer[k];
//			}
//			Write_Flash(TLE_ADDR + tle_packets*UPLINK_BUFFER_SIZE, &tle, sizeof(tle));
//			tle_packets++;
//			uint8_t integer_part = (uint8_t) 138/UPLINK_BUFFER_SIZE;
//			if (tle_packets == integer_part+1){
//			tle_packets = 0;
//			}
//			break;
//}
