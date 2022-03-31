/*
 * task_pl.c
 *
 *  Created on: 28 mar. 2022
 *      Author: ainna
 */
#include "definitions.h"
#include "configuration.h"
#include "payload_camera.h"

void vPayloadTask(void *pvParameters) {
	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	uint32_t payload_time;
	const char *pcTaskName = "Task Payload";

	for (;;) {
		// Wait until comms notify us to take data
		//while (!Take_Photo_noti) {
		//}

		// How much time to make the antenna point to the Earth
		// ADCS: Antenna points to the Earth
		// antenna_pointing() --> ADCS

		// Read from memory the time to use the payload (Exact time)
		Read_Flash(PL_TIME_ADDR, &payload_time, sizeof(payload_time));

		//Read the real time clock
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		// We convert the Data and Time into UnixFormat
		uint8_t real_time;
		t(real_time, &sTime, &sDate);
		// Wait until the times coincide
		while (payload_time > real_time) {
		}
		// Take Data
		// takePhoto();
		resetCommsParams();

		// Second DATA_TAKEN_noti to COOMS_task !!!

		// if CONTINGENCY_state_noti
		// vDeleteTask()
	}
}
