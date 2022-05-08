/*
 * task_pl.c
 *
 *  Created on: 28 mar. 2022
 *      Author: ainna
 */

#include "task_pl.h"

void vPayloadTask(void *pvParameters) {

	uint32_t payload_time, realTime_int;
	const char *pcTaskName = "Task PAYLOAD";
	//TODO: Pasarli el hrtc !!!


	// signal to wait son les possibles senyals que podem rebre
	uint32_t signal_to_wait = WAKEUP_NOTI | POINTING_DONE_NOTI | CONTINGENCY_NOTI;
	uint32_t signal_received = 0;

	for (;;) {
		// Wait for Wake up notification

		if (xTaskNotifyWait(0, signal_to_wait, &signal_received,portMAX_DELAY) == pdTRUE) {
			if (signal_received & WAKEUP_NOTI) {
				// Initialize the Camera (SWITCH/PIN VCC)

				// Read from memory the when to take the photo
				Read_Flash(PL_TIME_ADDR, &payload_time, sizeof(payload_time));
				// Wait for ADCS to point
				if (xTaskNotifyWait(0, signal_to_wait, &signal_received,
				portMAX_DELAY) == pdTRUE) {
					if (signal_received & POINTING_NOTI) {
						// Wait until the it's time
						do {
							// Check real time
							HumanToUnixTime(&hrtc, realTime_int);

						} while (payload_time > realTime_int);

						// Take Data
						while (!takePhoto(&huart1))
							;
						// resetCommsParams();
						// If photo has been taken correctly:
						xTaskNotify("Task OBC", DONEPHOTO_NOTI, eSetBits);

						// Turn off the CAMERA
					}
				}
			}
			if (signal_received & CONTINGENCY_NOTI) {
				// vDeleteTask();
			}

		}
	}
}
