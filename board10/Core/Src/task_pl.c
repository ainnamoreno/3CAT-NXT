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
	uint32_t signal_to_wait = 0;
	uint32_t signal_received = 0;

	for (;;) {
		// Esperar 1a notificació a COMMS fer foto

		if (xTaskNotifyWait(0, signal_to_wait, &signal_received,portMAX_DELAY) == pdTRUE) {
			if (signal_received & TAKEPHOTO_NOTI) {
				// Inicialitzar la camara (activar PIN VCC)

				// Read from memory the time to use the payload (Exact time)
				Read_Flash(PL_TIME_ADDR, &payload_time, sizeof(payload_time));
				// Esperar 2a notificació a que ADCS estigui apuntant
				if (xTaskNotifyWait(0, signal_to_wait, &signal_received,
				portMAX_DELAY) == pdTRUE) {
					if (signal_received & POINTING_NOTI) {
						// Wait until the times coincide
						do {
							//RTC
							realTime_int = PL_Time(&hrtc);

						} while (payload_time > realTime_int);

						// Take Data
						while (!takePhoto(&huart1))
							;
						// resetCommsParams();

						// xTaskNotify("Task OBC", DONEPHOTO_NOTI, eSetBits);
						// Apagar la PAYLOAD
					}
				}
			}
			if (signal_received & CONTINGENCY_NOTI) {
				vDeleteTask();
			}

		}
	}
}
