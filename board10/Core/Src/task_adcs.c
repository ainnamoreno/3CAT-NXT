/*
 * task_adcs.c
 *
 *  Created on: 28 mar. 2022
 *      Author: ainna
 */

#include "task_adcs.h"

void vADCSTask(void *pvParameters) {

	const char *pcTaskName = "Task ADCS";
	uint32_t received_value = 0;

	for (;;) {
		// Constantly checking the sat position
		checkposition();

		// if in the contact region
		// xTaskNotify(OBC_task, GS_NOTI, eSetBits)

		// if NOT the contact region
		// xTaskNotify(OBC_task, NOTGS_NOTI, eSetBits)

		// if(xTaskNotifyWait(0,0,ULONG_MAX,&received_value,portMAX_DELAY)
			// if(received_value == CONTINGENCY_NOTI)
			// vDeleteTask()

			// if(received_value == TAKEPHOTO_NOTI) --> start pointing
				//when finish pointing --> Send notification POINTING_NOTI
				//xTaskNotify(PL_task, POINTING_NOTI, eSetBits);
	}
}