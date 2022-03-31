/*
 * task_adcs.c
 *
 *  Created on: 28 mar. 2022
 *      Author: ainna
 */

#include "definitions.h"
#include "configuration.h"
#include "adcs.h"

void vADCSTask(void *pvParameters) {

	const char *pcTaskName = "Task ADCS";

	for (;;) {
		// Constantly checking the sat position
		checkposition();

		// if in the contact region
		// send GS_noti to COMMS

		// if NOT the contact region
		// send GS_not_noti to COMMS

		// if CONTINGENCY_state_noti
		// vDeleteTask()
	}
}
