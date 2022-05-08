#include "task_comms.h"



void vCommsTask(void *pvParameters) {
	const char *pcTaskName = "Task Comms";

	for (;;) {
		// Start their own statemachine
		// Passarli al state machine el signal_received -> PER PODER COMUNICAR LES FUNCIONS
		// stateMachine();
		// If GS_noti from ADCS -> change comms state machine to RX
		// if process_telecommand = SET_TIME, we send the RTC to GS
		// if Data_taken_noti (from payload_task) --> senddata() to GS
		// if state = CONTINGENCY --> Send_telemetry to GS
		// if process_telecommand = EXIT_LOW_PWER --> Send ExitLowPower_noti to main_task

		// if CONTINGENCY_state_noti
		// vDeleteTask()
	}
}


