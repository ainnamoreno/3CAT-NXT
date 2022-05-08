/*
 * task_obc.c
 *
 *  Created on: 25 abr. 2022
 *      Author: ainna
 */

#include "task_obc.h"

void vOBCTask(void *pvParameters) {
	const char *pcTaskName = "Task OBC";

	uint8_t currentState, exit_low, nominal, low, critical;
	Write_Flash(CURRENT_STATE_ADDR, INIT, 1);
	Read_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));
	Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
	// Update the battery thresholds -> OKEY
	uint8_t prova_nom = 90, prova_low = 85, prova_critical = 80;
	Write_Flash(NOMINAL_ADDR, &prova_nom, sizeof(prova_nom));
	Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
	Write_Flash(LOW_ADDR, &prova_low, sizeof(prova_low));
	Read_Flash(LOW_ADDR, &low, sizeof(low));
	Write_Flash(CRITICAL_ADDR, &prova_critical, sizeof(prova_critical));
	Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));

	// Signals related to NOTIFICATIONS
	uint32_t signal_received = 0;
	uint32_t signal_to_wait = CONTINGENCY_NOTI | WAKEUP_NOTI | RESET_NOTI |
	GS_NOTI | NOTGS_NOTI | TAKEPHOTO_NOTI | TAKERF_NOTI |
	DONEPHOTO_NOTI | DONERF_NOTI | PAYLOAD_ERROR_NOTI |
	EXITLOWPOWER_NOTI | SETTIME_NOTI | NOMINAL_NOTI |
	LOW_NOTI | CRITICAL_NOTI | CTEKP_NOTI | TLE_NOTI |
	GYRORES_NOTI | CALIBRATION_NOTI | POINTING_NOTI |
	DETUMBLING_NOTI | POINTING_DONE_NOTI | DETUMBLING_NOTI |
	ROTATE_NOTI | STOP_POINTING_NOTI;

	uint32_t settime;
	;
	for (;;) {
		// START OF THE OBC STATE MACHINE;
		//Look for a new notification (UPDATE VALUES or NOTIFY another Task)
		// function --> notifications() ?
		if (xTaskNotifyWait(0, signal_to_wait, &signal_received,
				portMAX_DELAY) == pdTRUE) {
			if (signal_received & RESET_NOTI) {
				// If we need to do a system reset
				HAL_NVIC_SystemReset();
			}
			if (signal_received & PAYLOAD_ERROR_NOTI) {
				// Notify COMMS that PL Camera is not working
				xTaskNotify("Task COMMS", PAYLOAD_ERROR_NOTI, eSetBits);
			}
			if (signal_received & NOMINAL_NOTI) {
				//If GS changed the NOMINAL threshold
				Read_Flash(NOMINAL_ADDR, &nominal, sizeof(nominal));
			}
			if (signal_received & LOW_NOTI) {
				//If GS changed the LOW threshold
				Read_Flash(LOW_ADDR, &low, sizeof(low));
			}
			if (signal_received & CRITICAL_NOTI) {
				//If GS changed the CRITICAL threshold
				Read_Flash(CRITICAL_ADDR, &critical, sizeof(critical));
			}
			if (signal_received & SETTIME_NOTI) {
				//If GS send TIME
				Read_Flash(SET_TIME_ADDR, &settime, sizeof(settime));
				UnixToHumanTime(settime, &hrtc);
			}
			if (signal_received & GS_NOTI) {
				//If contact region with GS -> NOTIFY COMMS
				xTaskNotify("Task COMMS", GS_NOTI, eSetBits);
			}
			if (signal_received & TAKEPHOTO_NOTI) {
				// If contact TAKEPHOTO_NOTI
				// Notify ADCS it should start pointing
				xTaskNotify("Task ADCS", POINTING_NOTI, eSetBits);
				// Wake up PL Task
				xTaskNotify("Task PAYLOAD", WAKEUP_NOTI, eSetBits);
			}
			if (signal_received & TAKERF_NOTI) {
				// Wake up PL RF Task
				// xTaskNotify("Task PAYLOAD RF", WAKEUP_NOTI, eSetBits);
			}
			if (signal_received & DONEPHOTO_NOTI) {
				// Notify ADCS to stop pointing
				xTaskNotify("Task ADCS", STOP_POINTING_NOTI, eSetBits);
				// Notify COMMS
				xTaskNotify("Task COMMS", DONEPHOTO_NOTI, eSetBits);
			}
			if (signal_received & DONERF_NOTI) {
				// Notify COMMS
				// xTaskNotify("Task COMMS", DONERF_NOTI, eSetBits);
			}
			if (signal_received & CTEKP_NOTI) {
				// Notify ADCS it should start pointing
				xTaskNotify("Task ADCS", CTEKP_NOTI, eSetBits);
			}
			if (signal_received & TLE_NOTI) {
				// Notify ADCS it should start pointing
				xTaskNotify("Task ADCS", TLE_NOTI, eSetBits);
			}
			if (signal_received & GYRORES_NOTI) {
				// Notify ADCS it should start pointing
				xTaskNotify("Task ADCS", GYRORES_NOTI, eSetBits);
			}
			if (signal_received & CALIBRATION_NOTI) {
				// Notify ADCS it should start pointing
				xTaskNotify("Task ADCS", CALIBRATION_NOTI, eSetBits);
			}
		}
		// Check which state
		switch (currentState) {

		case CHECK: // Check satellite general state
			/* State that periodically checks the satellite general state (batteries,
			 * temperatures, voltages...
			 * From this state the satellite can go to contingency states
			 * if systemstate() returns a value different than 0 */

			// It is only done the first time we enter to IDLE
			// We send notifications to COMMS, ADCS tasks to start tunning (IDLE_State_noti)
			if (system_state(&hi2c1, nominal, low, critical) > 0) {
				currentState = CONTINGENCY;
				// Only update if we change state
				Write_Flash(PREVIOUS_STATE_ADDR, CHECK, 1);
				Write_Flash(CURRENT_STATE_ADDR, &currentState,
						sizeof(currentState));
			} else {
				sensorReadings(&hi2c1); /*Updates the values of temperatures, voltages and currents*/
			}
			break;
		case CONTINGENCY:
			/*Turn STM32 to Stop Mode or Standby Mode
			 *Loop to check at what batterylevel are we
			 *Out of CONTINGENCY State when batterylevel is NOMINAL
			 */
			// TODO: LOW POWER RUN MODE (De momento podriem fer un SleepMode)
			// HAL_PWR_EnterSTOPMode();
			// Avisar a COMMS que entrem en CONTINGENCY (enviar paquet de telemetria)
			// S'envia cada vegada que rebem un EXITLOWPOWER_NOTI i la bateria no millora
			xTaskNotify("Task COMMS", CONTINGENCY_NOTI, eSetBits);
			// Esperem que COMMS rebi telecommand de la GS (EXITLOWPOWER_NOTI)
			if (xTaskNotifyWait(0, signal_to_wait, &signal_received,
					portMAX_DELAY) == pdTRUE) {
				if (signal_received & EXITLOWPOWER_NOTI) {
					// Mirem el nivell de bateria
					// Si ha empitjorat
					if (system_state(&hi2c1, nominal, low, critical) > 1) {
						currentState = SUNSAFE;
						// Actualitzem l'estat només si canviem d'estat
						Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
						Write_Flash(CURRENT_STATE_ADDR, &currentState,
								sizeof(currentState));

						// Si ha millorat
					} else if (system_state(&hi2c1, nominal, low, critical)
							== 0) {
						/*Return to Run Mode*/
						currentState = CHECK;
						// Actualitzem l'estat només si canviem d'estat
						Write_Flash(PREVIOUS_STATE_ADDR, CONTINGENCY, 1);
						Write_Flash(CURRENT_STATE_ADDR, &currentState,
								sizeof(currentState));
					}
				}
			}

			// Una opció és fer reset total del satelit quan surti de contingency

			break;

		case SUNSAFE:
			Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
			// Entrem en el SleepMode
			// HAL_PWR_EnterSLEEPMode();

			// IWDG initialize
			if (system_state(&hi2c1, nominal, low, critical) == 2) {
				// Nothing has changed:
				// Delay the system so the IWDG resets the system
				HAL_Delay(33000);
			}
			// IWDG exits SleepMode automatically
			// system_state = 3 --> battery level < CRITICAL
			else if (system_state(&hi2c1, nominal, low, critical) == 3) {
				currentState = SURVIVAL;
				// Update only if state change
				Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));

				// system_state = 1 --> LOW < battery level < NOMINAL
			} else if (system_state(&hi2c1, nominal, low, critical) == 1) {
				/*Return to Run Mode*/
				currentState = CONTINGENCY;
				// Update only if state change
				Write_Flash(PREVIOUS_STATE_ADDR, SUNSAFE, 1);
				Write_Flash(CURRENT_STATE_ADDR, currentState,
						sizeof(currentState));
			}
			break;

		case SURVIVAL:
			// Enter in the Low Power Sleep Mode
			// HAL_PWR_EnterLOWPOWERSLEEPMode();

			// Refresh IWDG to avoid reseting the system
			HAL_IWDG_Refresh(&hiwdg);
			// From SURVIVAL we can only go to CONTINGENCY
			if (system_state(&hi2c1, nominal, low, critical) == 1) {
				currentState = CONTINGENCY;
				// Only update if state change
				Write_Flash(PREVIOUS_STATE_ADDR, SURVIVAL, 1);
				Write_Flash(CURRENT_STATE_ADDR, &currentState,
						sizeof(currentState));
			} else {
				HAL_Delay(33000);
			}

			break;

		case INIT:
			init(&hi2c1);
			Write_Flash(PREVIOUS_STATE_ADDR, INIT, 1);
			Read_Flash(CURRENT_STATE_ADDR, &currentState, sizeof(currentState));
			// Wake up COMMS and ADCS tasks
			xTaskNotify("Task ADCS", DONEPHOTO_NOTI, eSetBits);
			xTaskNotify("Task COMMS", DONEPHOTO_NOTI, eSetBits);
			// Send notification to ADCS --> DETUMBLING_NOTI
			//xTaskNotify("Task ADCS", DETUMBLING_NOTI, eSetBits);

			break;
			/*If we reach this state something has gone wrong*/
		default:
			/*REBOOT THE SYSTEM*/
			break;
		}

		/*Start a TIMER*/

//	    return 0;
		//todo variable que conti ticks rellotge per fer reset
	}

}
