/*
 * utils.h
 *
 *  Created on: 28 mar. 2022
 *      Author: ainna
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

// HandleTypeDef
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern WWDG_HandleTypeDef hwwdg;

// GENERAL NOTIFICATIONS
// Notification: Contingency state
// To: COMMS, ADCS
// From:OBC
#define CONTINGENCY_NOTI	0x00100000
// Notification: Wake up a task that is suspended (sleep)
// To: COMMS, ADCS, PAYLOAD
// From:OBC
#define WAKEUP_NOTI			0x01000000


// COMMS NOTIFICATIONS
#define GS_NOTI				0x00000001
#define NOTGS_NOTI			0x00000010


// PAYLOAD Notifications
// Notification: Take photo
// To: PAYLOAD_CAMERA
// From: COMMS
#define TAKEPHOTO_NOTI		0x00000100
// Notification: Take RF
// To: PAYLOAD_RF
// From: COMMS
#define TAKERF_NOTI
// Notification: Photo taken
// To: COMMS
// From: PAYLOAD_CAMERA
#define DONEPHOTO_NOTI		0x00001000
// Notification: RF taken
// To: COMMS
// From: PAYLOAD_RF
#define DONERF_NOTI	1<<7


// OBS NOTIFICATIONS
// Notification: Exit Low Power Mode in Contingency State
// To: OBC
// From: COMMS
#define EXITLOWPOWER_NOTI	0x10000000
// Notification: Set time from GS
// To: OBC
// From: COMMS
#define SETTIME_NOTI 1<<30
// Notification: New NOMINAL level
// To: OBC
// From: COMMS
#define NOMINAL_NOTI 1<<31
// Notification: New LOW level
// To: OBC
// From: COMMS
#define LOW_NOTI	1<<29
// Notification: New CRITICAL level
// To: OBC
// From: COMMS
#define CRITICAL_NOTI 1<<28

// ADCS NOTIFICATIONS
#define CTEKP_NOTI
// Notification: TLE available
// To: ADCS
// From: COMMS-> OBC
#define TLE_NOTI
#define GYRORES_NOTI
#define CALIBRATION_NOTI
#define POINTING_NOTI		0x00010000
#define DETUMBLING_NOTI
#define ROTATE_NOTI

#endif /* INC_UTILS_H_ */
