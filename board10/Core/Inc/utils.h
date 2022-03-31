/*
 * utils.h
 *
 *  Created on: 28 mar. 2022
 *      Author: ainna
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern IWDG_HandleTypeDef hiwdg;

extern RTC_HandleTypeDef hrtc;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;

extern HCD_HandleTypeDef hhcd_USB_OTG_FS;

extern WWDG_HandleTypeDef hwwdg;

#endif /* INC_UTILS_H_ */
