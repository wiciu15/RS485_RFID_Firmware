/*
 * led.h
 *
 *  Created on: Jun 7, 2020
 *      Author: wiki1
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"

/*
#define LED_RED_CH  	TIM_CHANNEL_1
#define LED_GREEN_CH 	TIM_CHANNEL_2
#define LED_BLUE_CH 	TIM_CHANNEL_3
#define BUZZER_CH		TIM_CHANNEL_4
*/
extern TIM_HandleTypeDef htim2;

typedef enum
{
    RED,
    GREEN,
    BLUE
} typedef_color;

void LED_SignalDeviceError(uint8_t numOfBeeps);
void LED_Init(void);
void LED_SetColor(uint8_t color);
void LED_Clear(void);
void LED_ClearColor(uint8_t color);
void BUZZ_Enable(void);
void BUZZ_Disable(void);


#endif /* INC_LED_H_ */
