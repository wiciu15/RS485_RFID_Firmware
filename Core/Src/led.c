/*
 * led.c
 *
 *  Created on: Jun 7, 2020
 *      Author: wiki1
 */

#include "led.h"

void LED_SignalDeviceError(uint8_t numOfBeeps){
	for(uint8_t i=0;i<numOfBeeps;i++){
		LED_Clear();
		BUZZ_Disable();
		osDelay(300);
		LED_SetColor(RED);
		BUZZ_Enable();
		osDelay(200);
	}
	BUZZ_Disable();
}

void LED_Init(void){
	BUZZ_Enable();
	for(uint16_t i=0;i<2;i++){
		LED_SetColor(RED);
		osDelay(200);
		LED_SetColor(GREEN);
		osDelay(200);
		LED_SetColor(BLUE);
		osDelay(200);
		LED_ClearColor(RED);
		osDelay(200);
		LED_ClearColor(GREEN);
		osDelay(200);
		LED_ClearColor(BLUE);
		osDelay(500);
	}
	BUZZ_Disable();

	/*
	__HAL_TIM_SET_COMPARE(&htim2,BUZZER_CH,1000);
	for(uint16_t i=0;i<99;i++){
		LED_SetColor(RED, i*100);
		osDelay(1);
	}
	osDelay(100);
	__HAL_TIM_SET_COMPARE(&htim2,BUZZER_CH,3000);
	for(uint16_t i=99;i>=0;i--){
		LED_SetColor(RED, i*100);
		osDelay(1);
	}
	osDelay(100);
	for(uint16_t i=0;i<99;i++){
		LED_SetColor(GREEN, i*100);
		osDelay(1);
	}
	__HAL_TIM_SET_COMPARE(&htim2,BUZZER_CH,5000);
	osDelay(100);
	for(uint16_t i=99;i>=0;i--){
		LED_SetColor(GREEN, i*100);
		osDelay(1);
	}
	__HAL_TIM_SET_COMPARE(&htim2,BUZZER_CH,8000);
	osDelay(100);
	for(uint16_t i=0;i<99;i++){
		LED_SetColor(BLUE, i*100);
		osDelay(1);
	}
	__HAL_TIM_SET_COMPARE(&htim2,BUZZER_CH,9999);
	osDelay(100);
	for(uint16_t i=99;i>=0;i--){
		LED_SetColor(BLUE, i*100);
		osDelay(1);
	}
	__HAL_TIM_SET_COMPARE(&htim2,BUZZER_CH,0);
	*/
}

void LED_SetColor(typedef_color color){
	switch (color){
	case RED:
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		break;
	case GREEN:
			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
			break;
	case BLUE:
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
			break;
	}
}

void LED_Clear(){
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
}
void LED_ClearColor(uint8_t color){
	switch (color){
	case RED:
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		break;
	case GREEN:
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		break;
	case BLUE:
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
		break;
	}
}

void BUZZ_Enable(){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}
void BUZZ_Disable(){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}
