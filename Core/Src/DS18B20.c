/*
 * DS18B20.c
 *
 *  Created on: Dec 7, 2024
 *      Author: user
 */

#include "DS18B20.h"
#include "tx_api.h"
extern TIM_HandleTypeDef htim2;

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Base_Start(&htim2);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us){
		tx_thread_relinquish();
	}
	HAL_TIM_Base_Stop(&htim2);
}

uint8_t DS18B20_Start(void) {
	uint8_t response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
	HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
	delay_us(480);
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
	delay_us(80);
	if (!HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))response = 1;
	else response = -1;
	delay_us(400);
	return response;
}

void DS18B20_Write(uint8_t data) {
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
	for (int i = 0; i < 8; i++) {
		if (data & (1 << i)) {
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
			delay_us(1);
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
			delay_us(60);
		} else {
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
			delay_us(60);
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read(void) {
	uint8_t value = 0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
	for (int i = 0; i < 8; i++) {
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
		HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
		delay_us(2);
		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)) {
			value |= 1 << i;
		}
		delay_us(60);
	}
	return value;
}

float DS18B20_GetTemp(void) {
	uint8_t temp_lsb, temp_msb;
	int16_t temp;

	DS18B20_Start();
	DS18B20_Write(0xCC);  // SKIP ROM
	DS18B20_Write(0x44);  // CONVERT T
	tx_thread_sleep(750);

	DS18B20_Start();
	DS18B20_Write(0xCC);  // SKIP ROM
	DS18B20_Write(0xBE);  // READ SCRATCHPAD

	temp_lsb = DS18B20_Read();
	temp_msb = DS18B20_Read();
	temp = (temp_msb << 8) | temp_lsb;

	return (float)temp / 16.0;
}
