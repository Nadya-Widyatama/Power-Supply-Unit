/*
 * DS18B20.h
 *
 *  Created on: Dec 7, 2024
 *      Author: user
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "main.h"

#define DS18B20_PIN GPIO_PIN_5
#define DS18B20_PORT GPIOA
#define TX_TICK_RATE 10  // 1 tick = 10 ms


void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void delay_us(uint16_t us);
void thread_sleep_ms(uint32_t ms);
uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
float DS18B20_GetTemp(void);

#endif /* INC_DS18B20_H_ */
