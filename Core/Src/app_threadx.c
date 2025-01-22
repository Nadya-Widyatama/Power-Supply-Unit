/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS18B20.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THREAD_STACK_SIZE 2048
#define Sensor_Current_Limit 30		//provide a value that corresponds to the sensor used
#define BatteryCapacity1 1.0 //9.2 	//Measurement in units of Ah
#define BatteryCapacity2 1.0 //3.31 //Measurement in units of Ah
#define SoC_End_Discharge 20.0

#define DS18B20_PIN GPIO_PIN_5
#define DS18B20_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t thread_Setone[THREAD_STACK_SIZE];
uint8_t thread_Settwo[THREAD_STACK_SIZE];
uint8_t thread_Setthree[THREAD_STACK_SIZE];
uint8_t thread_Setfour[THREAD_STACK_SIZE];
uint8_t thread_Setfive[THREAD_STACK_SIZE];
uint8_t thread_Setsix[THREAD_STACK_SIZE];

TX_THREAD setone;
TX_THREAD settwo;
TX_THREAD setthree;
TX_THREAD setfour;
TX_THREAD setfive;
TX_THREAD setsix;

void Beep_Beep(uint8_t cycle, uint16_t delay1, uint16_t delay2);
void Save_Data(void *value, uint32_t address, size_t size);
void Read_Data(void *buffer, uint32_t address, size_t size);

uint32_t SoC_Temp_current_time1, SoC_Temp_current_time2,
SoC_prev_Temp_current_time1,SoC_prev_Temp_current_time2;
uint32_t address_SoH1 = 0x08000000, address_SoH2 = 0x08002000;
uint32_t address_Cycle1 = 0x08004000, address_Cycle2 = 0x08006000;
uint8_t SoH1, SoH2, st1 = 0, st2 = 0, activePowerSource = 1;

float AH_Restored1, AH_Consumed1, AH_Restored2, AH_Consumed2, ActualBatteryCapacity1, ActualBatteryCapacity2;
float temperature, voltage1, current1, voltage2, current2, voltagecharge, SoC1 = 100, SoC2 = 100;
float capacity_actual1, capacity_actual2, cycle_count1, cycle_count2;
int setzerobatt1 = 0, setzerobatt2 = 0, setcharging1 = 0, setcharging2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID ADC_Reading(ULONG initial_input);
VOID Set_LED(ULONG initial_input);
VOID Transmit_Data(ULONG initial_input);
VOID Power_Management(ULONG initial_input);
VOID Temperature_Reading(ULONG initial_input);
VOID SoH_Management(ULONG initial_input);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
	(void)byte_pool;

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
	tx_thread_create(&setone, "Setone", ADC_Reading, 0, thread_Setone, THREAD_STACK_SIZE, 4, 4, 1, TX_AUTO_START);
	tx_thread_create(&settwo, "Settwo", Set_LED, 0, thread_Settwo, THREAD_STACK_SIZE, 6, 6, 1, TX_AUTO_START);
	tx_thread_create(&setthree, "Setthree", Transmit_Data, 0, thread_Setthree, THREAD_STACK_SIZE, 6, 6, 1, TX_AUTO_START);
	tx_thread_create(&setfour, "Setfour", Power_Management, 0, thread_Setfour, THREAD_STACK_SIZE, 7, 7, 1, TX_AUTO_START);
	tx_thread_create(&setfive, "Setfive", Temperature_Reading, 0, thread_Setfive, THREAD_STACK_SIZE, 4, 4, 1, TX_AUTO_START);
	tx_thread_create(&setsix, "Setsix", SoH_Management, 0, thread_Setsix, THREAD_STACK_SIZE, 5, 5, 1, TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

void ADC_Reading(ULONG initial_input) {
	uint32_t sumADC_voltage1, sumADC_current1, sumADC_voltage2, sumADC_current2, adcBuffer[4];
	uint32_t avg_voltage1, avg_current1, avg_voltage2, avg_current2;
	uint16_t value_voltage1, value_current1, value_voltage2, value_current2;
	float voltage_current1, voltage_current2, sensitivity, currentdischargelimit;

	if(Sensor_Current_Limit == 20){
		sensitivity = 0.1;
		currentdischargelimit = -19.5;
	}
	else if(Sensor_Current_Limit == 30){
		sensitivity = 0.06741;
		currentdischargelimit = -29.5;
	}

    while(1) {
    	sumADC_voltage1 = 0;
    	sumADC_current1 = 0;
    	sumADC_voltage2 = 0;
    	sumADC_current2 = 0;

    	/*starts collecting ADC readout data with DMA*/
    	HAL_ADC_Start_DMA(&hadc1, adcBuffer, 4);
    	for (int i = 0; i < 50; i++) {
    		sumADC_voltage1 += adcBuffer[0];
    		sumADC_voltage2 += adcBuffer[1];
    		sumADC_current1 += adcBuffer[2];
    		sumADC_current2 += adcBuffer[3];
    		tx_thread_sleep(10);
    	}
    	HAL_ADC_Stop_DMA(&hadc1);

    	/*ADC read average counter*/
    	avg_voltage1 = sumADC_voltage1 / 50;
    	avg_current1 = sumADC_current1 / 50;
    	avg_voltage2 = sumADC_voltage2 / 50;
    	avg_current2 = sumADC_current2 / 50;

    	/*mapping ADC readout to 8bit*/
    	value_voltage1 = ((avg_voltage1 - 64) * 1023) / (3953 - 64);
    	value_current1 = ((avg_current1 - 60) * 1023) / (4095 - 60);
    	value_voltage2 = ((avg_voltage2 - 67) * 1023) / (3960 - 67);
    	value_current2 = ((avg_current2 - 61) * 1023) / (4095 - 61);

    	/*gives the limit voltage reading*/
    	if(value_voltage1 > 3882){
    		value_voltage1 = 0;
    	}
    	if(value_voltage2 > 3886){
    		value_voltage2 = 0;
    	}

    	/*convert digital value to voltage battery 1*/
    	voltage1 = (value_voltage1 * 14.6) / 1023;
    	voltage_current1 = (value_current1 * 3.253) / 1023;
    	/*converts voltage to battery current value 1*/
    	current1 = ((voltage_current1 - 2.5121) / sensitivity);

    	/*convert digital value to voltage battery 2*/
    	voltage2 = (value_voltage2 * 14.6) / 1023;
    	voltage_current2 = (value_current2 * 3.253) / 1023;
    	/*converts voltage to battery current value 2*/
    	current2 = ((voltage_current2 - 2.5153) / sensitivity);

    	/*ensure there is no current when the voltage is zero*/
    	if(voltage1 <= 1){
    		current1 = 0.0;
    	}
    	if(voltage2 <= 1){
    		current2 = 0.0;
    	}

//    	printf("voltage_current1: %.4f | ", voltage_current1);
//    	printf("voltage_current2: %.4f \n", voltage_current2);
    	/*protection in case of overcurrent*/
    	if(current1 <= currentdischargelimit || current2 <= currentdischargelimit){
    		/*disconnects the power flow if the current exceeds the limit*/
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
    		Beep_Beep(15,50,50);
    	}
    }
}

/*operating system for power management and calculating stage of charge*/
void Power_Management(ULONG initial_input) {
	float CurrentFiltered1,CurrentFiltered2;
	while(1) {
		/*Current batt1 Consumption Algorithm*/
		SoC_Temp_current_time1 = tx_time_get();
		if (SoC_Temp_current_time1 - SoC_prev_Temp_current_time1 >= 1000) {
			CurrentFiltered1 = 0.2 * current1 + 0.8 * CurrentFiltered1;
			if (current1 > 0.05){
				AH_Restored1 += (CurrentFiltered1 / 3600);
				if(current1 > 0.2){
					st1 = 1;
				}
			}
			else if (current1 < 0.05) {
				if (setzerobatt1 == 1 && SoC1 == 100){
					AH_Consumed1 = 0;
					AH_Restored1 = 0;
					setzerobatt1 = 0;
				}
				AH_Consumed1 -= (CurrentFiltered1 / 3600);
				if(current1 < 0.1){
					st1 = 0;
				}
			}
			SoC1 = ((BatteryCapacity1 - AH_Consumed1 + AH_Restored1) / BatteryCapacity1) * 100;
			if (SoC1 > 100.0) {
				SoC1 = 100.0;
			}
			else if(SoC1 < 0.0) {
				SoC1 = 0;
			}
			SoC_prev_Temp_current_time1 = SoC_Temp_current_time1;
		}

		/*charging logic battery 1*/
		if(SoC1 <= SoC_End_Discharge){
			//Beep_Beep(1,100,50);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			/*power transfer process from battery 1 to battery 2*/
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			activePowerSource = 2;
			/*start charging if battery1 capacity is 20%*/
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
		}
		else if(SoC1 == 100.0){
			/*stop charging when battery1 capacity is 100%*/
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
		}

		/*Current batt2 Consumption Algorithm*/
		SoC_Temp_current_time2 = tx_time_get();
		if (SoC_Temp_current_time2 - SoC_prev_Temp_current_time2 >= 1000) {
			CurrentFiltered2 = 0.2 * current2 + 0.8 * CurrentFiltered2;
			if(current2 > 0.05) {
				AH_Restored2 += (CurrentFiltered2 / 3600);
				if(current2 > 0.2){
					st2 = 1;
				}
			}
			else if (current2 < 0.05) {
				if (setzerobatt2 == 1 && SoC2 == 100){
					AH_Consumed2 = 0;
					AH_Restored2 = 0;
					setzerobatt2 = 0;
				}
				AH_Consumed2 -= (CurrentFiltered2 / 3600);
				if(current2 < 0.1){
					st2 = 0;
				}
			}
			SoC2 = ((BatteryCapacity2 - AH_Consumed2 + AH_Restored2) / BatteryCapacity2) * 100;
			if (SoC2 > 100.0) {
				SoC2 = 100.0;
			}
			else if(SoC2 < 0.0) {
				SoC2 = 0;
			}
			SoC_prev_Temp_current_time2 = SoC_Temp_current_time2;
		}

		/*charging logic battery 1*/
		if(SoC2 <= SoC_End_Discharge){
			//Beep_Beep(1,100,50);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
			/*power transfer process from battery 2 to battery */
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			activePowerSource = 1;
			/*start charging if battery2 capacity is 20%*/
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		}
		else if(SoC2 == 100.0){
			/*stop charging when battery2 capacity is 100%*/
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
		}
	tx_thread_sleep(500);
	}
}

/*operating system to calculate the Stage of Healty value */
void SoH_Management(ULONG initial_input) {
	capacity_actual1 = BatteryCapacity1;
	float capacity_used1 = 0;
	float capacity_charged1 = 0;
	int cycle_completed1 = 0;
	float cycle_fraction1 = 0;
	float cycle_health_factor1 = 1.0;

	capacity_actual2 = BatteryCapacity2;
	float capacity_used2 = 0;
	float capacity_charged2 = 0;
	int cycle_completed2 = 0;
	float cycle_fraction2 = 0;
	float cycle_health_factor2 = 1.0;

	uint16_t max_cycles = 2000;
	int last_st1 = -1, last_st2 = -1;

	Read_Data(&SoH1, address_SoH1, sizeof(SoH1));
	Read_Data(&cycle_count1, address_Cycle1, sizeof(cycle_count1));
	Read_Data(&SoH2, address_SoH2, sizeof(SoH2));
	Read_Data(&cycle_count2, address_Cycle2, sizeof(cycle_count2));

	while (1) {
		/*calculating the stage of Healty (SoH) battery 1*/
		if (st1 != last_st1) {
			if (st1 == 1) {
				capacity_used1 = AH_Consumed1;
			}
			else if (st1 == 0) {
				capacity_charged1 = AH_Restored1;
				capacity_actual1 += capacity_charged1;

				if (capacity_charged1 > 0) {
					capacity_actual1 -= capacity_used1;
					setzerobatt1 = 1;

					if (capacity_actual1 > BatteryCapacity1) {
						capacity_actual1 = BatteryCapacity1;
					} else if (capacity_actual1 < 0) {
						capacity_actual1 = 0;
					}

					cycle_completed1 = 1;
					cycle_fraction1 = ((capacity_used1 + capacity_charged1) / BatteryCapacity1) / 2;
					if (cycle_fraction1 > 1.0) cycle_fraction1 = 1.0;
					cycle_count1 += cycle_fraction1;
					Save_Data(&cycle_count1, address_Cycle1, sizeof(cycle_count1));

					if (cycle_count1 <= max_cycles) {
						cycle_health_factor1 = 1.0 - ((float)cycle_count1 / max_cycles);
					} else {
						cycle_health_factor1 = 0.0;
					}
				}
			}
			last_st1 = st1;
		}

		/*calculating the estimated state of health of battery 2*/
		if (cycle_completed1) {
			SoH1 = ((capacity_actual1 / BatteryCapacity1) * 100.0) * cycle_health_factor1;

			if (SoH1 > 100.0) {
				SoH1 = 100.0;
			} else if (SoH1 < 0.0) {
				SoH1 = 0.0;
			}

			Save_Data(&SoH1, address_SoH1, sizeof(SoH1));
			cycle_completed1 = 0;
		}

		/*calculating the stage of Healty (SoH) battery 1*/
		if (st2 != last_st2) {
			if (st2 == 1) {
				capacity_used2 = AH_Consumed2;
			}
			else if (st2 == 0) {
				capacity_charged2 = AH_Restored2;
				capacity_actual2 += capacity_charged2;

				if (capacity_charged2 > 0) {
					capacity_actual2 -= capacity_used2;
					setzerobatt2 = 1;

					if (capacity_actual2 > BatteryCapacity2) {
						capacity_actual2 = BatteryCapacity2;
					} else if (capacity_actual2 < 0) {
						capacity_actual2 = 0;
					}

					cycle_completed2 = 1;
					cycle_fraction2 = ((capacity_used2 + capacity_charged2) / BatteryCapacity2) / 2;
					if (cycle_fraction2 > 1.0) cycle_fraction2 = 1.0;
					cycle_count2 += cycle_fraction2;
					Save_Data(&cycle_count2, address_Cycle2, sizeof(cycle_count2));

					if (cycle_count2 <= max_cycles) {
						cycle_health_factor2 = 1.0 - ((float)cycle_count2 / max_cycles);
					} else {
						cycle_health_factor2 = 0.0;
					}
				}
			}
			last_st2 = st2;
		}

		/*calculating the estimated state of health of battery 2*/
		if (cycle_completed2) {
			SoH2 = ((capacity_actual2 / BatteryCapacity2) * 100.0) * cycle_health_factor2;

			if (SoH2 > 100.0) {
				SoH2 = 100.0;
			} else if (SoH2 < 0.0) {
				SoH2 = 0.0;
			}

			Save_Data(&SoH2, address_SoH2, sizeof(SoH2));
			cycle_completed2 = 0;
		}

		printf("V1 : %.2f |", voltage1);
		printf("C1 : %.2f |", current1);
		printf("AH_Consumed1 : %.4f Ah |", AH_Consumed1);
		printf("AH_Restored1: %.4f Ah |", AH_Restored1);
		printf("C_charged: %.4f Ah |", capacity_charged1);
		printf("C_used: %.4f Ah |", capacity_used1);
		printf("C_actua1: %.4f Ah |", capacity_actual1);
		printf("Cycle: %.2f |", cycle_count1);
		printf("SoH1: %d %% |", SoH1);
		printf("SoC1: %.2f |", SoC1);
		printf("PowerON: %d |", activePowerSource);
		printf("st1: %d\n", st1);

		printf("V2 : %.2f |", voltage2);
		printf("C2 : %.2f |", current2);
		printf("AH_Consumed2 : %.4f Ah |", AH_Consumed2);
		printf("AH_Restored2: %.4f Ah |", AH_Restored2);
		printf("C_charged: %.4f Ah |", capacity_charged2);
		printf("C_used: %.4f Ah |", capacity_used2);
		printf("C_actua1: %.4f Ah |", capacity_actual2);
		printf("Cycle: %.2f |", cycle_count2);
		printf("SoH2: %d %% |", SoH2);
		printf("SoC2: %.2f |", SoC2);
		printf("PowerON: %d |", activePowerSource);
		printf("st2: %d |", st2);
		printf("temp: %.2f\n\n", temperature);
		tx_thread_sleep(1000);
	}
}

/*operating system to manage LED*/
void Set_LED(ULONG initial_input) {
	Beep_Beep(2,100,50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	cycle_count2 = 0;
//	Save_Data(&cycle_count2, address_Cycle1, sizeof(cycle_count2));
//	Save_Data(&cycle_count2, address_Cycle2, sizeof(cycle_count2));
//	SoH1 = 100;
//	Save_Data(&SoH1, address_SoH1, sizeof(SoH1));
//	Save_Data(&SoH1, address_SoH2, sizeof(SoH1));

	while(1) {
		for(int i = 0; i < 4095; i++){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
			tx_thread_sleep(1);
		}
		for(int i = 4095; i > 0; i--){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, i);
			tx_thread_sleep(1);
		}
	}
}

/*operating system for sending data with half duplex single wire communication*/
void Transmit_Data(ULONG initial_input) {
    uint8_t requestBuffer[0];
    char buffer[64];
    while (1) {
        /*Receive request from UART*/
        HAL_HalfDuplex_EnableReceiver(&huart2);
        HAL_UART_Receive(&huart2, requestBuffer, 1, 1000);

        /*Data send command*/
        if (requestBuffer[0] == 0x6A) {
        	tx_thread_sleep(10);
            int len = snprintf(buffer, sizeof(buffer),
                "%.1f#%-5.1f#%-4d#%d#%-3d#%.1f#%-5.1f#%-4d#%d#%-3d#%.1f %d",
                voltage1, current1, (int)round(SoC1), st1, SoH1,
				voltage2, current2, (int)round(SoC2), st2, SoH2,
				temperature, activePowerSource);
            HAL_HalfDuplex_EnableTransmitter(&huart2);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
        }
        tx_thread_sleep(5);
    }
}

/*operating system to retrieve data from temperature sensor from DS18B20*/
void Temperature_Reading(ULONG initial_input) {
	HAL_TIM_Base_Start(&htim2);
    while (1) {
        temperature = DS18B20_GetTemp();
        tx_thread_sleep(1000);
    }
}

/*Function Group Zone*/
/*Function to produce a beep sound on the buzzer*/
void Beep_Beep(uint8_t cycle, uint16_t delay1, uint16_t delay2) {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	for (int i = 0; i < cycle; i++) {
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 3000);
		tx_thread_sleep(delay1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		tx_thread_sleep(50);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
		tx_thread_sleep(delay2);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		tx_thread_sleep(50);
	}
}

/*Function to erase and write values to flash memory*/
void Save_Data(void *value, uint32_t address, size_t size) {
    uint8_t write_enable_cmd = 0x06;
    uint8_t data[size];
    memcpy(data, value, size);

    /*Erase Sector Function (4 KB)*/
    uint8_t erase_cmd[4];
    erase_cmd[0] = 0x20;
    erase_cmd[1] = (address >> 16) & 0xFF;
    erase_cmd[2] = (address >> 8) & 0xFF;
    erase_cmd[3] = address & 0xFF;

    /*Write Enable Function*/
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, erase_cmd, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    tx_thread_sleep(100);

    /*Command Write Data*/
    uint8_t write_cmd[4];
    write_cmd[0] = 0x02;
    write_cmd[1] = (address >> 16) & 0xFF;
    write_cmd[2] = (address >> 8) & 0xFF;
    write_cmd[3] = address & 0xFF;

    /*Perintah Write Enable*/
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &write_enable_cmd, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    tx_thread_sleep(50);
}

void Read_Data(void *buffer, uint32_t address, size_t size) {
    uint8_t read_cmd[4];
    read_cmd[0] = 0x03; //Command to read
    read_cmd[1] = (address >> 16) & 0xFF;
    read_cmd[2] = (address >> 8) & 0xFF;
    read_cmd[3] = address & 0xFF;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, read_cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/* USER CODE END 1 */
