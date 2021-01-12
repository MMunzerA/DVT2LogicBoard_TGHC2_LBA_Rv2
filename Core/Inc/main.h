/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void jump_to_bootloader(void);
void Reboot(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC32_OUT_Pin GPIO_PIN_14
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_15
#define OSC32_IN_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define OP1_Pin GPIO_PIN_0
#define OP1_GPIO_Port GPIOC
#define OP2_Pin GPIO_PIN_1
#define OP2_GPIO_Port GPIOC
#define OP3_Pin GPIO_PIN_2
#define OP3_GPIO_Port GPIOC
#define WKUP_Pin GPIO_PIN_0
#define WKUP_GPIO_Port GPIOA
#define OP6_Pin GPIO_PIN_1
#define OP6_GPIO_Port GPIOA
#define OP5_Pin GPIO_PIN_2
#define OP5_GPIO_Port GPIOA
#define OP4_Pin GPIO_PIN_3
#define OP4_GPIO_Port GPIOA
#define SCK1_R_Pin GPIO_PIN_5
#define SCK1_R_GPIO_Port GPIOA
#define MISO1_Pin GPIO_PIN_6
#define MISO1_GPIO_Port GPIOA
#define MOSI1_R_Pin GPIO_PIN_7
#define MOSI1_R_GPIO_Port GPIOA
#define R7_Pin GPIO_PIN_5
#define R7_GPIO_Port GPIOC
#define R8_Pin GPIO_PIN_0
#define R8_GPIO_Port GPIOB
#define R6_Pin GPIO_PIN_1
#define R6_GPIO_Port GPIOB
#define CS020712_Pin GPIO_PIN_2
#define CS020712_GPIO_Port GPIOB
#define R4_Pin GPIO_PIN_10
#define R4_GPIO_Port GPIOB
#define CS030813_Pin GPIO_PIN_12
#define CS030813_GPIO_Port GPIOB
#define SCK2_R_Pin GPIO_PIN_13
#define SCK2_R_GPIO_Port GPIOB
#define MISO2_Pin GPIO_PIN_14
#define MISO2_GPIO_Port GPIOB
#define MOSI2_R_Pin GPIO_PIN_15
#define MOSI2_R_GPIO_Port GPIOB
#define CS040914_Pin GPIO_PIN_7
#define CS040914_GPIO_Port GPIOC
#define CS051015_Pin GPIO_PIN_8
#define CS051015_GPIO_Port GPIOC
#define CS010611_Pin GPIO_PIN_9
#define CS010611_GPIO_Port GPIOC
#define R3_Pin GPIO_PIN_8
#define R3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_15
#define R2_GPIO_Port GPIOA
#define SCK3_R_Pin GPIO_PIN_10
#define SCK3_R_GPIO_Port GPIOC
#define MISO3_Pin GPIO_PIN_11
#define MISO3_GPIO_Port GPIOC
#define MOSI3_R_Pin GPIO_PIN_12
#define MOSI3_R_GPIO_Port GPIOC
#define R9_Pin GPIO_PIN_2
#define R9_GPIO_Port GPIOD
#define R10_Pin GPIO_PIN_3
#define R10_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_4
#define R1_GPIO_Port GPIOB
#define R5_Pin GPIO_PIN_5
#define R5_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


typedef struct
{
  uint16_t Pin;
  GPIO_TypeDef* Port;
} pin_port_s;
void chip_unselect(uint8_t id);
void chip_select(uint8_t id);
pin_port_s get_chip_pin(uint8_t id);

#define L_FIRMWARE_VERSION 6
// Message type
#define HUMIDITY_REQ             (uint8_t)0x01
#define TEMPERATURE_REQ          (uint8_t)0x02
#define FORCE_REQ                (uint8_t)0x03
#define DEVICEID_REQ             (uint8_t)0x04
#define HUMIDITY_REF_REQ         (uint8_t)0x05
#define TEMPERATURE_REF_REQ      (uint8_t)0x06
#define ACCELERAMTER_REQ		 (uint8_t)0x07
#define ALL_REQS                 (uint8_t)0x0F

// Message (CMD) ID from interface ID
#define SET_DIGIPOT_MSG        		(uint8_t)0x0A
#define GET_ACL_SENSOR_VALUE_MSG    (uint8_t)0x0B
#define GET_REF_SENSOR_VALUE_MSG   	(uint8_t)0x0C
#define DEVICEID_REQUEST_MSG        (uint8_t)0x0D
#define DATA_REQUEST_MSG         	(uint8_t)0x0E
#define UPDATE_SENSOR_VALUE_MSG    	(uint8_t)0x0F

#define MOVE_TO_BOOTLOADER          (uint8_t)0x02
#define Reboot_LB                   (uint8_t)0x03

#define FORCE_DATA_REQUEST          (uint8_t)0x07
#define OTHER_DATA_REQUEST          (uint8_t)0x08

#define DATA_REPLY_MSG          	(uint8_t)0x00
#define DEVICEID_REPLY_MSG        	(uint8_t)0x01
#define REF_SENSOR_REPLY_MSG        (uint8_t)0x02
#define ACCELERAMETER_REPLY_MSG     (uint8_t)0x03

#define RECEIVER_ID	0xFE
#define INVALID_DEVICE_ID 0

#define TILES_IN_HORIZONTAL			2
#define F_ROWS_PERTILE				6
#define F_COLUMNS_PERTILE 			10
#define TH_ROWS_PERTILE				3
#define TH_COLUMNS_PERTILE  		5
#define TOTAL_F_SENSORS_PERTILE		(F_ROWS_PERTILE * F_COLUMNS_PERTILE)
#define TOTAL_TH_SENSORS_PERTILE	(TH_ROWS_PERTILE * TH_COLUMNS_PERTILE)

#define USING_HTS221_SENSOR_ISLAND

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;


extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
