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
#include "stm32f7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DI2_PULSE_Pin GPIO_PIN_2
#define DI2_PULSE_GPIO_Port GPIOE
#define DI3_PULSE_Pin GPIO_PIN_3
#define DI3_PULSE_GPIO_Port GPIOE
#define DI4_Pin GPIO_PIN_4
#define DI4_GPIO_Port GPIOE
#define DI5_Pin GPIO_PIN_5
#define DI5_GPIO_Port GPIOE
#define DI6_Pin GPIO_PIN_6
#define DI6_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define DO1_Pin GPIO_PIN_1
#define DO1_GPIO_Port GPIOF
#define DO2_Pin GPIO_PIN_2
#define DO2_GPIO_Port GPIOF
#define DO3_Pin GPIO_PIN_3
#define DO3_GPIO_Port GPIOF
#define DO4_Pin GPIO_PIN_4
#define DO4_GPIO_Port GPIOF
#define DO5_Pin GPIO_PIN_5
#define DO5_GPIO_Port GPIOF
#define DO6_Pin GPIO_PIN_6
#define DO6_GPIO_Port GPIOF
#define DO7_Pin GPIO_PIN_7
#define DO7_GPIO_Port GPIOF
#define DO_ENABLE_Pin GPIO_PIN_8
#define DO_ENABLE_GPIO_Port GPIOF
#define DO_FAULT_Pin GPIO_PIN_9
#define DO_FAULT_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define AI6_Pin GPIO_PIN_0
#define AI6_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define AO_MISO_Pin GPIO_PIN_2
#define AO_MISO_GPIO_Port GPIOC
#define AI7_Pin GPIO_PIN_3
#define AI7_GPIO_Port GPIOC
#define AI0_Pin GPIO_PIN_0
#define AI0_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define AI1_Pin GPIO_PIN_3
#define AI1_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_4
#define AI2_GPIO_Port GPIOA
#define AI3_Pin GPIO_PIN_5
#define AI3_GPIO_Port GPIOA
#define AI4_Pin GPIO_PIN_6
#define AI4_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define AI5_Pin GPIO_PIN_1
#define AI5_GPIO_Port GPIOB
#define DO_LOAD_Pin GPIO_PIN_11
#define DO_LOAD_GPIO_Port GPIOF
#define DI7_Pin GPIO_PIN_7
#define DI7_GPIO_Port GPIOE
#define DI8_Pin GPIO_PIN_8
#define DI8_GPIO_Port GPIOE
#define DI9_Pin GPIO_PIN_9
#define DI9_GPIO_Port GPIOE
#define DI10_Pin GPIO_PIN_10
#define DI10_GPIO_Port GPIOE
#define DI11_Pin GPIO_PIN_11
#define DI11_GPIO_Port GPIOE
#define DI12_Pin GPIO_PIN_12
#define DI12_GPIO_Port GPIOE
#define DI13_Pin GPIO_PIN_13
#define DI13_GPIO_Port GPIOE
#define DI14_Pin GPIO_PIN_14
#define DI14_GPIO_Port GPIOE
#define DI15_Pin GPIO_PIN_15
#define DI15_GPIO_Port GPIOE
#define AO_SCK_Pin GPIO_PIN_10
#define AO_SCK_GPIO_Port GPIOB
#define AO_CS1_Pin GPIO_PIN_11
#define AO_CS1_GPIO_Port GPIOB
#define AO_CS2_Pin GPIO_PIN_12
#define AO_CS2_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define AO_MOSI_Pin GPIO_PIN_15
#define AO_MOSI_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define uSD_Detect_Pin GPIO_PIN_3
#define uSD_Detect_GPIO_Port GPIOD
#define RS485_DE_Pin GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOD
#define RS485_TX_Pin GPIO_PIN_5
#define RS485_TX_GPIO_Port GPIOD
#define RS485_RX_Pin GPIO_PIN_6
#define RS485_RX_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define DI0_PULSE_Pin GPIO_PIN_0
#define DI0_PULSE_GPIO_Port GPIOE
#define DI1_PULSE_Pin GPIO_PIN_1
#define DI1_PULSE_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
