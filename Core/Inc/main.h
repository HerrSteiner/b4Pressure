/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define Eg2Vol2_Pin GPIO_PIN_2
#define Eg2Vol2_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define EgTrigger_Pin GPIO_PIN_2
#define EgTrigger_GPIO_Port GPIOB
#define Osc1FM_Pin GPIO_PIN_11
#define Osc1FM_GPIO_Port GPIOF
#define PadMode1_Pin GPIO_PIN_12
#define PadMode1_GPIO_Port GPIOF
#define PadMode2_Pin GPIO_PIN_13
#define PadMode2_GPIO_Port GPIOF
#define PadMode3_Pin GPIO_PIN_14
#define PadMode3_GPIO_Port GPIOF
#define PadMode4_Pin GPIO_PIN_15
#define PadMode4_GPIO_Port GPIOF
#define Eg2Cut_Pin GPIO_PIN_7
#define Eg2Cut_GPIO_Port GPIOE
#define Eg2Morph_Pin GPIO_PIN_8
#define Eg2Morph_GPIO_Port GPIOE
#define Eg2FilterVol_Pin GPIO_PIN_9
#define Eg2FilterVol_GPIO_Port GPIOE
#define Osc1Filter_Pin GPIO_PIN_10
#define Osc1Filter_GPIO_Port GPIOE
#define Osc1Out2_Pin GPIO_PIN_11
#define Osc1Out2_GPIO_Port GPIOE
#define Osc2Filter_Pin GPIO_PIN_13
#define Osc2Filter_GPIO_Port GPIOE
#define Osc2Out2_Pin GPIO_PIN_14
#define Osc2Out2_GPIO_Port GPIOE
#define Osc2FM_Pin GPIO_PIN_15
#define Osc2FM_GPIO_Port GPIOE
#define Eg1FM1_Pin GPIO_PIN_10
#define Eg1FM1_GPIO_Port GPIOB
#define Eg1Wave1_Pin GPIO_PIN_11
#define Eg1Wave1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define Eg1Vol1_Pin GPIO_PIN_11
#define Eg1Vol1_GPIO_Port GPIOD
#define Eg1Cut_Pin GPIO_PIN_12
#define Eg1Cut_GPIO_Port GPIOD
#define Eg1Morph_Pin GPIO_PIN_13
#define Eg1Morph_GPIO_Port GPIOD
#define Eg1FilterVol_Pin GPIO_PIN_14
#define Eg1FilterVol_GPIO_Port GPIOD
#define Eg2FM2_Pin GPIO_PIN_15
#define Eg2FM2_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define FilterModus_Pin GPIO_PIN_6
#define FilterModus_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define Eg1Loop_Pin GPIO_PIN_8
#define Eg1Loop_GPIO_Port GPIOB
#define Eg2Loop_Pin GPIO_PIN_9
#define Eg2Loop_GPIO_Port GPIOB
#define Eg2Wave2_Pin GPIO_PIN_0
#define Eg2Wave2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
