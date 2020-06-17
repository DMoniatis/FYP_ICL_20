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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PwrBtn_Pin GPIO_PIN_13
#define PwrBtn_GPIO_Port GPIOC
#define PwrBtn_EXTI_IRQn EXTI15_10_IRQn
#define Boost_Pin GPIO_PIN_0
#define Boost_GPIO_Port GPIOC
#define Buck_Pin GPIO_PIN_1
#define Buck_GPIO_Port GPIOC
#define VC3_Pin GPIO_PIN_0
#define VC3_GPIO_Port GPIOA
#define I_Chg_Pin GPIO_PIN_1
#define I_Chg_GPIO_Port GPIOA
#define I_Dchg_Pin GPIO_PIN_2
#define I_Dchg_GPIO_Port GPIOA
#define VC6_Pin GPIO_PIN_3
#define VC6_GPIO_Port GPIOA
#define Vin_Pin GPIO_PIN_4
#define Vin_GPIO_Port GPIOA
#define Temp1_Pin GPIO_PIN_5
#define Temp1_GPIO_Port GPIOA
#define Temp2_Pin GPIO_PIN_6
#define Temp2_GPIO_Port GPIOA
#define Temp3_Pin GPIO_PIN_7
#define Temp3_GPIO_Port GPIOA
#define Temp4_Pin GPIO_PIN_4
#define Temp4_GPIO_Port GPIOC
#define Temp_En_Pin GPIO_PIN_5
#define Temp_En_GPIO_Port GPIOC
#define CB_En_Pin GPIO_PIN_0
#define CB_En_GPIO_Port GPIOB
#define VC1_Pin GPIO_PIN_1
#define VC1_GPIO_Port GPIOB
#define VC_En_Pin GPIO_PIN_2
#define VC_En_GPIO_Port GPIOB
#define VC0_Pin GPIO_PIN_12
#define VC0_GPIO_Port GPIOB
#define VC4_Pin GPIO_PIN_13
#define VC4_GPIO_Port GPIOB
#define VC2_Pin GPIO_PIN_14
#define VC2_GPIO_Port GPIOB
#define VC5_Pin GPIO_PIN_15
#define VC5_GPIO_Port GPIOB
#define BalC1_Pin GPIO_PIN_6
#define BalC1_GPIO_Port GPIOC
#define BalC2_Pin GPIO_PIN_7
#define BalC2_GPIO_Port GPIOC
#define BalC3_Pin GPIO_PIN_8
#define BalC3_GPIO_Port GPIOC
#define BalC4_Pin GPIO_PIN_9
#define BalC4_GPIO_Port GPIOC
#define BalC5_Pin GPIO_PIN_8
#define BalC5_GPIO_Port GPIOA
#define BalC6_Pin GPIO_PIN_9
#define BalC6_GPIO_Port GPIOA
#define CANmode_Pin GPIO_PIN_10
#define CANmode_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define timingPin_Pin GPIO_PIN_15
#define timingPin_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
