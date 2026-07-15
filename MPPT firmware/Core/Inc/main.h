/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  STATE_INIT = 0,  /* Boot/configuration state before normal control starts. */
  STATE_IDLE,      /* PWM stopped, gate drivers disabled, measurements active. */
  STATE_MPPT_STARTUP,   /* Conservative first switching state before MPPT is allowed. */
  STATE_MPPT_RUN,       /* Normal closed-loop MPPT update state. */
  STATE_FAULT      /* Safe latched state entered after invalid feedback or PWM failure. */
} State_t;

typedef enum
{
  FAULT_NONE = 0,
  FAULT_ADC_READ,
  FAULT_ADC_RANGE,
  FAULT_PWM_START
} Fault_t;
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
const char *getStateName(State_t state);
const char *getFaultName(Fault_t fault);
State_t getState(void);
Fault_t getFault(void);
void requestMpptStart(void);
void requestStop(void);
void requestFaultReset(void);
void enterFault(Fault_t fault);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DE___RS485_Pin GPIO_PIN_0
#define DE___RS485_GPIO_Port GPIOF
#define I_IN_Pin GPIO_PIN_0
#define I_IN_GPIO_Port GPIOA
#define I_O_Pin GPIO_PIN_1
#define I_O_GPIO_Port GPIOA
#define V_O_Pin GPIO_PIN_4
#define V_O_GPIO_Port GPIOA
#define V_IN_Pin GPIO_PIN_5
#define V_IN_GPIO_Port GPIOA
#define RE___RS485_Pin GPIO_PIN_1
#define RE___RS485_GPIO_Port GPIOB
#define DQ__Temp_Pin GPIO_PIN_10
#define DQ__Temp_GPIO_Port GPIOA
#define BST_STP_Pin GPIO_PIN_3
#define BST_STP_GPIO_Port GPIOB
#define BCK_STP_Pin GPIO_PIN_4
#define BCK_STP_GPIO_Port GPIOB
#define BCK_DIS_Pin GPIO_PIN_5
#define BCK_DIS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BST_DIS_Pin GPIO_PIN_1
#define BST_DIS_GPIO_Port GPIOF

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
