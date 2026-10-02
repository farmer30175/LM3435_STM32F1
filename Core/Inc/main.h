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
#include "stm32f1xx_hal.h"

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
#define VCP_CMD_BUFFER_SIZE   64

/* USB CDC command queue (usbd_cdc_if.c): filled by the USB IRQ, drained by the
   main loop. VCP_QueuePop() returns 1 and fills dst when a command is ready. */
uint8_t         VCP_QueuePop(char *dst);

void            VCP_Send(const char *s);
void            VCP_Printf(const char *fmt, ...);
void            Process_Command(char *cmd);

HAL_StatusTypeDef   LM3435_WriteReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef   LM3435_ReadReg(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef   LM3435_Set_RGB_Current(uint16_t r_val, uint16_t g_val, uint16_t b_val);
void            LM3435_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
