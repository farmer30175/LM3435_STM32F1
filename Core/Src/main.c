/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- LM3435 (SNVS724C) ---- */
/* 7-bit slave address is 0x28 -> chip address byte 0x50 (write) */
#define LM3435_ADDR      (0x28<<1)

#define LM3435_REG_LEDLO   0x00    /* [5:4]=RLED[1:0] [3:2]=BLED[1:0] [1:0]=GLED[1:0] */
#define LM3435_REG_GLEDH   0x01    /* GLED[9:2]  */
#define LM3435_REG_BLEDH   0x02    /* BLED[9:2]  */
#define LM3435_REG_RLEDH   0x03    /* RLED[9:2]  */
#define LM3435_REG_FLT_RPT 0x05    /* [0]=fault reporting enable, reset default 0x01 */
#define LM3435_REG_DELAY   0x06    /* [7:6]RDLY [5:4]BDLY [3:2]GDLY -> 0=5us 1=15 2=25 3=35 */
#define LM3435_REG_FAULT   0x07    /* RO: GO GS - BO BS - RO RS */

/* ---- RGB sequential PWM geometry ----
 * f_TIM = 72MHz / (PSC+1) / (ARR+1) = 72e6 / 1 / 3600 = 20.000 kHz, 1 tick = 13.889 ns
 * 10% duty = 360 ticks = 5 us, three non-overlapping slots inside the 50 us frame:
 *   RED    PA0  TIM2_CH1 PWM1  CNT    0 ..  359   ( 0.00 -  5.00 us)
 *   GREEN  PA7  TIM3_CH2 PWM1  CNT 1200 .. 1559   (16.67 - 21.67 us)   (CNT preloaded)
 *   BLUE   PA2  TIM2_CH3 PWM2  CNT 3240 .. 3599   (45.00 - 50.00 us)
 * PA1 HAS NO TIM3 FUNCTION on STM32F103 (TIM3_CH2 is PA7 / PB5 / PC7 only),
 * so PA1 cannot generate the middle slot. PA1 offers only TIM2_CH2.
 * NOTE: LM3435 is a sequential driver and does NOT support overlapping control
 *       signals. Priority if they overlap is GREEN > BLUE > RED.
 */
#define PWM_ARR          3599
#define PWM_DUTY_TICKS   360
#define PWM_GREEN_PHASE  1200
#define PWM_BLUE_CCR     (PWM_ARR - PWM_DUTY_TICKS + 1)   /* 3240, PWM mode 2 */

/* Slot layout inside one 3600-tick frame:
      RED    CNT 0 ........... duty-1          (PWM mode 1, CCR = duty)
      GREEN  CNT 1200 ........ 1200+duty-1     (PWM mode 1, CCR = duty, CNT preloaded)
      BLUE   CNT 3600-duty ... 3599           (PWM mode 2, CCR = 3600-duty)
   GREEN is pinned to PWM_GREEN_PHASE (= 1/3 of the frame) so the three slots stay
   ordered for any duty.

   The real ceiling is NOT 1/3 of the frame. LM3435 enforces a transition delay at
   every colour change (06h DELAY, minimum value 0 = 5us) and one frame contains
   three changes (R->G, G->B, B->R), so 15us of every frame is reserved as dead
   time no matter how the slots are packed:
       duty_max = (frame - 15us) / 3 / frame
   With a 50us frame (20kHz): (50 - 15) / 3 / 50 = 23.3%.
   Note that raising the PWM frequency makes this WORSE, because the fixed 15us
   dead time eats a larger fraction of a shorter frame. Going higher would require
   a longer frame: 33% needs 1500us = 667Hz, which flickers visibly.
   23% = 828 ticks; round down to 810 (22.5%) so the total never exceeds the frame. */
#define PWM_DUTY_MAX_TICKS 810U
#define PWM_DUTY_MAX_PCT   23U

#define LM3435_I2C_TIMEOUT  100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* The boot banner is generated long before the host finishes enumerating the
   CDC device, so VCP_Send() drops it. Set once we have managed to actually
   deliver something, then re-send the banner on the first command. */
static uint8_t vcp_link_up = 0U;
static uint8_t vcp_banner_sent = 0U;
static uint8_t vcp_cmd_seen = 0U;
static uint32_t vcp_hb_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void MX_RGB_PWM_Start(void);
static uint8_t PWM_SetDuty(uint8_t duty_pct);
static void VCP_Process(void);
static void VCP_SendBanner(void);
static void LM3435_PrintFault(uint8_t flt);
static HAL_StatusTypeDef LM3435_ClearFault(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(500);

	/* program inter-color transition delay to 5us (register 06h = 0x00) */
	LM3435_Init();

	/* Non-overlapping RGB PWM. MUST keep running: I2C current updates and
	   LED fault detection only act on the falling edge of the CTRL signals. */
	MX_RGB_PWM_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	VCP_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_ARR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_DUTY_TICKS;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* CH3 uses PWM mode 2 so the BLUE pulse sits at the END of the frame
     (CNT 3240..3599) and does not overlap the RED pulse at the start. */
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = PWM_BLUE_CCR;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_DUTY_TICKS;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ------------------------------------------------------------------ */
/*  VCP (USB CDC) helpers                                             */
/* ------------------------------------------------------------------ */
void VCP_Send(const char *s)
{
    uint32_t wait;

    if ((s == NULL) || (*s == '\0'))
    {
        return;
    }
    /* CDC_Transmit_FS dereferences pClassData - it is still NULL until the
       host enumerates the device, so drop the message instead of faulting. */
    if (hUsbDeviceFS.pClassData == NULL)
    {
        return;
    }

    /* The CDC class keeps a single TX buffer with a TxState busy flag. A second
       send issued while the previous packet is still in flight returns USBD_BUSY
       and used to be dropped silently - which is why commands appeared to get
       "no reply". Retry until the previous packet is out. Safe to block: VCP
       code only ever runs from the main loop. */
    for (wait = 0U; wait < 2000U; wait++)
    {
        if (CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s)) != USBD_BUSY)
        {
            vcp_link_up = 1U;
            return;
        }
        HAL_Delay(1);
    }
}

void VCP_Printf(const char *fmt, ...)
{
    /* static: USB transmission is asynchronous, so the buffer must stay valid
       after this function returns. A stack buffer could be overwritten by the
       main loop while the packet is still in flight. */
    static char buf[128];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    VCP_Send(buf);
}

/* called from the main loop only - never from the USB IRQ */
static void VCP_Process(void)
{
    static char cmd[VCP_CMD_BUFFER_SIZE];
    uint32_t now = HAL_GetTick();

    /* Liveness check: one dot per second until the first command is received.
       If nothing shows up at all, the firmware is not running / not flashed,
       or the terminal is not actually connected to this CDC port. */
    if ((vcp_cmd_seen == 0U) && ((now - vcp_hb_tick) >= 1000U))
    {
        vcp_hb_tick = now;
        VCP_Send(".");
    }

    /* Dispatch on a line ending, or after 500 ms of typing silence so the
       command still runs on a terminal that sends no newline at all. */
    if (VCP_QueuePop(cmd) == 0U)
    {
        return;
    }

    vcp_cmd_seen = 1U;

    /* Proves the whole RX->main loop->TX chain works, and re-delivers the boot
       banner that was lost before the host enumerated the CDC device. */
    if (vcp_banner_sent == 0U)
    {
        VCP_SendBanner();
    }

    Process_Command(cmd);
}

/* ------------------------------------------------------------------ */
/*  LM3435 I2C helpers                                                */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef LM3435_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, LM3435_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &data, 1, LM3435_I2C_TIMEOUT);
}

HAL_StatusTypeDef LM3435_ReadReg(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, LM3435_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            data, 1, LM3435_I2C_TIMEOUT);
}

void LM3435_Init(void)
{
    HAL_StatusTypeDef st;

    /* Reset values of 00h-03h are 0x3FF = full current, so the LEDs are driven
       at maximum from power-up until the first I2C write. Drop to the minimum
       code first to avoid an overcurrent spike at boot. A zero code can raise a
       false SHORT fault, so use code 1. */
    if (LM3435_Set_RGB_Current(1U, 1U, 1U) != HAL_OK)
    {
        VCP_Printf("ERR: LM3435 not responding (I2C err=0x%04X)\r\n",
                   (unsigned)HAL_I2C_GetError(&hi2c1));
        return;
    }

    /* 06h: RDLY/BDLY/GDLY = 0 -> 5us transition delay.
       Default 0xFF = 35us, which is longer than one 20kHz slot (16.7us). */
    st = LM3435_WriteReg(LM3435_REG_DELAY, 0x00U);
    if (st != HAL_OK)
    {
        VCP_Printf("ERR: LM3435 not responding (I2C err=0x%04X)\r\n",
                   (unsigned)HAL_I2C_GetError(&hi2c1));
        return;
    }

    /* 05h bit0 = 1 keeps LED open/short fault reporting enabled (default 0x01) */
    (void)LM3435_WriteReg(LM3435_REG_FLT_RPT, 0x01U);

    VCP_Send("LM3435 ready (addr 0x28, DELAY=5us, FAULT report on, boot current clamped)\r\n");
}

HAL_StatusTypeDef LM3435_Set_RGB_Current(uint16_t r_val, uint16_t g_val, uint16_t b_val)
{
    uint8_t reg00, reg01, reg02, reg03;
    HAL_StatusTypeDef st;

    if (r_val > 1023U) { r_val = 1023U; }
    if (g_val > 1023U) { g_val = 1023U; }
    if (b_val > 1023U) { b_val = 1023U; }

    /* 00h LEDLO : [7:6]=0 [5:4]=RLED[1:0] [3:2]=BLED[1:0] [1:0]=GLED[1:0] */
    reg00 = (uint8_t)((((r_val) & 0x03U) << 4) |
                      (((b_val) & 0x03U) << 2) |
                       ((g_val) & 0x03U));
    reg01 = (uint8_t)(g_val >> 2);   /* 01h GLEDH */
    reg02 = (uint8_t)(b_val >> 2);   /* 02h BLEDH */
    reg03 = (uint8_t)(r_val >> 2);   /* 03h RLEDH */

    /* The datasheet only documents "addr + register + single data byte".
       No auto address increment is specified, so write each register alone. */
    st = LM3435_WriteReg(LM3435_REG_LEDLO, reg00);
    if (st != HAL_OK) { return st; }
    st = LM3435_WriteReg(LM3435_REG_GLEDH, reg01);
    if (st != HAL_OK) { return st; }
    st = LM3435_WriteReg(LM3435_REG_BLEDH, reg02);
    if (st != HAL_OK) { return st; }
    return LM3435_WriteReg(LM3435_REG_RLEDH, reg03);
}

/* 07h FAULT : D7 GO  D6 GS  D5 -  D4 BO  D3 BS  D2 -  D1 RO  D0 RS
   (bit set = that fault is latched) */
static void LM3435_PrintFault(uint8_t flt)
{
    if (flt == 0x00U)
    {
        VCP_Send("none\r\n");
        return;
    }

    VCP_Printf("0x%02X : ", flt);
    if ((flt & 0x80U) != 0U) { VCP_Send("GREEN-open  "); }
    if ((flt & 0x40U) != 0U) { VCP_Send("GREEN-short "); }
    if ((flt & 0x10U) != 0U) { VCP_Send("BLUE-open   "); }
    if ((flt & 0x08U) != 0U) { VCP_Send("BLUE-short  "); }
    if ((flt & 0x02U) != 0U) { VCP_Send("RED-open    "); }
    if ((flt & 0x01U) != 0U) { VCP_Send("RED-short   "); }

    /* A commanded current of 0 leaves VOUT at VIN, which the datasheet calls
       a short (< VIN + 1.5V). Report that so it is not mistaken for a real
       fault. */
    if ((flt & (uint8_t)((flt & 0x48U) | (flt & 0x09U))) != 0U)
    {
        VCP_Send("\r\n    (SHORT flags are also reported when the current code is 0 - that is a false fault)\r\n");
        return;
    }
    VCP_Send("\r\n");
}

/* Datasheet: the latched fault is cleared by writing 0 to bit0 of 05h; write 1
   again afterwards to re-enable fault reporting. */
static HAL_StatusTypeDef LM3435_ClearFault(void)
{
    if (LM3435_WriteReg(LM3435_REG_FLT_RPT, 0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return LM3435_WriteReg(LM3435_REG_FLT_RPT, 0x01U);
}

/* ------------------------------------------------------------------ */
/*  Non-overlapping RGB PWM start                                     */
/* ------------------------------------------------------------------ */
static void MX_RGB_PWM_Start(void)
{
    /* Preload the counter BEFORE enabling CEN so the pulse lands in the
       correct slot of the 50us frame. */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* RED    PA0  CNT    0.. 359 */
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   /* BLUE   PA2  CNT 3240..3599 */

    __HAL_TIM_SET_COUNTER(&htim3, PWM_GREEN_PHASE);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* GREEN  PA7  CNT 1200..1559 */

    VCP_Printf("RGB PWM 20.00kHz duty 10.0%% : PA0/R=0us PA7/G=16.67us PA2/B=45us (non-overlap)\r\n");
}

/*  Change the duty of all three channels at runtime, in percent of one frame.
    Returns 1 on success, 0 if the argument is out of range.                */
static uint8_t PWM_SetDuty(uint8_t duty_pct)
{
    uint32_t ticks;

    if ((duty_pct == 0U) || (duty_pct > PWM_DUTY_MAX_PCT))
    {
        return 0U;
    }

    /* 1% = 36 ticks (3600 / 100). Clamp so that three slots plus the three
       5us transition delays always fit inside the 50us frame. */
    ticks = ((uint32_t)PWM_ARR + 1U) * (uint32_t)duty_pct / 100U;
    if (ticks > PWM_DUTY_MAX_TICKS)
    {
        ticks = PWM_DUTY_MAX_TICKS;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ticks);           /* RED   */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ticks);           /* GREEN */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_ARR - ticks + 1U); /* BLUE, PWM2 */

    VCP_Printf("OK: duty %u%% (%lu ticks, %.2f us per channel)\r\n",
               (unsigned)duty_pct, ticks, ((double)ticks * 50.0 / 3600.0));
    VCP_Printf("    slots: R=%lu G=%lu B=%lu ticks (total %lu of 3600)\r\n",
               ticks, (uint32_t)PWM_GREEN_PHASE, (uint32_t)PWM_ARR - ticks + 1U,
               ticks * 3U);
    VCP_Printf("    times : R=0.00us G=%.2fus B=%.2fus  (brightness = duty x current)\r\n",
               ((double)PWM_GREEN_PHASE * 50.0 / 3600.0),
               ((double)(PWM_ARR - ticks + 1U) * 50.0 / 3600.0));
    return 1U;
}

/* ------------------------------------------------------------------ */
/*  Command parser                                                    */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/*  Deferred boot banner                                               */
/* ------------------------------------------------------------------ */
static void VCP_SendBanner(void)
{
    vcp_banner_sent = 1U;
    VCP_Send("\r\n==== LM3435 RGB sequential driver ====\r\n");
    VCP_Send("PWM 20.00kHz duty 10.0% : PA0/R=0us  PA7/G=16.67us  PA2/B=45us\r\n");
    VCP_Send("LM3435 is controlled by CURRENT, duty is fixed at 10%.\r\n");
    VCP_Printf("Commands: PING | HELP | STATUS | CLR | DUTY <1..%u> | LM3435 <0..100>\r\n\r\n",
               (unsigned)PWM_DUTY_MAX_PCT);
}

/* ------------------------------------------------------------------ */
/*  Command parser                                                    */
/* ------------------------------------------------------------------ */
void Process_Command(char *cmd)
{
    int cmd_val = 0;
    uint16_t code;
    uint8_t  flt = 0xFFU;
    uint8_t  i;

    /* accept any case, so "lm3435 10" works as well as "LM3435 10" */
    for (i = 0U; cmd[i] != '\0'; i++)
    {
        if ((cmd[i] >= 'a') && (cmd[i] <= 'z'))
        {
            cmd[i] = (char)(cmd[i] - 32);
        }
    }

    if (strcmp(cmd, "PING") == 0)
    {
        VCP_Send("PONG\r\n");
        return;
    }

    if ((strcmp(cmd, "HELP") == 0) || (strcmp(cmd, "?") == 0))
    {
        VCP_Send("PING            - check the USB command link\r\n");
        VCP_Send("STATUS          - USB / PWM / I2C / LM3435 FAULT state\r\n");
        VCP_Send("CLR             - clear the latched LM3435 fault (05h)\r\n");
        VCP_Printf("DUTY <1..%u>    - PWM duty %% of one frame, all three channels\r\n",
                   (unsigned)PWM_DUTY_MAX_PCT);
        VCP_Send("                   max 23% at 20kHz: the LM3435 transition\r\n");
        VCP_Send("                   delay (5us x 3) takes 15us of every frame\r\n");
        VCP_Send("LM3435 <0..100> - set R,G,B LED current as %% of IREF\r\n");
        VCP_Send("                   (duty is fixed at 10%%, brightness = duty x current)\r\n");
        VCP_Send("                   do NOT use 0 to switch off: it raises a false\r\n");
        VCP_Send("                   SHORT fault. Use a low current instead.\r\n");
        return;
    }

    if (strcmp(cmd, "CLR") == 0)
    {
        if (LM3435_ClearFault() != HAL_OK)
        {
            VCP_Printf("ERR: I2C write failed (HAL err=0x%04X)\r\n",
                       (unsigned)HAL_I2C_GetError(&hi2c1));
            return;
        }
        HAL_Delay(2);
        if (LM3435_ReadReg(LM3435_REG_FAULT, &flt) == HAL_OK)
        {
            VCP_Send("fault cleared, FAULT(07h) now = ");
            LM3435_PrintFault(flt);
        }
        return;
    }

    if (strncmp(cmd, "DUTY ", 5) == 0)
    {
        int duty_val = 0;

        if ((sscanf(cmd + 5, "%d", &duty_val) != 1) ||
            (duty_val < 1) || (duty_val > (int)PWM_DUTY_MAX_PCT))
        {
            VCP_Printf("ERR: usage DUTY <1..%u> (got \"%s\")\r\n",
                       (unsigned)PWM_DUTY_MAX_PCT, cmd);
            VCP_Send("     LM3435 needs 5us per colour change and a frame has three\r\n");
            VCP_Send("     changes, so 15us of every 50us frame is dead time:\r\n");
            VCP_Send("         duty_max = (frame - 15us) / 3 / frame\r\n");
            VCP_Printf("         at 20kHz = (50 - 15) / 3 / 50 = 23%%\r\n");
            VCP_Send("     Duty 0 is also rejected: it makes LM3435 raise a false SHORT\r\n");
            VCP_Send("     fault. Raising the PWM frequency lowers this limit further.\r\n");
            return;
        }

        if (PWM_SetDuty((uint8_t)duty_val) == 0U)
        {
            VCP_Send("ERR: duty out of range\r\n");
        }
        return;
    }

    if (strcmp(cmd, "STATUS") == 0)
    {
        VCP_Send("USB CDC  : ");
        VCP_Send(vcp_link_up ? "enumerated\r\n" : "NOT enumerated\r\n");
        VCP_Printf("PWM      : 20.00 kHz, 50us frame, DUTY <1..%u> (15us dead time)\r\n",
                   (unsigned)PWM_DUTY_MAX_PCT);
        VCP_Send("Pins     : PA0=TIM2_CH1/R  PA7=TIM3_CH2/G  PA2=TIM2_CH3/B\r\n");
        VCP_Send("I2C1     : 100 kHz, addr 0x50, HAL err=0x");
        VCP_Printf("%04X\r\n", (unsigned)HAL_I2C_GetError(&hi2c1));
        if (LM3435_ReadReg(LM3435_REG_FAULT, &flt) == HAL_OK)
        {
            VCP_Send("LM3435   : ACK, FAULT(07h)=");
            LM3435_PrintFault(flt);
        }
        else
        {
            VCP_Printf("LM3435   : NO ACK on 0x50, HAL err=0x%04X\r\n",
                       (unsigned)HAL_I2C_GetError(&hi2c1));
        }
        return;
    }

    if (sscanf(cmd, "LM3435 %d", &cmd_val) != 1)
    {
        VCP_Printf("ERR: unknown command \"%s\"\r\n", cmd);
        VCP_Send("     type HELP for the command list\r\n");
        return;
    }

    if ((cmd_val < 0) || (cmd_val > 100))
    {
        VCP_Send("ERR: value out of range, use 0..100\r\n");
        return;
    }

    /* 10-bit code: 0x000 ~ 0x3FF, where 0x3FF = IREF (set by the IREFx resistor) */
    code = (uint16_t)((1023U * (uint16_t)cmd_val) / 100U);

    if (LM3435_Set_RGB_Current(code, code, code) != HAL_OK)
    {
        VCP_Printf("ERR: I2C write failed (HAL err=0x%04X)\r\n",
                   (unsigned)HAL_I2C_GetError(&hi2c1));
        return;
    }

    VCP_Printf("OK: %d%% -> RLED=%u GLED=%u BLED=%u\r\n",
               cmd_val, code, code, code);

    if (cmd_val == 0)
    {
        VCP_Send("    warning: code 0 leaves VOUT at VIN, which the LM3435 reports\r\n");
        VCP_Send("             as a SHORT fault. Use a low % instead of 0.\r\n");
    }

    if (LM3435_ReadReg(LM3435_REG_FAULT, &flt) == HAL_OK)
    {
        VCP_Send("    FAULT(07h)=");
        LM3435_PrintFault(flt);
    }
    else
    {
        VCP_Printf("    FAULT(07h) read failed (I2C err=0x%04X)\r\n",
                   (unsigned)HAL_I2C_GetError(&hi2c1));
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
