/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   RC Receiver PWM Input Capture — Channel 1
  *          Blue Pill STM32F103C8T6
  *          PA0  = TIM2_CH1 Input (← Receiver CH1 signal)
  *          PA9  = UART1 TX → PuTTY 115200
  *          PA10 = UART1 RX
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* -----------------------------------------------------------
 * Input Capture variables
 * ----------------------------------------------------------- */
static uint32_t ic_rise      = 0;   /* tick luc xung len (rising)  */
static uint32_t ic_fall      = 0;   /* tick luc xung xuong (fall)  */
static uint32_t pulse_width  = 0;   /* do rong xung tinh bang tick */
static uint8_t  capture_done = 0;   /* co: da do xong 1 chu ky     */
static uint8_t  edge_state   = 0;   /* 0=cho rising, 1=cho falling */

/* printf → UART1 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* ============================================================
 *  HAL Input Capture callback — goi tu ISR TIM2
 * ============================================================ */
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;
    if (htim->Channel  != HAL_TIM_ACTIVE_CHANNEL_1) return;

    if (edge_state == 0)
    {
        /* ---- Rising edge: bat dau xung ---- */
        ic_rise    = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        edge_state = 1;

        /* Chuyen sang bat falling edge */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        /* ---- Falling edge: ket thuc xung ---- */
        ic_fall = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        /* Xu ly tran so (timer overflow) */
        if (ic_fall >= ic_rise)
            pulse_width = ic_fall - ic_rise;
        else
            pulse_width = (65535 - ic_rise) + ic_fall + 1;

        capture_done = 1;
        edge_state   = 0;

        /* Chuyen lai bat rising edge cho chu ky tiep */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
    }
}
/* USER CODE END 0 */

/* ============================================================
 *  main
 * ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    /* Bat Input Capture interrupt TIM2 CH1 */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    printf("\r\n=== RC Receiver Input Capture ===\r\n");
    printf("Kenh 1 (CH1) noi vao PA0\r\n");
    printf("Bat tay dieu khien len...\r\n\r\n");
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        if (capture_done)
        {
            capture_done = 0;

            /* 1 tick = 10us (100kHz timer) → us = tick * 10 */
            uint32_t pulse_us = pulse_width * 10;

            /* Loc nhieu: chi in neu nam trong dai hop le 800–2200us */
            if (pulse_us >= 800 && pulse_us <= 2200)
            {
                /* Tinh phan tram: 1000us=0%, 2000us=100% */
                int32_t pct = ((int32_t)pulse_us - 1000) * 100 / 1000;
                if (pct < 0)   pct = 0;
                if (pct > 100) pct = 100;

                /* Ve thanh bar don gian */
                char bar[21];
                int  bar_len = pct / 5;   /* 0-20 ky tu */
                for (int i = 0; i < 20; i++)
                    bar[i] = (i < bar_len) ? '=' : '-';
                bar[20] = '\0';

                printf("CH1: %4lu us  |%s| %3ld%%\r\n",
                       pulse_us, bar, pct);
            }
            else
            {
                /* Tin hieu ngoai dai — co the chua ket noi */
                printf("CH1: %4lu us  [NGOAI DAI - kiem tra day noi]\r\n",
                       pulse_us);
            }
        }

        HAL_Delay(50);   /* In 20 lan/giay — du de theo doi */
        /* USER CODE END 3 */
    }
}

/* ============================================================
 *  System Clock — HSI 8MHz
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* ============================================================
 *  TIM2 — Input Capture, PA0 = CH1
 *  Prescaler=79 → 100kHz → 1 tick = 10us
 * ============================================================ */
static void MX_TIM2_Init(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 79;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK) Error_Handler();

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
}

/* ============================================================
 *  USART1 — PA9=TX PA10=RX 115200
 * ============================================================ */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* ============================================================
 *  GPIO — enable clock PA
 * ============================================================ */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
