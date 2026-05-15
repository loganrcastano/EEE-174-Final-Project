/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Power Monitoring and Protection System
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Thresholds */
#define VOLTAGE_MAX             9.0f
#define CURRENT_MAX             1.5f
#define POWER_MAX               6.0f

#define VOLTAGE_WARN            6.0f
#define CURRENT_WARN            0.8f
#define POWER_WARN              5.0f

/* ADC scaling */
#define ADC_MAX_COUNTS          4095.0f
#define ADC_REF_VOLTAGE         3.3f

/* Voltage divider: R1 from load voltage to PA0, R2 from PA0 to GND */
#define R1                      100000.0f
#define R2                      15000.0f
#define VOLTAGE_DIV_GAIN        ((R1 + R2) / R2)

/* ACS712 5A sensor */
#define CURRENT_SENSOR_ZERO     2.32f
#define CURRENT_SENSOR_SENS     0.185f

#define NUM_SAMPLES             16
#define FAULT_COUNT_LIMIT       3

/* Pin definitions based on your .ioc */
#define RELAY_GPIO_PORT         GPIOA
#define RELAY_PIN               GPIO_PIN_7

#define LED_GREEN_PORT          GPIOA
#define LED_GREEN_PIN           GPIO_PIN_3

#define LED_YELLOW_PORT         GPIOA
#define LED_YELLOW_PIN          GPIO_PIN_4

#define LED_RED_PORT            GPIOA
#define LED_RED_PIN             GPIO_PIN_5

#define RESET_BTN_PORT          GPIOA
#define RESET_BTN_PIN           GPIO_PIN_8

#define VOLTAGE_ADC_CHANNEL     ADC_CHANNEL_1
#define CURRENT_ADC_CHANNEL     ADC_CHANNEL_2

/*
   Relay logic:
   If your relay module is active HIGH:
      RELAY_ON_STATE = GPIO_PIN_SET
      RELAY_OFF_STATE = GPIO_PIN_RESET

   If your relay module is active LOW:
      RELAY_ON_STATE = GPIO_PIN_RESET
      RELAY_OFF_STATE = GPIO_PIN_SET
*/
#define RELAY_ON_STATE          GPIO_PIN_SET
#define RELAY_OFF_STATE         GPIO_PIN_RESET

typedef enum
{
    Setup_State,
    Voltage_Capture_State,
    Current_Capture_State,
    Power_Calc_State,
    Threshold_Check_State,
    Normal_State,
    Warning_State,
    Trip_State,
    Log_State,
    Reset_State,
    Sleep_State
} eSystemState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void SetupHandler(float* fVoltage, float* fCurrent, float* fPower,
                  uint8_t* warningFlag, uint8_t* tripFlag, uint8_t* faultCount);

void VoltageCaptureHandler(float* fVoltage);
void CurrentCaptureHandler(float* fCurrent);
void PowerCalcHandler(float* fVoltage, float* fCurrent, float* fPower);
void ThresholdCheckHandler(float* fVoltage, float* fCurrent, float* fPower,
                           uint8_t* warningFlag, uint8_t* tripFlag, uint8_t* faultCount);

void NormalHandler(void);
void WarningHandler(void);
void TripHandler(void);
void DataLogHandler(float* fVoltage, float* fCurrent, float* fPower);
void ResetHandler(uint8_t* warningFlag, uint8_t* tripFlag, uint8_t* faultCount);
void SleepHandler(void);

uint32_t ReadADC_Average(uint32_t channel);
float ConvertADCToVoltage(uint32_t rawValue);
float ConvertADCToCurrent(uint32_t rawValue);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void SetupHandler(float* fVoltage, float* fCurrent, float* fPower,
                  uint8_t* warningFlag, uint8_t* tripFlag, uint8_t* faultCount)
{
    *fVoltage = 0.0f;
    *fCurrent = 0.0f;
    *fPower = 0.0f;
    *warningFlag = 0;
    *tripFlag = 0;
    *faultCount = 0;

    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_PIN, RELAY_OFF_STATE);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
}

uint32_t ReadADC_Average(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t sum = 0;

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    return sum / NUM_SAMPLES;
}

float ConvertADCToVoltage(uint32_t rawValue)
{
    float vadc = ((float)rawValue / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
    return vadc * VOLTAGE_DIV_GAIN;
}

float ConvertADCToCurrent(uint32_t rawValue)
{
    float vsense = ((float)rawValue / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
    float current = (vsense - CURRENT_SENSOR_ZERO) / CURRENT_SENSOR_SENS;

    current = fabsf(current);

    if (current < 0.03f)
    {
        current = 0.0f;
    }

    return current;
}

void VoltageCaptureHandler(float* fVoltage)
{
    uint32_t rawValue = ReadADC_Average(VOLTAGE_ADC_CHANNEL);
    *fVoltage = ConvertADCToVoltage(rawValue);
}

void CurrentCaptureHandler(float* fCurrent)
{
    uint32_t rawValue = ReadADC_Average(CURRENT_ADC_CHANNEL);
    *fCurrent = ConvertADCToCurrent(rawValue);
}

void PowerCalcHandler(float* fVoltage, float* fCurrent, float* fPower)
{
    *fPower = (*fVoltage) * (*fCurrent);
}

void ThresholdCheckHandler(float* fVoltage, float* fCurrent, float* fPower,
                           uint8_t* warningFlag, uint8_t* tripFlag, uint8_t* faultCount)
{
    *warningFlag = 0;
    *tripFlag = 0;

    if ((*fVoltage > VOLTAGE_WARN) ||
        (*fCurrent > CURRENT_WARN) ||
        (*fPower > POWER_WARN))
    {
        *warningFlag = 1;
    }

    if ((*fVoltage > VOLTAGE_MAX) ||
        (*fCurrent > CURRENT_MAX) ||
        (*fPower > POWER_MAX))
    {
        (*faultCount)++;
    }
    else
    {
        *faultCount = 0;
    }

    if (*faultCount >= FAULT_COUNT_LIMIT)
    {
        *tripFlag = 1;
    }
}

void NormalHandler(void)
{
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_PIN, RELAY_ON_STATE);

    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
}

void WarningHandler(void)
{
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_PIN, RELAY_ON_STATE);

    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
}

void TripHandler(void)
{
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_PIN, RELAY_OFF_STATE);

    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
}

void DataLogHandler(float* fVoltage, float* fCurrent, float* fPower)
{
    char msg[128];

    sprintf(msg, "Voltage: %.2f V, Current: %.2f A, Power: %.2f W\r\n",
            *fVoltage, *fCurrent, *fPower);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void ResetHandler(uint8_t* warningFlag, uint8_t* tripFlag, uint8_t* faultCount)
{
    /*
       Assumes PA8 uses pull-up:
       button not pressed = HIGH
       button pressed = LOW
    */
    if (HAL_GPIO_ReadPin(RESET_BTN_PORT, RESET_BTN_PIN) == GPIO_PIN_RESET)
    {
        HAL_Delay(50);

        if (HAL_GPIO_ReadPin(RESET_BTN_PORT, RESET_BTN_PIN) == GPIO_PIN_RESET)
        {
            *warningFlag = 0;
            *tripFlag = 0;
            *faultCount = 0;
        }
    }
}

void SleepHandler(void)
{
    HAL_Delay(100);
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  eSystemState eNextState = Setup_State;

  float fVoltage = 0.0f;
  float fCurrent = 0.0f;
  float fPower = 0.0f;

  uint8_t warningFlag = 0;
  uint8_t tripFlag = 0;
  uint8_t faultCount = 0;
  /* USER CODE END 1 */

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    switch (eNextState)
    {
        case Setup_State:
            SetupHandler(&fVoltage, &fCurrent, &fPower,
                         &warningFlag, &tripFlag, &faultCount);
            eNextState = Voltage_Capture_State;
            break;

        case Voltage_Capture_State:
            VoltageCaptureHandler(&fVoltage);
            eNextState = Current_Capture_State;
            break;

        case Current_Capture_State:
            CurrentCaptureHandler(&fCurrent);
            eNextState = Power_Calc_State;
            break;

        case Power_Calc_State:
            PowerCalcHandler(&fVoltage, &fCurrent, &fPower);
            eNextState = Threshold_Check_State;
            break;

        case Threshold_Check_State:
            ThresholdCheckHandler(&fVoltage, &fCurrent, &fPower,
                                  &warningFlag, &tripFlag, &faultCount);

            if (tripFlag)
            {
                eNextState = Trip_State;
            }
            else if (warningFlag)
            {
                eNextState = Warning_State;
            }
            else
            {
                eNextState = Normal_State;
            }
            break;

        case Normal_State:
            NormalHandler();
            eNextState = Log_State;
            break;

        case Warning_State:
            WarningHandler();
            eNextState = Log_State;
            break;

        case Trip_State:
            TripHandler();
            eNextState = Log_State;
            break;

        case Log_State:
            DataLogHandler(&fVoltage, &fCurrent, &fPower);

            if (tripFlag)
            {
                eNextState = Reset_State;
            }
            else
            {
                eNextState = Sleep_State;
            }
            break;

        case Reset_State:
            ResetHandler(&warningFlag, &tripFlag, &faultCount);

            if (tripFlag == 0)
            {
                eNextState = Sleep_State;
            }
            else
            {
                eNextState = Reset_State;
            }
            break;

        case Sleep_State:
            SleepHandler();
            eNextState = Voltage_Capture_State;
            break;

        default:
            eNextState = Setup_State;
            break;
    }

    /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
