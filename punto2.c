/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SEG_A  (1<<0)
#define SEG_B  (1<<1)
#define SEG_C  (1<<2)
#define SEG_D1 (1<<3)
#define SEG_D2 (1<<4)
#define SEG_E  (1<<5)
#define SEG_F  (1<<6)
#define SEG_G  (1<<7)
#define SEG_H  (1<<8)
#define SEG_I  (1<<9)
#define SEG_J  (1<<10)
#define SEG_K  (1<<11)
#define SEG_L  (1<<12)
#define SEG_M  (1<<13)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
#include <string.h>

uint16_t letras[27] =
{
    // A
    SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G,

    // B
    SEG_A|SEG_B|SEG_C|SEG_D1|SEG_D2|SEG_E|SEG_F|SEG_G,

    // C
    SEG_A|SEG_D1|SEG_D2|SEG_E|SEG_F,

    // D
    SEG_A|SEG_B|SEG_C|SEG_D1|SEG_D2|SEG_I|SEG_L,

    // E
    SEG_A|SEG_D1|SEG_D2|SEG_E|SEG_F|SEG_G,

    // F
    SEG_A|SEG_E|SEG_F|SEG_G,

    // G
    SEG_A|SEG_C|SEG_D1|SEG_D2|SEG_E|SEG_F|SEG_G,

    // H
    SEG_B|SEG_C|SEG_E|SEG_F|SEG_G,

    // I
	SEG_A | SEG_D1 | SEG_D2 | SEG_L|SEG_M,

    // J
    SEG_B|SEG_C|SEG_D1|SEG_D2,

    // K
    SEG_I|SEG_J|SEG_K|SEG_L,

    // L
    SEG_D1|SEG_D2|SEG_E|SEG_F,

    // M
    SEG_B|SEG_C|SEG_E|SEG_F|SEG_H|SEG_I,

    // N
    SEG_B|SEG_C|SEG_E|SEG_F|SEG_H|SEG_K,

    // O
    SEG_A|SEG_B|SEG_C|SEG_D1|SEG_D2|SEG_E|SEG_F,

    // P
    SEG_A|SEG_B|SEG_E|SEG_F|SEG_G,

    // Q
    SEG_A|SEG_B|SEG_C|SEG_D1|SEG_D2|SEG_E|SEG_F|SEG_K,

    // R
    SEG_A|SEG_B|SEG_E|SEG_F|SEG_G|SEG_K,

    // S
    SEG_A|SEG_C|SEG_D1|SEG_D2|SEG_F|SEG_G,

    // T
    SEG_A|SEG_L|SEG_M,

    // U
    SEG_B|SEG_C|SEG_D1|SEG_D2|SEG_E|SEG_F,

    // V
    SEG_H|SEG_I,

    // W
    SEG_B|SEG_C|SEG_E|SEG_F|SEG_J|SEG_K,

    // X
    SEG_H|SEG_I|SEG_J|SEG_K,

    // Y
    SEG_H|SEG_I|SEG_M,

    // Z
    SEG_A|SEG_D1|SEG_D2|SEG_I|SEG_J,

    // Espacio
    0
};

uint8_t nombre_1[] = {9,20,11,8,0,13,0,26,18,14,11,14,17,25,0};
uint8_t nombre_2[] = {0,11,4,9,0,13,3,17,14,26,0,2,14,18,19,0};
uint8_t nombre_3[] = {13,4,11,18,14,13,26,7,4,17,13,0,13,3,4,25};
uint8_t *nombres[] = {nombre_1,nombre_2,nombre_3};
uint8_t longitudes[] = {15,16,16};

uint8_t nombreActual = 0;
uint8_t scrollIndex = 0;
uint8_t displayActivo = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A_Pin|B_Pin|C_Pin|D1_Pin
                          |D2_Pin|E_Pin|F_Pin|G_Pin
                          |H_Pin|I_Pin|J_Pin|K_Pin
                          |L_Pin|nulo_Pin|nuloA14_Pin|M_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|P2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin D1_Pin
                           D2_Pin E_Pin F_Pin G_Pin
                           H_Pin I_Pin J_Pin K_Pin
                           L_Pin nulo_Pin nuloA14_Pin M_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D1_Pin
                          |D2_Pin|E_Pin|F_Pin|G_Pin
                          |H_Pin|I_Pin|J_Pin|K_Pin
                          |L_Pin|nulo_Pin|nuloA14_Pin|M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 P2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void multiplex(void)
{
    uint16_t patron;

    // 1. Apagar ambos displays
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    if(displayActivo == 0)
    {
        patron = letras[nombres[nombreActual][scrollIndex]];
        displayActivo = 1;

        escribirSegmentos(patron);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else
    {
        uint8_t siguiente = scrollIndex + 1;
        if(siguiente >= longitudes[nombreActual])
            siguiente = 0;

        patron = letras[nombres[nombreActual][siguiente]];
        displayActivo = 0;

        escribirSegmentos(patron);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        multiplex();
    }

    if(htim->Instance == TIM3)
    {
        scrollIndex++;

        if(scrollIndex >= longitudes[nombreActual])
            scrollIndex = 0;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last = 0;

    if(GPIO_Pin == GPIO_PIN_15)
    {
        uint32_t now = HAL_GetTick();
        if(now - last < 200) return;   // antirrebote
        last = now;

        nombreActual++;

        if(nombreActual >= 3)
            nombreActual = 0;

        scrollIndex = 0;
    }
}

void escribirSegmentos(uint16_t patron)
{
    GPIOA->ODR = patron;
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