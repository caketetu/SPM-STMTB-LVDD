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
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SPM_Modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define servo_max 15
#define SERVO_DE_SETUP_DELAY_US 100U
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//デバイス定義
int16_t model_no = 51;
int16_t version = 1;
int16_t sub_version = 0;

//シリアルデータ
uint8_t recv_data;
uint8_t rx_buf[256];
uint8_t tx_buf[256];
volatile int16_t rx_len=0;

uint8_t rx3_buf[10];
uint8_t rx4_buf[10];
uint8_t tx3_buf[10];
uint8_t tx4_buf[10];

uint8_t rx_buf_len=0;
uint8_t r_mode = 0;		//�ǂݎ�胂�[�h�A0:pos,1:cur,2:temp
volatile uint16_t pos[servo_max];	//�T�[�{�|�W�V�������M�p
volatile uint16_t repos[servo_max];	//�T�[�{�|�W�V������M�p
uint16_t target_pos[servo_max];	//�T�[�{�|�W�V����
uint16_t pos_max[servo_max];	//�T�[�{�|�W�V����
uint16_t pos_min[servo_max];	//�T�[�{�|�W�V����
volatile uint8_t	cur[servo_max];	//�T�[�{�d��
volatile uint8_t	temp[servo_max];	//�T�[�{���x
volatile uint32_t servo_tx_busy_count = 0;
volatile uint32_t servo_tx_error_count = 0;
volatile uint32_t servo_rx_error_count = 0;
volatile uint32_t servo_rx_restart_error_count = 0;

//アプリケーション変数
int16_t dev_id=51; //デバイスID
//LEDのRGBデータ 1チャンネルで最大32個までのLEDを制御可能
//一つのLEDは8bit×24bitのデータが必要だが、レジスタは16bitなので、2つのレジスタに分けて格納する
// ledData[0] = LED1_R, LED1_G
// ledData[1] = LED1_B, LED2_R
// ledData[2] = LED2_G, LED2_B
// ...
int16_t enable_input = 0; //イネーブル信号入力

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  //MX_DMAMUX_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //各レジスタ初期化
  SPM_ModbusInit();
  //レジスタ登録
  p_input_regs[0] = &model_no;   //モデルナンバー
  p_input_regs[1] = &version; //バージョン
  p_input_regs[2] = &sub_version; //サブバージョン

  //Cyclic Function登録
  // cycfunc0.rx_len = 2;
  // cycfunc0.rx_adr[0] = &io_in;
  // cycfunc0.rx_adr[1] = &io_out;
  // cycfunc0.tx_len = 1;
  // cycfunc0.tx_adr[0] = &io_out;

  ParamLoad();
  SPM_ModbusSetAddress(dev_id);

  HAL_UART_Receive_IT(&huart2, &recv_data, 1);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // LED ON
  if (HAL_TIM_Base_Start_IT(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  pos[0] = 7500;

  //FDCAN1 filter configuration
  // FDCAN_FilterTypeDef sFilterConfig;
  // sFilterConfig.IdType = FDCAN_STANDARD_ID;
  // sFilterConfig.FilterIndex = 0;
  // sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // sFilterConfig.FilterID1 = 0x123; // Standard ID is configured as-is (no manual bit shift)
  // sFilterConfig.FilterID2 = 0x7FF; // Filter mask (accept all IDs)
  // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Filter configuration Error
  //   Error_Handler();
  // }else{
    
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  // // Start FDCAN module
  // if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Start Error
  //   Error_Handler();
  // }else{
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // uint32_t last_servo_update = HAL_GetTick();
  // uint32_t last_position_update = HAL_GetTick();
  // uint32_t last_led_toggle = HAL_GetTick();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  while (1)
  {
    // //FDCAN送信
    // FDCAN_TxHeaderTypeDef TxHeader = {0};
    // uint8_t CanTxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, counter++}; // 送信データ
    // TxHeader.Identifier = 0x143; // 送信するID
    // TxHeader.IdType = FDCAN_STANDARD_ID;
    // TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    // TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    // TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    // TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // TxHeader.MessageMarker = 0;
    // if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CanTxData) != HAL_OK)
    // {
    //   // Transmission request Error
    //   TxDataLength = sprintf((char*)TxData, "FDCAN Transmission Error\r\n");
    //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
    //   Error_Handler();
    // }else{
    //   TxDataLength = sprintf((char*)TxData, "FDCAN Transmission Success\r\n");
    //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
    // }

    //シリアル通信処理
		if(rx_len>0){
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			HAL_Delay(10);
			int16_t tx_len = SPM_ModbusTask(rx_buf, rx_len, tx_buf);
			HAL_UART_Transmit(&huart2, tx_buf, tx_len, 100);
			rx_len=0;
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		}

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((4799 * 70) / 100));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_Delay(2000);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((4799 * 70) / 100));
    HAL_Delay(2000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
volatile int timer_1s = 0; // タイマーカウンタ
volatile int timer_10ms = 0; // タイマーカウンタ
volatile int led_state[4] = {0,0,0,0}; // タイマーカウンタ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM14)
    {
        timer_1s++;
        if(timer_1s>=1000)
        {
            timer_1s=0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle LED to indicate 1 second elapsed
        }
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART2)
   {
     if (rx_len < sizeof(rx_buf))
     {
       rx_buf[rx_len] = recv_data;
       rx_len++;
     }
     HAL_UART_Receive_IT(&huart2, &recv_data, 1);
   }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
	// if( UartHandle->Instance == USART3 ){
	// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	// 	servo_restart_receive(&huart3, rx3_buf, sizeof(rx3_buf));
	// }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	// if (UartHandle->Instance == USART3)
	// {
	// 	servo_rx_error_count++;
	// 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	// 	servo_restart_receive(&huart3, rx3_buf, sizeof(rx3_buf));
	// }
	if (UartHandle->Instance == USART4)
	{
		servo_rx_error_count++;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
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
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // Toggle LED to indicate error
    HAL_Delay(100); // 0.1秒待機
  }
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
