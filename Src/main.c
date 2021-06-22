/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "checksum.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define HMI_BUF_SIZE    256

#define PC_DATA_RX_FLAG         0x0001
#define PC_DATA_TX_FLAG         0x0002

#define STORM_DATA_RX_FLAG         0x0001
#define STORM_DATA_TX_FLAG         0x0002
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t commLength;
  uint16_t commIndex;
  uint16_t IndexIn;
  uint16_t IndexOut;
  uint8_t  buf[HMI_BUF_SIZE];
  uint8_t flag;
}pcMsg_TypeDef_t;

typedef struct {
  uint8_t len;
  uint8_t buf[50];
}pc_data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

osThreadId mainTaskHandle;
osThreadId TxPCTaskHandle;
osThreadId RxPCTaskHandle;
osThreadId StormTxTaskHandle;
osThreadId StormRxTaskHandle;
/* USER CODE BEGIN PV */
static pcMsg_TypeDef_t pcMSG;
static pcMsg_TypeDef_t stormMSG;
EventGroupHandle_t xCreatedEventPC;
EventGroupHandle_t xCreatedEventStorm;

osMessageQId xQueuePC;
osMessageQId xQueueStorm;

uint8_t CMD_SETANGLE_UART[19] = {0xFA, 0x0E, 0x11, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00,
                            0x00,                       //The flags byte allows to modify the behavior of the angle setting for each axis
                            0x00,                       //The type byte is not used currently and has to be set to zero
                            0x00, 0x00};                //crc_low_byte, crc_high_byte
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM10_Init(void);
void StartMainTask(void const * argument);
void StartTxPCTask(void const * argument);
void StartRxPCTask(void const * argument);
void StartStormTxTask(void const * argument);
void StartStormRxTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ReceivePCData(void);
void TransmitPCEnd(void);
void Install_required_angle(float roll, float pitch, float yaw);
void TransmitStormEnd(void);
void ReceiveStormData(void);
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
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  xCreatedEventPC = xEventGroupCreate();
  xCreatedEventStorm = xEventGroupCreate();
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, StartMainTask, osPriorityNormal, 0, 128);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of TxPCTask */
  osThreadDef(TxPCTask, StartTxPCTask, osPriorityNormal, 0, 2048);
  TxPCTaskHandle = osThreadCreate(osThread(TxPCTask), NULL);

  /* definition and creation of RxPCTask */
  osThreadDef(RxPCTask, StartRxPCTask, osPriorityNormal, 0, 2048);
  RxPCTaskHandle = osThreadCreate(osThread(RxPCTask), NULL);

  /* definition and creation of StormTxTask */
  osThreadDef(StormTxTask, StartStormTxTask, osPriorityNormal, 0, 2048);
  StormTxTaskHandle = osThreadCreate(osThread(StormTxTask), NULL);

  /* definition and creation of StormRxTask */
  osThreadDef(StormRxTask, StartStormRxTask, osPriorityNormal, 0, 2048);
  StormRxTaskHandle = osThreadCreate(osThread(StormRxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQDef(queuePC, 30, pc_data_t); // Declare a message queue
  xQueuePC = osMessageCreate(osMessageQ(queuePC), NULL);
  osMessageQDef(queueStorm, 30, pc_data_t); // Declare a message queue
  xQueueStorm = osMessageCreate(osMessageQ(queueStorm), NULL);
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ReceivePCData(void)
{
  BaseType_t xHigherPriorityTaskWoken;
  
  if(pcMSG.buf[pcMSG.IndexOut] == '<')
  {
    pcMSG.flag |= 0x01;
    pcMSG.IndexIn = pcMSG.IndexOut;
  }
  else if((pcMSG.flag & 0x01) && pcMSG.buf[pcMSG.IndexOut] == '>')
  {
    pcMSG.flag = 0x00;
    pcMSG.commIndex = pcMSG.IndexIn;
    if(pcMSG.IndexIn <= pcMSG.IndexOut)
    {
      pcMSG.commLength = pcMSG.IndexOut - pcMSG.IndexIn;
    }
    else
    {
      pcMSG.commLength = (HMI_BUF_SIZE - pcMSG.IndexIn) + pcMSG.IndexOut;
    }   
    xEventGroupSetBitsFromISR(xCreatedEventPC, PC_DATA_RX_FLAG, &xHigherPriorityTaskWoken);
  }
  
  if((++pcMSG.IndexOut) >= HMI_BUF_SIZE)
  {
    pcMSG.IndexOut = 0;
  }
  HAL_UART_Receive_IT(&huart5, &pcMSG.buf[pcMSG.IndexOut], 1);
}

/*Storm*/
void ReceiveStormData(void)
{
  BaseType_t xHigherPriorityTaskWoken;
  
  if(stormMSG.buf[stormMSG.IndexOut] == 0x06)
  {
    stormMSG.flag |= 0x01;
    stormMSG.IndexIn = stormMSG.IndexOut;
  }
  else if((stormMSG.flag & 0x01) && stormMSG.buf[stormMSG.IndexOut] == 'o')
  {
    stormMSG.flag = 0x00;
    stormMSG.commIndex = stormMSG.IndexIn;
    if(stormMSG.IndexIn <= stormMSG.IndexOut)
    {
      stormMSG.commLength = stormMSG.IndexOut - stormMSG.IndexIn;
    }
    else
    {
      stormMSG.commLength = (HMI_BUF_SIZE - stormMSG.IndexIn) + stormMSG.IndexOut;
    }   
    xEventGroupSetBitsFromISR(xCreatedEventStorm, STORM_DATA_RX_FLAG, &xHigherPriorityTaskWoken);
    stormMSG.IndexOut = 0;
    stormMSG.IndexIn = 0;
    stormMSG.flag = 0;
    HAL_UART_Receive_IT(&huart4, &stormMSG.buf[0], 1);
  }
  

  if(stormMSG.flag & 0x01)
  {
    if((++stormMSG.IndexOut) >= HMI_BUF_SIZE)
    {
      stormMSG.IndexOut = 0;
      stormMSG.IndexIn = 0;
      stormMSG.flag = 0;
    }
    HAL_UART_Receive_IT(&huart4, &stormMSG.buf[stormMSG.IndexOut], 1);
  }
  else
  {
    HAL_UART_Receive_IT(&huart4, &stormMSG.buf[0], 1);
  }     
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == UART5)
  {
    ReceivePCData();
  }
  else if(huart->Instance == UART4)
  {
    ReceiveStormData();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == UART5)
  {
    TransmitPCEnd();
  }
  else if(huart->Instance == UART4)
  {
    TransmitStormEnd();
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{

  pc_data_t message;
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    message.len = 1;
    message.buf[0] = 'd';
    xQueueSend(xQueueStorm, &message, portMAX_DELAY);
    osDelay(10);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTxPCTask */
/**
* @brief Function implementing the TxPCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTxPCTask */
void StartTxPCTask(void const * argument)
{
  /* USER CODE BEGIN StartTxPCTask */
  pc_data_t message;
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(xQueuePC,  &message, portMAX_DELAY);
    HAL_UART_Transmit_IT(&huart5, message.buf, message.len);
    xEventGroupWaitBits(xCreatedEventPC, PC_DATA_TX_FLAG, pdTRUE, pdFALSE, portMAX_DELAY);
  }
  /* USER CODE END StartTxPCTask */
}

/* USER CODE BEGIN Header_StartRxPCTask */
/**
* @brief Function implementing the RxPCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRxPCTask */
void StartRxPCTask(void const * argument)
{
  uint8_t i;
  EventBits_t event;
  uint8_t local_buf[40];
  float roll, pitch, yaw;
  
  HAL_UART_Receive_IT(&huart5, &pcMSG.buf[0], 1);
  /* USER CODE BEGIN StartRxPCTask */
  /* Infinite loop */
  for(;;)
  {
    event = xEventGroupWaitBits(xCreatedEventPC, PC_DATA_RX_FLAG, pdTRUE, pdFALSE, portMAX_DELAY);
    if(event & PC_DATA_RX_FLAG)
    {
      for(i = 0; i < pcMSG.commLength; i++)
      {
        local_buf[i] = pcMSG.buf[pcMSG.IndexIn];
        if((++pcMSG.IndexIn) >= HMI_BUF_SIZE)
        {
          pcMSG.IndexIn = 0;
        }
      }
      local_buf[i] = '\0';
      
      if(strstr((char*)local_buf, "<SetAngle:") != NULL)
      {
        sscanf((char*)&local_buf, "<SetAngle:%f:%f:%f>", &roll, &pitch, &yaw);
        Install_required_angle(roll, pitch, yaw);
      }
      
    }
  }
  /* USER CODE END StartRxPCTask */
}

void TransmitPCEnd(void)
{
  BaseType_t xHigherPriorityTaskWoken;
  xEventGroupSetBitsFromISR(xCreatedEventPC, PC_DATA_TX_FLAG, &xHigherPriorityTaskWoken);
}

void TransmitStormEnd(void)
{
  BaseType_t xHigherPriorityTaskWoken;
  xEventGroupSetBitsFromISR(xCreatedEventStorm, STORM_DATA_TX_FLAG, &xHigherPriorityTaskWoken);
}


union {
  float Angle_from_Simulink;
  uint8_t Angle_from_PC[4];
} Roll_from_PC, Pitch_from_PC;

void Install_required_angle(float roll, float pitch, float yaw)
{
  pc_data_t message;
  uint16_t CRC_Full_Result;
  
  Roll_from_PC.Angle_from_Simulink = roll;
  Pitch_from_PC.Angle_from_Simulink = pitch;
  //---------------------Переписываем значения углов установки платформы в массивы для расчета CRC и для отправки по UART
  //-------------- Значение крена-------------------------------------------
  uint8_t local_index = 3;
  for (uint8_t k=0; k<4; k++){
    CMD_SETANGLE_UART[local_index] = Roll_from_PC.Angle_from_PC[k];
    local_index++;
  }
  
  //------------- Значение тангажа------------------------------------------
  local_index = 7;
  for (uint8_t k=0; k<4; k++){
    CMD_SETANGLE_UART[local_index] = Pitch_from_PC.Angle_from_PC[k];
    local_index++;
  }
  //------------------------------------------------------------------------
  
  CRC_Full_Result = crc_calculate(&CMD_SETANGLE_UART[1], 16);       //вычисление контрольной суммы тестового массива требуемых углов
  
  CMD_SETANGLE_UART[17] = CRC_Full_Result & 0xFF;               //младшая часть контрольной суммы
  
  CMD_SETANGLE_UART[18] = (CRC_Full_Result & 0xFF00) >> 8;      //старшая часть контрольной суммы
  
  message.len = sizeof(CMD_SETANGLE_UART);
  memcpy(message.buf, CMD_SETANGLE_UART, message.len);
  xQueueSend(xQueueStorm, &message, portMAX_DELAY);  
}

/* USER CODE BEGIN Header_StartStormTxTask */
/**
* @brief Function implementing the StormTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStormTxTask */
void StartStormTxTask(void const * argument)
{
  /* USER CODE BEGIN StartStormTxTask */
  pc_data_t message;
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(xQueueStorm,  &message, portMAX_DELAY);
    HAL_UART_Transmit_IT(&huart4, message.buf, message.len);
    xEventGroupWaitBits(xCreatedEventStorm, STORM_DATA_TX_FLAG, pdTRUE, pdFALSE, portMAX_DELAY);
  }
  /* USER CODE END StartStormTxTask */
}

/* USER CODE BEGIN Header_StartStormRxTask */
/**
* @brief Function implementing the StormRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStormRxTask */
void StartStormRxTask(void const * argument)
{
  /* USER CODE BEGIN StartStormRxTask */
  uint8_t i;
  pc_data_t message;
  EventBits_t event;
  uint8_t local_buf[40];
  int16_t Roll, Pitch, Yaw;
  
  HAL_UART_Receive_IT(&huart4, &stormMSG.buf[0], 1);
  /* Infinite loop */
  for(;;)
  {
    event = xEventGroupWaitBits(xCreatedEventStorm, STORM_DATA_RX_FLAG, pdTRUE, pdFALSE, portMAX_DELAY);
    if(event & STORM_DATA_RX_FLAG)
    {
      for(i = 0; i < stormMSG.commLength; i++)
      {
        local_buf[i] = stormMSG.buf[stormMSG.IndexIn];
        if((++stormMSG.IndexIn) >= HMI_BUF_SIZE)
        {
          stormMSG.IndexIn = 0;
        }
      }
      local_buf[i] = '\0';
      
      if(local_buf[0] == 0x06)
      {
        Roll = (((int16_t)local_buf[33] << 8) | (local_buf[32]));    
        Pitch = (((int16_t)local_buf[35] << 8) | (local_buf[34]));      
        Yaw = (((int16_t)local_buf[37] << 8) | (local_buf[36]));
        
        message.len = sprintf (message.buf, "<%d:%d>\n", Roll, Pitch);
        xQueueSend(xQueuePC, &message, portMAX_DELAY);
      }
      
    }
  }
  /* USER CODE END StartStormRxTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
