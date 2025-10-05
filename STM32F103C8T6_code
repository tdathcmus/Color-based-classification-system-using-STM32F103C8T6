/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int mau_hong = 0;
int mau_cam = 0;
int mau_tim = 0;
char buffer[32];
volatile uint8_t state = 0;
volatile uint8_t flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TCS34725_ADDRESS          (0x29 << 1) /* I2C address */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)
#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
#define TCS34725_INTEGRATIONTIME_154MS   0xC0
#define TCS34725_GAIN_4X                0x01

uint8_t _tcs34725Initialised = 0;
int red, green, blue;


void write8 (uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 2, 100);
}

uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

uint16_t read16(uint8_t reg)
{
  uint16_t ret;
    uint8_t txBuffer[1],rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
  return ret;
}

void enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  HAL_Delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  HAL_Delay(50);
}

void disable(void)
{
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
  uint8_t readValue = read8(TCS34725_ID);
  if ((readValue != 0x44) && (readValue != 0x10) && (readValue != 0x4d))
  {
    return;
  }
  _tcs34725Initialised = 1;

  setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS);
  setGain(TCS34725_GAIN_4X);
  enable();
}


void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (_tcs34725Initialised == 0) tcs3272_init();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  HAL_Delay(154);
}

void getRGB(int *R, int *G, int *B)
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);
    if(rawClear == 0)
    {
      *R = 0;
      *G = 0;
      *B = 0;
    }
    else
    {
      *R = (int)rawRed * 255 / rawClear;
      *G = (int)rawGreen * 255 / rawClear;
      *B = (int)rawBlue * 255 / rawClear;
    }
}

void sendbyte1(){
	sprintf(buffer, "%03d%03d%03d\n", mau_hong, mau_cam, mau_tim);
	HAL_UART_Transmit(&huart1, (int*)buffer, strlen(buffer), 100);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12){
		state = !state;
	}
	if(GPIO_Pin == GPIO_PIN_14){
		flag = 1;
	}
}
void lcdd(){
	LCD_clear();
	LCD_setCursor(0, 0);
	LCD_printf("TIM:0");
	LCD_setCursor(0, 8);
	LCD_printf("CAM:0");
	LCD_setCursor(1, 0);
	LCD_printf("HONG:0");
}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  LCD_init();
  lcdd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(state ==0 && flag == 1){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 1220);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
		mau_tim = 0;
		mau_cam = 0;
		mau_hong = 0;
		lcdd();
		flag = 0;
	  }
	  if(state == 1){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
		  flag = 0;
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 1220);
		  HAL_Delay(1000);
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 800);
		  getRGB(&red, &green, &blue);
		  if(red>=160&& red<=180 && green>=34 && green<=54 && blue>=45 && blue<= 65)
		  {
			  mau_hong++;
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 1250);
			  LCD_setCursor(1, 0);
			  LCD_printf("HONG:%d",mau_hong);
			  HAL_Delay(1000);
		  }
		  else if(red>=152 && red<=172 && green>=54 && green<=74 && blue>=23 && blue<= 43)
		  {
			  mau_cam++;
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 760);
			  LCD_setCursor(0, 8);
			  LCD_printf("CAM:%d",mau_cam);
			  HAL_Delay(1000);
		  }
		  else if (red>=110 && red<=130 && green>=56 && green<=76 && blue>=66 && blue<= 86)
		  {
			  mau_tim++;
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 250);
			  LCD_setCursor(0, 0);
			  LCD_printf("TIM:%d",mau_tim);
			  HAL_Delay(1000);
		  }
		  else {
			  HAL_GPIO_WritePin(GPIOA, GPIO_Pin_15, 1);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOA, GPIO_Pin_15, 0);
		  }

		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 380);
		  HAL_Delay(500);
		  sendbyte1();
	  }else{
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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


 
•	Code của ESP32
#include <WiFi.h>
#include <HTTPClient.h>
#include "HardwareSerial.h"

const char* ssid = "123456";  
const char* password = "88888887";
const char* serverURL = "https://script.google.com/macros/s/AKfycbzcpawd5pJsAPLRN4VF64XEDThW5jDw41KGQHPs-KCJZah31dwMFkIkz7nu9Kv0NEac/exec";

HardwareSerial mySerial(1);
char buffer[32];

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.println("Đang kết nối WiFi...");
        attempts++;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Không thể kết nối WiFi!");
        return;
    }
    Serial.println("WiFi đã kết nối!");
    mySerial.begin(115200, SERIAL_8N1, 16, 17); 
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        if (mySerial.available() >= 9) {
            int len = mySerial.readBytes(buffer, 32);  
            buffer[len] = '\0';
            Serial.printf("Chuỗi nhận được: '%s'\n", buffer);
            char mau_hong[4] = {0}, mau_cam[4] = {0}, mau_tim[4] = {0};
            memcpy(mau_hong, buffer, 3);
            memcpy(mau_cam, buffer + 3, 3);
            memcpy(mau_tim, buffer + 6, 3);
            if (strlen(mau_hong) == 3 && strlen(mau_cam) == 3 && strlen(mau_tim) == 3) {
                Serial.printf("Nhận dữ liệu - mauhong:%s, maucam:%s, mautim:%s\n", 
                            mau_hong, mau_cam, mau_tim);
                HTTPClient http;
                String url = String(serverURL) + "?mau_hong=" + String(mau_hong) + 
                           "&mau_cam=" + String(mau_cam) + "&mau_tim=" + String(mau_tim);
                http.begin(url);
                int httpCode = http.GET();
                if (httpCode > 0) {
                    Serial.println("Gửi thành công!");
                } else {
                    Serial.printf("Lỗi gửi HTTP: %s\n", http.errorToString(httpCode).c_str());
                }
                http.end();
                delay(100);
            } else {
                Serial.println("Dữ liệu không hợp lệ!");
            }
        }
    } else {
        Serial.println("WiFi chưa kết nối!");
        WiFi.reconnect();
        delay(5000);
    }
    delay(500);
}
