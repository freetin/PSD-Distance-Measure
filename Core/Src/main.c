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
#include "adc.h"
#include "bdma.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "string.h"
#include "stdio.h"
#include "filters.h"
#include "i2c_hal.h"
#include "at24cxx.h"
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

/* USER CODE BEGIN PV */
uint8_t rx_buf[20];
char tx_buf[20];

uint8_t rx_data,rx_count;
uint8_t adc1flag,adc2flag,adc3flag;
uint32_t adc1,dac_value;
float adc_volt;
float volt_rate,volt_fi;
float Psd_Lx = 11.62;
float postion_x=0,filtered_volt_rate=0.0f,alpha=0.5f,postion_x1=0,postion_x2=0;
float adc1_ave,adc2_ave,adc3_ave;
	uint8_t buff[6]={1};
float kalman_volt,kalman_px;

uint8_t bb=3;
#define ADC_BUFFER_SIZE 6
ALIGN_32BYTES (uint16_t adc3_value[ADC_BUFFER_SIZE]) __attribute__((section(".ARM.__at_0x38000000")));
uint16_t adc1_value[ADC_BUFFER_SIZE]__attribute__((section(".ARM.__at_0x24000000")));
uint16_t adc2_value[ADC_BUFFER_SIZE]__attribute__((section(".ARM.__at_0x24010000")));
uint8_t aa=6;
uint8_t test_char = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  //MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//EventRecorderInitialize(EventRecordAll, 1U); 
	//EventRecorderStart();
	DWT_Init();
	I2CInit();
	
	HAL_UART_Receive_IT(&huart1,&rx_data,1);
	sprintf(tx_buf,"111");
	HAL_UART_Transmit(&huart1 , (uint8_t *)(tx_buf) , strlen(tx_buf) , 50);
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
	//unsigned char buf_w[] = {0x11, 0x22, 0x33};
	I2CWrite(0x120, &aa, 1); // ?????0x1A3F
	HAL_Delay(1000);
	// ????
	//unsigned char buf_r[10];
	I2CRead(0x120, &bb, 1); // ?0x2000??10??
//	
//	//HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buf,25);
//	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc1_value,ADC_BUFFER_SIZE);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)&adc2_value,ADC_BUFFER_SIZE);
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&adc3_value,ADC_BUFFER_SIZE);
	dac_value=1.5 * 4095 /2.5;
	//HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_value);
	//HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
	LowPassFilter low_filter;
	//MovingAverageFilter* filter = createMovingAverageFilter(3);
	initLowPassFilter(&low_filter,0.5,0.4);
	VoltRatioFilter* filter = create_volt_ratio_filter(12, 2, 0.1, 0.2);
	MovingAverageFilter *voltaverage = createMovingAverageFilter(20);
	//uint8_t res;
	//res = AT24CXX_Check();
//	if(!res)
//	{
//		aa=666;
//	}
//	else
//	{
//		aa=776;
//	}
//	HAL_Delay(1000);
//	AT24CXX_Write(0x16,(uint8_t *)bb,1);
//	HAL_Delay(1000);
//	AT24CXX_Read(0x16,buff,5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
         
	  if (adc1flag == 1)  
        {  
            uint32_t sum = 0;  
            for (int i = 0; i < ADC_BUFFER_SIZE; i++)  
            {  
                sum += (uint32_t)adc1_value[i];
            }  
            float average = (float)sum / ADC_BUFFER_SIZE;  
            adc1_ave = average * 3.0f / 65535.0f;  
            adc1flag = 0; 
        }
		if (adc2flag == 1)  
        {  
            uint32_t sum = 0;  
            for (int i = 0; i < ADC_BUFFER_SIZE; i++)  
            {  
                sum += (uint32_t)adc2_value[i];
            }  
            float average = (float)sum / ADC_BUFFER_SIZE;  
            adc2_ave = average * 3.0f / 65535.0f;  
            adc2flag = 0; // ?????  
        }
		
		if (adc3flag == 1)  
        {  
            uint32_t sum = 0;  
            for (int i = 0; i < ADC_BUFFER_SIZE; i++)  
            {  
                sum += (uint32_t)adc3_value[i];
            }  
            float average = (float)sum / ADC_BUFFER_SIZE;  
            adc3_ave = average * 3.0f / 65535.0f;  
            adc3flag = 0; // ?????  
        }
		if(adc1_ave > adc2_ave)
		{
			postion_x=(adc1_ave-adc2_ave)*Psd_Lx/(-(adc1_ave+adc2_ave)+2*adc3_ave)/2;
			volt_rate= (adc1_ave-adc3_ave)/(adc2_ave-adc3_ave);
			if(adc2_ave-adc3_ave==0) volt_rate=0.0f;
			postion_x=(adc1_ave-adc2_ave)*Psd_Lx/((adc1_ave-adc3_ave)+(adc2_ave-adc3_ave))/2;
		}
		else
		{
			volt_rate= (adc2_ave-adc3_ave)/(adc1_ave-adc3_ave);
			if(adc1_ave-adc3_ave==0) volt_rate=0.0f;
			postion_x=(adc2_ave-adc1_ave)*Psd_Lx/(-(adc1_ave+adc2_ave)+2*adc3_ave)/2;
			//postion_x=(adc2_ave-adc1_ave)*Psd_Lx/((adc2_ave-adc3_ave)+(adc1_ave-adc3_ave))/2;
		}
		postion_x1 = Psd_Lx * (1.0f - volt_rate)/(1+volt_rate)/2.0f; 
        volt_fi= updateMovingAverage(voltaverage,volt_rate);
		postion_x2=Psd_Lx * (1.0f - volt_fi)/(1+volt_fi)/2.0f;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rx_buf[rx_count++]=rx_data;
	HAL_UART_Receive_IT(&huart1,&rx_data,1);
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
