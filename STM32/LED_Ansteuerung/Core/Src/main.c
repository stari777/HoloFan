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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  DigiLed_init(&hspi1); // Initialisierung der Library

  uint8_t rx_byte;
  uint8_t rx_buf[100];
  uint8_t idx = 0;
  uint8_t ready_msg[] = "READY\r\n";
  #define MAX_VALUES 30

  float rad_arr[MAX_VALUES];
  float phi_arr[MAX_VALUES];
  uint8_t rgb_arr[MAX_VALUES][3];

  int count = 0;
  char msg[100]; // für Ausgabe über UART

  // Kurz warten
  HAL_Delay(10);
  // READY an Python Code senden
  HAL_UART_Transmit(&huart2, ready_msg, sizeof(ready_msg), 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_UART_Receive(&huart2, &rx_byte, 1, 10) == HAL_OK)
	  	  {
	  		  //HAL_UART_Transmit(&huart2, &rx_byte, 1, HAL_MAX_DELAY); // Echo zum Test
	  		  if (rx_byte == '\n' || rx_byte == '\r')
	  		  {
	  			  if(idx > 0)
	  			  {
	  				  rx_buf[idx] = 0; // Nullterminator, Stringende
	  				  idx = 0;	// auf 0 setzen fuer naechste Nachricht

	  				  float rad, phi;
	  				  uint8_t r, g, b;
	  				  unsigned int r_tmp, g_tmp, b_tmp;
	  				  int n = sscanf((char*)rx_buf, "%f,%f,%u,%u,%u", &rad, &phi, &r_tmp, &g_tmp, &b_tmp);

	  				  if(n == 5)
	  				  {	  // Casten auf uint8_t
	  					  r = (uint8_t) r_tmp;
	  					  g = (uint8_t) g_tmp;
	  					  b = (uint8_t) b_tmp;

	  					  //int n = sscanf((char*)rx_buf, "%f,%f,%hhu,%hhu,%hhu", &rad, &phi, &r, &g, &b);
	  					  int len = sprintf(msg, "n=%d, rad=%.2f, phi=%.2f, RGB(%d,%d,%d)\r\n", n, rad, phi, r, g, b);
	  					  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 1000);

	  					  if (count < MAX_VALUES)
	  					  {
	  						  rad_arr[count] = rad;
	  						  phi_arr[count] = phi;
	  						  rgb_arr[count][0] = r;
	  						  rgb_arr[count][1] = g;
	  						  rgb_arr[count][2] = b;
	  						  count++;
	  					  }

	  					 //if (count == MAX_VALUES)
	  					  //{
	  						 for (int i = 0; i < MAX_VALUES; i++)
	  						 {
	  							DigiLed_setColor(i, rgb_arr[i][0], rgb_arr[i][1], rgb_arr[i][2]);

	  						 }
	  						 DigiLed_setAllIllumination(1);
	  						 DigiLed_update(1);

	  						 count = 0; // zuruecksetzen für den naechsten Frame
	  					  //}
	  				  }
	  			  }
	  		  }
	  		  else
	  		  {	  // Ueberlauf verhindern
	  			  if (idx < sizeof(rx_buf) - 1)	// Prueft noch Platz im Buffer
	  				  rx_buf[idx++] = rx_byte;	// Byte uebergeben an Buffer, dann idx+1
	  		  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
