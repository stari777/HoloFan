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
#include "tim.h"
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
/*
	      rad_i = atof(p);
	      p = strchr(p, ',');
	      if (!p) goto parse_error;
	      p++;

	      phi_i = atof(p);
	      p = strchr(p, ',');
	      if (!p) goto parse_error;
	      p++;

	      r_tmp = atoi(p);
	      p = strchr(p, ',');
	      if (!p) goto parse_error;
	      p++;

	      g_tmp = atoi(p);
	      p = strchr(p, ',');
	      if (!p) goto parse_error;
	      p++;

	      b_tmp = atoi(p);

	      int n = sscanf((char*)rx_buf, "%f,%f,%u,%u,%u",
	    		  &rad_i, &phi_i, &r_tmp, &g_tmp, &b_tmp);
 */

/* --------------------------------------------------------------------------
 * UART-Empfang & Datenpuffer
 * -------------------------------------------------------------------------- */
uint8_t rx_byte;
uint8_t rx_buf[256];
volatile uint8_t new_line_received = 0;
uint16_t idx = 0;

#define MAX_VALUES 900
#define FRAME_SIZE 30

float rad_arr[MAX_VALUES];
float phi_arr[MAX_VALUES];
uint8_t rgb_arr[MAX_VALUES][3] = {0};

volatile int count = 0;

/* --------------------------------------------------------------------------
 * POV-Konfiguration
 * -------------------------------------------------------------------------- */
#define POV_SLICES          128        // Winkelaufloesung: 360° / 128 = 2,8125°
#define ROTATION_PERIOD_US  20000UL    // 3000 RPM → 20 ms pro Umdrehung

uint8_t pov_frame[POV_SLICES][FRAME_SIZE][3];
static  uint8_t pov_built = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        if(rx_byte == '\n' || rx_byte == '\r')
        {
            rx_buf[idx] = 0;          // String beenden
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
            new_line_received = 1;    // Hauptschleife informieren
            idx = 0;                  // Reset für neue Zeile
        }
        else
        {	// Ueberlauf verhindern
            if(idx < sizeof(rx_buf)-1)
                rx_buf[idx++] = rx_byte; // Wert uebergeben dann idx+1
        }
        __HAL_UART_CLEAR_OREFLAG(huart);          // Overrun Flag loeschen


        // Naechsten Interrupt starten
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

static uint32_t parse_uint(char **p)
{
	uint32_t val = 0; // aktuell gelesener Integer
	while(**p >= '0' && **p <= '9') // Ziffer
	{
		val = val * 10 + (**p - '0'); // ASCII in Zahlen
		(*p)++; // Zeiger auf naechstes Zeichen
	}
	if(**p == ',') (*p)++;  // Komma ueberspringen
	return val; // gibt geparste Zahl zurueck
}

volatile uint32_t ms_counter = 0;
volatile uint32_t us_counter = 0;

// Timer Interrupt Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) ms_counter++;
}

uint32_t millis() { return ms_counter; }

uint32_t micros() { return __HAL_TIM_GET_COUNTER(&htim1); }

static void build_pov_buffer(void)
{
    /* Puffer mit Schwarz initialisieren */
    for (int s = 0; s < POV_SLICES; s++)
        for (int l = 0; l < FRAME_SIZE; l++)
            pov_frame[s][l][0] = pov_frame[s][l][1] = pov_frame[s][l][2] = 0;

    /* Maximalen Radius für Normierung bestimmen */
    float max_rad = 0.0f;
    for (int j = 0; j < count; j++)
        if (rad_arr[j] > max_rad) max_rad = rad_arr[j];
    if (max_rad == 0.0f) max_rad = 1.0f;

    /* Jeden Punkt in pov_frame eintragen */
    for (int j = 0; j < count; j++)
    {
        /* Winkel → Slice-Index */
        float norm_phi = phi_arr[j];
        if (norm_phi < 0.0f)    norm_phi += 360.0f;
        if (norm_phi >= 360.0f) norm_phi -= 360.0f;

        int slice = (int)(norm_phi / 360.0f * (float)POV_SLICES);
        if (slice < 0)           slice = 0;
        if (slice >= POV_SLICES) slice = POV_SLICES - 1;

        /* Radius → LED-Index (0 = Achse, 29 = außen) */
        int led = (int)(rad_arr[j] / max_rad * (float)(FRAME_SIZE - 1) + 0.5f);
        if (led < 0)           led = 0;
        if (led >= FRAME_SIZE) led = FRAME_SIZE - 1;

        /* Hellerer Punkt gewinnt bei Überschneidung */
        if ((rgb_arr[j][0] + rgb_arr[j][1] + rgb_arr[j][2])
          > (pov_frame[slice][led][0] + pov_frame[slice][led][1] + pov_frame[slice][led][2]))
        {
            pov_frame[slice][led][0] = rgb_arr[j][0];
            pov_frame[slice][led][1] = rgb_arr[j][1];
            pov_frame[slice][led][2] = rgb_arr[j][2];
        }
    }
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim1);
  DigiLed_init(&hspi1); // Initialisierung der Library
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // Interrupt start

  uint8_t ready_msg[] = "READY\r\n";
  HAL_Delay(10); // Kurz warten
  HAL_UART_Transmit(&huart2, ready_msg, sizeof(ready_msg), 1000); // READY an Python Code senden


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if(new_line_received)
		{
			new_line_received = 0;
			//HAL_UART_Transmit(&huart2, &rx_byte, 1, HAL_MAX_DELAY); // Echo zum Test

			char* p = (char*)rx_buf;
			//float rad_i, phi_i;

			// Integer statt float
			uint32_t rad_i = parse_uint(&p);
			uint32_t phi_i = parse_uint(&p);
			uint32_t r_tmp  = parse_uint(&p);
			uint32_t g_tmp  = parse_uint(&p);
			uint32_t b_tmp  = parse_uint(&p);

			if(count < MAX_VALUES)
			{
				rgb_arr[count][0] = (uint8_t)r_tmp;
				rgb_arr[count][1] = (uint8_t)g_tmp;
				rgb_arr[count][2] = (uint8_t)b_tmp;
				rad_arr[count] = rad_i / 100.0f;
				phi_arr[count] = phi_i / 100.0f;
				count++;
			}

		}
		/*
		 --- LED-Ansteuerung ohne Motor ---

		if (count >= MAX_VALUES)
		{
				int TOTAL_FRAMES = MAX_VALUES / FRAME_SIZE;
				static int frame = 0;
				static uint32_t last_time = 0;
				int offset;

				if (micros() - last_time >= 10000)  // alle 100ms nächster Frame
				{
					last_time = micros();
					offset = frame * FRAME_SIZE;

					for (int i = 0; i < FRAME_SIZE; i++)
					{
						DigiLed_setColor(i,
								rgb_arr[offset + i][0],
								rgb_arr[offset + i][1],
								rgb_arr[offset + i][2]);
					}
					DigiLed_setAllIllumination(1);
					DigiLed_update(1);

					frame++;
					if (frame >= TOTAL_FRAMES)
						frame = 0;  // von vorne
				}
			}
		}*/
        if (count >= MAX_VALUES)
        {
            build_pov_buffer();
            pov_built = 1;
        }

        if (pov_built)
        {
            uint32_t pos_us = micros() % ROTATION_PERIOD_US;

            int slice = (int)((float)pos_us
                              / (float)ROTATION_PERIOD_US
                              * (float)POV_SLICES);

            if (slice < 0)           slice = 0;
            if (slice >= POV_SLICES) slice = POV_SLICES - 1;

            for (int i = 0; i < FRAME_SIZE; i++)
            {
                DigiLed_setColor(i,
                                 pov_frame[slice][i][0],
                                 pov_frame[slice][i][1],
                                 pov_frame[slice][i][2]);
            }

            DigiLed_setAllIllumination(1);
            DigiLed_update(1);
        }

		/*continue;

	  parse_error:
	      // Ungültige Zeile -> ignorieren
	      count = count;*/
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
