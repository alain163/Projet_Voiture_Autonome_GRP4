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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <CoVAPSy_Lidar.h>
#include "CoVAPSy_moteurs.h"


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

uint8_t Data_RX_LIDAR;
uint16_t Data_Lidar_mm[360];
uint8_t drapeau_fin_tour = 0;
uint16_t data_lidar_mm_main[360];


/////////////
float angle_degre, vitesse_m_s;



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
	uint8_t drapeau_fin_tour_old = 0;			// Variable pour le LiDAR
	uint32_t i = 0;

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Lidar_init();
  HAL_UART_Receive_IT(&huart1, &Data_RX_LIDAR, 1);
  Propulsion_init();
  Direction_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*set_direction_degres(0);
	HAL_Delay(1500);
	set_direction_degres(-10);
	HAL_Delay(1500);
	set_direction_degres(10);
	HAL_Delay(500);
	set_direction_degres(0);
	HAL_Delay(500);*/


	set_vitesse_m_s(1.0);
	HAL_Delay(1000);
	set_vitesse_m_s(2.0);
	HAL_Delay(1000);
	set_vitesse_m_s(3.0);
	HAL_Delay(1000);
	set_vitesse_m_s(0.0);
	HAL_Delay(1000);
	recule();
	HAL_Delay(1000);
	set_vitesse_m_s(0.0);
	HAL_Delay(1000);



  while (1)
  {

	  ///////////////////////////////////////////////////////////////////////////////////////////
	 //Recopie du tableau lidar en fin de tour, avec passage de sens horaire à sens trigo///////
	///////////////////////////////////////////////////////////////////////////////////////////
	if ((drapeau_fin_tour == 1) && (drapeau_fin_tour_old == 0)) {
		for (i = 0; i < 360; i++) {
			data_lidar_mm_main[i] = Data_Lidar_mm[i];
		}
	}
	drapeau_fin_tour_old = drapeau_fin_tour;

	//////////////////////////////////////////////////////////////////////////////
	/////// Gestion de la direction en fonction des données LIDAR
	//////////////////////////////////////////////////////////////////////////////
	angle_degre = 0.02 * (data_lidar_mm_main[60] - data_lidar_mm_main[300]); //distance à 60° - distance à -60°
	set_direction_degres(angle_degre);

	vitesse_m_s = 0.5;
	set_vitesse_m_s(vitesse_m_s);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint32_t index = 0;
	static uint8_t drapeau_demarrage = 0;  // Variable pour indiquer la réception des 7 premiers octets
	static uint8_t tableau_trame[7];      // Variable de stockage des 7 premiers octets
	uint16_t angle;
	uint16_t distance;

	//////////////////////////////////////////////////////////////////////////////
	/////// Réception des sept octets correspondant au descriptor     ////////////
	//////////////////////////////////////////////////////////////////////////////
	if (drapeau_demarrage == 0) {
		tableau_trame[index] = Data_RX_LIDAR;
		index++;
	}
	if ((index == 7) && (drapeau_demarrage == 0)) {
		index = 0;
		drapeau_demarrage = 1;				// Fin des sept octets du descriptor
	}

	//////////////////////////////////////////////////////////////////////////////
	/////// Réception des données par groupe de 5 octets          	   /////////
	/////// Qualité (1 octet), Angle (2 octets) et Distance (2 octets)   /////////
	//////////////////////////////////////////////////////////////////////////////
	if (drapeau_demarrage == 1) {
		if (index <= 4) {
			if ((Data_RX_LIDAR == 0x3e) || (Data_RX_LIDAR == 0x3d)) //trame de démarrage
				index = 0;
			tableau_trame[index] = Data_RX_LIDAR;
			if ((tableau_trame[0] == 0x3e) || (tableau_trame[0] == 0x3d))
				index++;
		}
		///////////////////////////////////////////////////////////
		/////// Traitement de donnée pour convertir      	/////////
		/////// les octets reçus en données réelles       /////////
		//////////////////////////////////////////////////////////
		if (index >= 5) {
			index = 0;
			angle = ((uint16_t) tableau_trame[2] << 1)
					+ ((uint16_t) tableau_trame[1] >> 7);
			distance = ((uint16_t) tableau_trame[4] << 6)
					+ ((uint16_t) tableau_trame[3] >> 2);

			////// Stockage de la distance mesurée dans le tableau à l'indice associé à l'angle
			if (angle == 0)
				Data_Lidar_mm[0] = distance;
			if ((angle < 360) && (angle > 0))
				Data_Lidar_mm[360 - angle] = distance;

			///////////////////////////////////////////////////////////
			///////  Détection de la fin d'un tour    	    /////////
			//////////////////////////////////////////////////////////
			if ((angle > 100) && (angle < 180) && (drapeau_fin_tour == 0)) {
				drapeau_fin_tour = 1;
			}
			if ((angle > 270) && (angle < 360) && (drapeau_fin_tour == 1)) {
				drapeau_fin_tour = 0;
			}

		}

	}

	///// Ré-activation de l'interruption UART RX
	HAL_UART_Receive_IT(&huart1, &Data_RX_LIDAR, 1);
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
