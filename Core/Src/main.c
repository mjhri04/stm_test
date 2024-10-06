/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68

#define IMU_SENSOR1_ADDR 0x68  // 예: MPU6050
#define IMU_SENSOR2_ADDR 0x69
#define IMU_SENSOR3_ADDR 0x6A
#define NUM_SENSORS 4
uint32_t pressure_values[NUM_SENSORS] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
unsigned char IMUarray[32] = {0};
int cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
char alpha[8] = {0};
char beta[8] = {0};
char gamm[8] = {0};
double roll_tp, pitch_tp, yaw_tp;
double roll, pitch, yaw;
char *pos;
int16_t accel[3] = {0};
int16_t gyro[3] = {0};
int32_t pressure = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
//void MPU6050_Init(uint8_t sensor_addr);
void Read_IMU_Date(uint8_t sensor_addr, int16_t *accel_data, int16_t *gyro_data);
void Read_Pressure_Sensors(uint32_t *values, uint8_t num_sensors);
void parse_IMU_data(void);

//void get_pitch(void);
//void Read_IMU_RPY(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // 센서 초기화 단계
  /*MPU6050_Init(); //초기화
  BMP280_Init(); //초기화
  */
  MPU6050_Init(IMU_SENSOR1_ADDR);
  MPU6050_Init(IMU_SENSOR2_ADDR);
  MPU6050_Init(IMU_SENSOR3_ADDR);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // IMU 데이터 읽기
	  Read_IMU_Data(IMU_SENSOR1_ADDR, accel, gyro);
	  parse_IMU_data();

	  // 압력센서 데이터 읽기
	  Read_Pressure_Sensors(pressure_values, NUM_SENSORS);

	  HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  for (int i = 0; i < NUM_SENSORS; i++) {
      sConfig.Channel = ADC_CHANNEL_0 + i; // FSR 센서가 연결된 채널
      sConfig.Rank = i + 1; // 각 채널의 순위 설정
      sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // 샘플링 시간 설정
      HAL_ADC_ConfigChannel(&hadc1, &sConfig); // 채널 설정
  }
  /* USER CODE END ADC1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*void MPU6050_Init(uint8_t sensor_addr){
	uint8_t check, Data;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, 0x75, 1, &check, 1, 1000);
	if(check == 104){
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, 0x6B, 1, &Data, 1, 1000);
	}
}

*/
void Read_IMU_Data(uint8_t sensor_addr, int16_t *accel_data, int16_t *gyro_data) {
    uint8_t data[14];
    // 가속도와 자이로 데이터 읽기 (MPU6050에서 가속도 레지스터 주소는 0x3B부터 시작)
    HAL_I2C_Mem_Read(&hi2c1, sensor_addr << 1, 0x3B, 1, data, 14, 1000);

    // 가속도 데이터 처리
    accel_data[0] = (int16_t)(data[0] << 8 | data[1]);  // X축
    accel_data[1] = (int16_t)(data[2] << 8 | data[3]);  // Y축
    accel_data[2] = (int16_t)(data[4] << 8 | data[5]);  // Z축

    // 자이로 데이터 처리
    gyro_data[0] = (int16_t)(data[8] << 8 | data[9]);   // X축
    gyro_data[1] = (int16_t)(data[10] << 8 | data[11]); // Y축
    gyro_data[2] = (int16_t)(data[12] << 8 | data[13]); // Z축

    // IMU 데이터 가공하여 IMUarray에 저장 (예제)
    snprintf((char*)IMUarray, sizeof(IMUarray), "%d,%d,%d\r", accel[0], accel[1], accel[2]);
}

void Read_Pressure_Sensors(uint32_t *values, uint8_t num_sensors) {
	HAL_ADC_Start(&hadc1);

	    // 모든 채널의 변환 요청
	    for (uint8_t i = 0; i < num_sensors; i++) {
	        HAL_ADC_ConfigChannel(&hadc1, &(ADC_ChannelConfTypeDef){
	            .Channel = ADC_CHANNEL_0 + i, // FSR 센서가 연결된 채널
	            .Rank = 1,
	            .SamplingTime = ADC_SAMPLETIME_15CYCLES
	        });
	    }

	    // ADC 변환 완료 대기
	    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	    // 압력 값 읽기
	    for (uint8_t i = 0; i < num_sensors; i++) {
	        values[i] = HAL_ADC_GetValue(&hadc1);
	        HAL_Delay(10);  // 센서 간 대기 시간 (필요에 따라 조정)
	    }

	    // ADC 종료
	    HAL_ADC_Stop(&hadc1);
}


void parse_IMU_data(void) {
    int cnt0 = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0;

    // IMUarray 배열에서 ','를 기준으로 문자열 파싱
    for(int i = 0; i < 31; i++) {
        if(IMUarray[i] == ',') {
            if(cnt0 == 0){
                cnt1 = i + 1;
            } else if (cnt0 == 1){
                cnt2 = i + 1;
            }
            cnt0++;
        } else {
            switch (cnt0) {
                case 0: alpha[i] = IMUarray[i]; break;
                case 1: beta[i - cnt1] = IMUarray[i]; break;
                case 2: gamm[i - cnt2] = IMUarray[i]; break;
            }
        }
        if(IMUarray[i] == '\r'){
            cnt3 = i + 1;
            break;
        }
    }

    // 문자열이 부족한 경우 나머지 0으로 채우기
    for (int j = cnt1 - 1; j < 8; j++) alpha[j] = '0';
    for (int k = cnt2 - cnt1; k < 8; k++) beta[k] = '0';
    for (int l = cnt3 - cnt2; l < 8; l++) gamm[l] = '0';

    // 문자열을 실수로 변환
    roll = strtod(alpha, &pos);
    pitch = strtod(beta, &pos);
    yaw = strtod(gamm, &pos);
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
