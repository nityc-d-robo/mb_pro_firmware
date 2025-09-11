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
  *ramped
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t id_own;
uint32_t id_angle;
SpeedController speed_controller[8] = { 0 };
AngleController angle_controller[4] = { 0 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float KP  = 0.01f;
float KI  = 0.0001f;
float KD  = 0.f;

float DT = 0.05f;

float ANGLE_KP  = 0.01f;
float ANGLE_KI  = 0.0001f;
float ANGLE_KD  = 0.f;
float ANGLE_DT  = 0.05f;

#define INTEGRAL_MAX (4000.0f)
#define INTEGRAL_MIN (-4000.0f)

#define CURRENT_MAX (1.f)
#define CURRENT_MIN (-1.f)

#define MAX_DIFF_RPM 30

#define NORMALIZED_MAX (16384)
#define NORMALIZED_MIN (-16384)

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  目標rpmと前周期の指示rpmを元にこの制御周期のramped_target_rpmを計算する
  * @param  target_rpm 最終的な目標のrpm値
  * @param  pre_ramped_rpm 前周期で算出した加速度でクリップされたrpm
  * @retval 加速度でクリップされたrpm
  */
float rampedRPM(float target_rpm, float pre_ramped_rpm) {
  float diff_rpm = target_rpm - pre_ramped_rpm;

  float ramped_rpm;

  if (diff_rpm > MAX_DIFF_RPM) {
    ramped_rpm = pre_ramped_rpm + MAX_DIFF_RPM;
  } else if (diff_rpm < -MAX_DIFF_RPM) {
    ramped_rpm = pre_ramped_rpm - MAX_DIFF_RPM;
  } else {
    ramped_rpm = target_rpm;
  }

  return ramped_rpm;
}

/**
  * @brief  target_rpmとnow_rpmを元に電流値を算出する
  * @param  target_rpm 目標のrpm int16_t
  * @param  now_rpm 現在のrpm int16_t
  * @retval 電流値，-20A ~ 20Aを-16384 ~ 16384の範囲に正規化
  */
int16_t rotateSpeed(SpeedController *cnt_) {
  cnt_->error = cnt_->target.ramped_rpm - cnt_->motors.rpm;

  cnt_->p = cnt_->error;
  float p_term = KP * cnt_->error;

  cnt_->i += cnt_->error * DT;
  if (cnt_->i > INTEGRAL_MAX) {
    cnt_->i = INTEGRAL_MAX;
  } else if (cnt_->i < INTEGRAL_MIN) {
    cnt_->i = INTEGRAL_MIN;
  }
  float i_term = KI * cnt_->i;
  cnt_->pre_error = cnt_->error;

  float output_current = p_term + i_term;
  if (output_current > CURRENT_MAX) {
    output_current = CURRENT_MAX;
  } else if (output_current < CURRENT_MIN) {
    output_current = CURRENT_MIN;
  }
  int16_t normalized_current = (int16_t)((output_current / 20.0) * NORMALIZED_MAX);

  return normalized_current;
}

int16_t rotateAngle(AngleController *cnt_) {
  cnt_->error = (float)(cnt_->target.fusion_cnt - cnt_->motors.fusion_cnt);

  cnt_->p = cnt_->error;
  float p_term = ANGLE_KP * cnt_->error;

  cnt_->i += cnt_->error * DT;
  if (cnt_->i > INTEGRAL_MAX) {
    cnt_->i = INTEGRAL_MAX;
  } else if (cnt_->i < INTEGRAL_MIN) {
    cnt_->i = INTEGRAL_MIN;
  }
  float i_term = ANGLE_KI * cnt_->i;
  cnt_->pre_error = cnt_->error;

  float output_current = p_term + i_term;
  if (output_current > CURRENT_MAX) {
    output_current = CURRENT_MAX;
  } else if (output_current < CURRENT_MIN) {
    output_current = CURRENT_MIN;
  }
  int16_t normalized_current = (int16_t)((output_current / 20.0) * 10000);

  return normalized_current;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);

    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_datas[8] = { 0 };

    tx_header.Identifier = 0x200;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;

    for (size_t i = 0; i < 4; i++) {
      if (speed_controller[i].target.mode != SPEED) { continue; }

      speed_controller[i].target.pre_ramped_rpm = speed_controller[i].target.ramped_rpm;

      speed_controller[i].target.ramped_rpm = rampedRPM(speed_controller[i].target.rpm, speed_controller[i].target.pre_ramped_rpm);

      int16_t current = rotateSpeed(&speed_controller[i]);

      tx_datas[i * 2] = ((current >> 8) & 0xFF);
      tx_datas[i * 2 + 1] = (current & 0xFF);
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_datas);

    tx_header.Identifier = 0x1FF;
    for (size_t i = 0; i < 4; i++) {
      if (speed_controller[i + 4].target.mode != SPEED) {
        tx_datas[i * 2] = 0;
        tx_datas[i * 2 + 1] = 0;
        continue; 
      }

      speed_controller[i + 4].target.pre_ramped_rpm = speed_controller[i + 4].target.ramped_rpm;

      speed_controller[i + 4].target.ramped_rpm = rampedRPM(speed_controller[i + 4].target.rpm, speed_controller[i + 4].target.pre_ramped_rpm);

      int16_t current = rotateSpeed(&speed_controller[i + 4]);

      tx_datas[i * 2] = ((current >> 8) & 0xFF);
      tx_datas[i * 2 + 1] = (current & 0xFF);
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_datas);

    /* 角度制御 */
    tx_header.Identifier = 0x200;
    angle_controller[3].target.mode = ANGLE;
    for (size_t i = 0; i < 4; i++) {
      if (angle_controller[i].target.mode != ANGLE) {
        tx_datas[i * 2] = 0;
        tx_datas[i * 2 + 1] = 0;
        continue; 
      }
      // int16_t current = 500;
      int16_t current = rotateAngle(&angle_controller[i]);

      tx_datas[i * 2] = ((current >> 8) & 0xFF);
      tx_datas[i * 2 + 1] = (current & 0xFF);
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_datas);
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
