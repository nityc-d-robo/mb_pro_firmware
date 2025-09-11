/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
extern uint32_t id_own;
extern uint32_t id_angle;

extern SpeedController speed_controller[8];
extern AngleController angle_controller[4];
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
  FDCAN_FilterTypeDef filter;
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 8;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // 重要度の高いブロードキャストをFIFO0へ設定
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = NMT_CODE | BROADCAST_ID;
  filter.FilterID2 = SYNC_CODE | BROADCAST_ID;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 1;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = TIME_CODE | BROADCAST_ID;
  filter.FilterID2 = TIME_CODE | BROADCAST_ID;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  /* このマザーボード宛のデータを受信 */
  id_own = (MB_ID | 0x0);
  id_angle = (MB_ID | 0x1);

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 2;
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  filter.FilterID1 = EMCY_CODE | id_own;
  filter.FilterID2 = TX_PDO1_CODE | id_own;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  filter.FilterIndex = 3;
  filter.FilterID1 = RX_PDO1_CODE | id_own;
  filter.FilterID2 = TX_PDO2_CODE | id_own;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  filter.FilterIndex = 4;
  filter.FilterID1 = RX_PDO2_CODE | id_own;
  filter.FilterID2 = TX_SDO_CODE | id_own;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  filter.FilterIndex = 5;
  filter.FilterID1 = RX_SDO_CODE | id_own;
  filter.FilterID2 = NMT_ERR_CODE | id_own;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  filter.FilterIndex = 6;
  filter.FilterID1 = RX_PDO1_CODE | id_angle;
  filter.FilterID2 = TX_PDO1_CODE | id_angle;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 7;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  filter.FilterID1 = ROBO_MASTER_RX1;
  filter.FilterID2 = ROBO_MASTER_RX8;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);

  HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan1, FDCAN1_RxFifo0Callback);
  HAL_FDCAN_RegisterRxFifo1Callback(&hfdcan1, FDCAN1_RxFifo1Callback);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0x0U);

  HAL_FDCAN_Start(&hfdcan1);
  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */
  FDCAN_FilterTypeDef filter;
  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 2;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 7;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 5;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  // RoboMasterに従い，0x201 ~ 0x208を設定
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = ROBO_MASTER_RX1;
  filter.FilterID2 = ROBO_MASTER_RX8;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);

  HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan2, FDCAN2_RxFifo0Callback);

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0x0U);

  HAL_FDCAN_Start(&hfdcan2);
  /* USER CODE END FDCAN2_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void FDCAN1_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1_, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[CAN_RX_SIZE] = { 0u };

  if  (HAL_FDCAN_GetRxMessage(hfdcan1_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) { return; }
}

void FDCAN1_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan1_, uint32_t RxFifo1ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[CAN_RX_SIZE] = { 0u };

  if  (HAL_FDCAN_GetRxMessage(hfdcan1_, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK) { return; }
  if (rx_header.Identifier == (RX_PDO1_CODE | id_own)) {
    for (size_t i = 0; i < 4; i++) {
      speed_controller[i].target.mode = SPEED;
      speed_controller[i].target.rpm = (float)((int16_t)(rx_data[i * 2] << 8 | rx_data[i * 2 + 1]) * GEAR_RATIO);
    }
  } else if (rx_header.Identifier == (RX_PDO2_CODE | id_own)) {
    for (size_t i = 0; i < 4; i++) {
      speed_controller[i + 4].target.mode = SPEED;
      speed_controller[i + 4].target.rpm = (float)((int16_t)(rx_data[i * 2] << 8 | rx_data[i * 2 + 1]) * GEAR_RATIO);
    }
  }
  if (rx_header.Identifier == (RX_PDO1_CODE | id_angle)) {
    for (size_t i = 0; i < 3; i++) {
      angle_controller[i].target.mode = ANGLE;
      angle_controller[i].target.fusion_cnt += (int64_t)((int16_t)(rx_data[i * 2] << 8 | rx_data[i * 2 + 1]) / 360) * 8192;
    }
  }
  if (rx_header.Identifier == (RX_PDO2_CODE | id_angle)) {
    for (size_t i = 0; i < 3; i++) {
      // angle_controller[i].target.mode = ANGLE;
      // angle_controller[i].target.angle += ((int16_t)(rx_data[i * 2] << 8 | rx_data[i * 2 + 1]) / 360) * 8192;
    }
  }

  switch (rx_header.Identifier) {
    case ROBO_MASTER_RX1:
    case ROBO_MASTER_RX2:
    case ROBO_MASTER_RX3:
    case ROBO_MASTER_RX4:
      int32_t motor_id = (rx_header.Identifier & 0x0F) - 1;
      int16_t new_angle = (int16_t)(rx_data[0] << 8 | rx_data[1]);
      if ((angle_controller[motor_id].motors.angle - new_angle) > 6000) {
        angle_controller[motor_id].motors.overflow++;
      } else if ((angle_controller[motor_id].motors.angle - new_angle) < -6000) {
        angle_controller[motor_id].motors.overflow--;
      }
      angle_controller[motor_id].motors.angle = new_angle;
      if (angle_controller[motor_id].motors.fusion_cnt == 0) {
        angle_controller[motor_id].target.fusion_cnt = (int64_t)(new_angle + (int64_t)(angle_controller[motor_id].motors.overflow * 8191));
      }
      
      angle_controller[motor_id].motors.fusion_cnt = (int64_t)(new_angle + (int64_t)(angle_controller[motor_id].motors.overflow * 8191));
      angle_controller[motor_id].motors.rpm = (float)((int16_t)(rx_data[2] << 8 | rx_data[3])); 
      angle_controller[motor_id].motors.current = (rx_data[4] << 8 | rx_data[5]);
      break;
    
    default:
      break;
  }
}

void FDCAN2_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan2_, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[CAN_RX_SIZE] = { 0u };

  if (HAL_FDCAN_GetRxMessage(hfdcan2_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) { return; }
  int32_t motor_id = (rx_header.Identifier & 0x0F) - 1;
  speed_controller[motor_id].motors.angle = (rx_data[0] << 8 | rx_data[1]);
  speed_controller[motor_id].motors.rpm = (float)((int16_t)(rx_data[2] << 8 | rx_data[3])); 
  speed_controller[motor_id].motors.current = (rx_data[4] << 8 | rx_data[5]);
  speed_controller[motor_id].motors.temp = (rx_data[6]);
}
/* USER CODE END 1 */
