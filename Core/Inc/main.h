/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define CAN_TX_SIZE 8
#define CAN_RX_SIZE 8
#define CAN_RINGBUF_SIZE 256

/* FUNC CODE (4bit) */
#define FUNC_CODE_MASK  (0x0FU << 7)

#define NMT_CODE        (0x00U << 7)
#define SYNC_CODE       (0x01U << 7)
#define EMCY_CODE       (0x02U << 7)
#define TIME_CODE       (0x05U << 7)
#define TX_PDO1_CODE    (0x06U << 7)
#define RX_PDO1_CODE    (0x07U << 7)
#define TX_PDO2_CODE    (0x08U << 7)
#define RX_PDO2_CODE    (0x09U << 7)
#define TX_SDO_CODE     (0x0AU << 7)
#define RX_SDO_CODE     (0x0BU << 7)
#define NMT_ERR_CODE    (0x0CU << 7)

/* TYPE ID (2bit) */
#define MD_ID           (0x01U << 5)
#define OTHER_ID        (0x02U << 5)

/* TYPE ID (4bit)*/
#define MB_ID           (0x01U << 3)


/* SPECIAL NODE ID (7bit)*/
#define ALL_NODE_ID_MASK 0x7FU

#define BROADCAST_ID    0x00U
#define USB_CAN_ID      0x01U

/* RoboMaster ID*/
// tx rx逆
#define ROBO_MASTER_TX1 0x200U
#define ROBO_MASTER_TX2 0x1FFU

#define ROBO_MASTER_RX_MASK   0x20FU
#define ROBO_MASTER_RX_FILTER 0x200U
#define ROBO_MASTER_RX1 0x201U
#define ROBO_MASTER_RX2 0x202U
#define ROBO_MASTER_RX3 0x203U
#define ROBO_MASTER_RX4 0x204U
#define ROBO_MASTER_RX5 0x205U
#define ROBO_MASTER_RX6 0x206U
#define ROBO_MASTER_RX7 0x207U
#define ROBO_MASTER_RX8 0x208U

/* MASK */
#define ALL_BIT_MASK    0x7FFU
#define NODE_ID_MASK    0x1FU
#define MB_ID_MASK      0x07U
#define TYPE_ID_MASK    (0x03U << 5)

typedef struct {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[CAN_RX_SIZE];
} RxCanFrame;

typedef struct {
  RxCanFrame buf[CAN_RINGBUF_SIZE];
  uint16_t write_idx;
  uint16_t read_idx;
} CanRingBuf;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define GEAR_RATIO 19.2
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
typedef struct {
  int16_t angle;
  int16_t overflow;
  int32_t fusion_cnt;
  float rpm;
  int16_t current;
  int8_t temp;
} MotorState;

typedef enum {
  None, SPEED, ANGLE, CURRENT
} Mode;

typedef struct {
  Mode mode;
  float rpm;
  float ramped_rpm;
  float pre_ramped_rpm;
} TargetState;

typedef struct {
  Mode mode;
  int32_t fusion_cnt;
  float ramped_angle;
  float pre_ramped_angle;
} AngleTargetState;

typedef struct {
  MotorState motors;
  TargetState target;
  float p;
  float i;
  float d;
  float error;
  float pre_error;
} SpeedController;

typedef struct {
  MotorState motors;
  AngleTargetState target;
  float p;
  float i;
  float d;
  float error;
  float pre_error;
} AngleController;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
