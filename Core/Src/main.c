/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#define MESSAGE_LEN 256
#define RX_DMA_SIZE 256

// covariance define so i don't mess up the index
#define IND_ANGLE_X 0
#define IND_ANGLE_Y 1
#define IND_ANGLE_Z 2
#define IND_BIAS_X 3
#define IND_BIAS_Y 4
#define IND_BIAS_Z 5

// PID define so i don't mess up the index
#define KP_R [0][0]
#define KI_R [0][1]
#define KD_R [0][2]
#define KP_P [1][0]
#define KI_P [1][1]
#define KD_P [1][2]
#define KP_Y [2][0]
#define KI_Y [2][1]
#define KD_Y [2][2]

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float q[4];      // store quaternion
  float bias[3];   // gyro bias (booty cheeks)
  float P[6][6];   // uncertainty 1-3 is rot error
                   // rest is for gyro bias(i,j,k)
  float PID[3][3]; // just for neatness
} EKF_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// priv var// RX - Circular DMA
static uint8_t rx_dma_buf[RX_DMA_SIZE];
volatile uint16_t rx_last_pos = 0;
static uint8_t rx_buffer[MESSAGE_LEN]; // parsed packet storage
static volatile uint8_t data_ready = 0;
static char uart2_tx_buffer[MESSAGE_LEN];
static char uart1_tx_buffer[MESSAGE_LEN];
static volatile uint8_t uart2_tx_busy = 0;
static volatile uint8_t uart1_tx_busy = 0;
static EKF_State_t
    ekf; // Global just stores any values that must stay outside main scope
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void EKF_Predict(EKF_State_t *state, float gi, float gj, float gk, float dt);
void EKF_Update(EKF_State_t *state, float ai, float aj, float ak);
void EKF_Init(EKF_State_t *state);
float compute_elevator(EKF_State_t *state);
float compute_aileron(EKF_State_t *state);
void quaternion_to_body_x(float q[4], float x_body[3]);
void quaternion_to_body_y(float q[4], float y_body[3]);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void uart_output(uint8_t port, char *msg);
void process_rx_data(uint16_t old_pos, uint16_t new_pos);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // listen for 1 byte, save to rx_byte, and interrupt when done
  HAL_UART_Receive_DMA(&huart2, rx_dma_buf, RX_DMA_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  EKF_Init(&ekf);
  float sim_time, ax, ay, az, gx, gy, gz;
  float last_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (data_ready) {
      data_ready = 0;

      char *ptr = (char *)rx_buffer;
      char *end;

      // Parse all 7 fields
      sim_time = strtof(ptr, &end);
      if (*end != ',' ||
          end == ptr) { // Add check: end == ptr means no conversion
        EKF_Init(&ekf); // Reset on bad packet
        continue;
      }
      ptr = end + 1;

      ax = strtof(ptr, &end);
      if (*end != ',' || end == ptr) {
        EKF_Init(&ekf);
        continue;
      }
      ptr = end + 1;

      ay = strtof(ptr, &end);
      if (*end != ',' || end == ptr) {
        EKF_Init(&ekf);
        continue;
      }
      ptr = end + 1;

      az = strtof(ptr, &end);
      if (*end != ',' || end == ptr) {
        EKF_Init(&ekf);
        continue;
      }
      ptr = end + 1;

      gx = strtof(ptr, &end);
      if (*end != ',' || end == ptr) {
        EKF_Init(&ekf);
        continue;
      }
      ptr = end + 1;

      gy = strtof(ptr, &end);
      if (*end != ',' || end == ptr) {
        EKF_Init(&ekf);
        continue;
      }
      ptr = end + 1;

      gz = strtof(ptr, &end);
      if (end == ptr) { // Last field - no comma check needed
        EKF_Init(&ekf);
        continue;
      }

      // blink to show valid packet
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

      // compute time difference
      float dt = sim_time - last_time;
      if (dt <= 0.0f || dt > 0.05f)
        dt = 0.025f;
      last_time = sim_time;

      // EKF
      EKF_Predict(&ekf, gx, gy, gz, dt);
      EKF_Update(&ekf, ax, ay, az);

      if (isnan(ekf.q[0])) {
        EKF_Init(&ekf);
        continue;
      }

      float aileron_cmd = compute_aileron(&ekf);
      float elevator_cmd = compute_elevator(&ekf);

      char cmd_msg[MESSAGE_LEN];
      snprintf(cmd_msg, sizeof(cmd_msg), "%.4f,%.4f\n", aileron_cmd,
               elevator_cmd);
      uart_output(2, cmd_msg);

      char debug_msg[MESSAGE_LEN];
      snprintf(debug_msg, sizeof(debug_msg),
               "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
               ekf.q[0], ekf.q[1], ekf.q[2], ekf.q[3], ax, ay, az, gx, gy, gz,
               sim_time, dt);
      uart_output(1, debug_msg);
    }
  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// okay to see if we can trust gyro gng i need to predict where im gonna be
// first
void EKF_Predict(EKF_State_t *state, float gi, float gj, float gk, float dt) {
  // diangonal filter for now
  state->P[0][0] += 0.01f * dt;
  state->P[1][1] += 0.01f * dt;
  state->P[2][2] += 0.01f * dt;
  state->P[3][3] += 1e-6f * dt;
  state->P[4][4] += 1e-6f * dt;
  state->P[5][5] += 1e-6f * dt;

  // subtract bias :)
  float ui = gi - state->bias[0];
  float uj = gj - state->bias[1];
  float uk = gk - state->bias[2];

  // copy quaternion values into a variable so its easy to use
  float qw = state->q[0];
  float qi = state->q[1];
  float qj = state->q[2];
  float qk = state->q[3];
  // kinematic equation
  //  q(dot) = 1/2*q cross-mult w
  state->q[0] += 0.5f * dt * (-qi * ui - qj * uj - qk * uk);
  state->q[1] += 0.5f * dt * (qw * ui + qj * uk - qk * uj);
  state->q[2] += 0.5f * dt * (qw * uj - qi * uk + qk * ui);
  state->q[3] += 0.5f * dt * (qw * uk + qi * uj - qj * ui);
  // normalize, need it so the quaternion coordinate doesn't shrink to zero
  // bum ass rounding error why is 0.1+0.2 = 0.3000001 this is :/
  float q_mag = sqrtf(state->q[0] * state->q[0] + state->q[1] * state->q[1] +
                      state->q[2] * state->q[2] + state->q[3] * state->q[3]);
  state->q[0] /= q_mag;
  state->q[1] /= q_mag;
  state->q[2] /= q_mag;
  state->q[3] /= q_mag;
}
void EKF_Update(EKF_State_t *state, float ai, float aj, float ak) {
  // normalize data - convert from m/s² to g-units
  float a_mag = sqrtf(ai * ai + aj * aj + ak * ak);
  if (a_mag < 0.01f)
    return; // avoid divide by zero.

  // Convert to g-units (divide by 9.8) before checking magnitude
  float a_mag_g = a_mag / 9.8f;
  if (fabsf(a_mag_g - 1.0f) > 0.15f) {
    return; // ignore accel update during maneuver (>0.15g deviation)
  }

  // Normalize to unit vector
  ai /= a_mag;
  aj /= a_mag;
  ak /= a_mag;
  // get quaternion
  float qw = state->q[0];
  float qi = state->q[1];
  float qj = state->q[2];
  float qk = state->q[3];
  // predict gravity direction in body frame
  // FlightGear: Z-up, so gravity is in -Z direction
  float gx = 2.0f * (qw * qk - qi * qj);
  float gy = 2.0f * (qi * qw + qk * qj);
  float gz = (qw * qw - qi * qi - qj * qj + qk * qk);
  // error vector predicted x accelerator values
  float e[3];
  e[0] = (gy * ak - gz * aj);
  e[1] = (gz * ai - gx * ak);
  e[2] = (gx * aj - gy * ai);

  float P_angle[3] = {state->P[IND_ANGLE_X][IND_ANGLE_X],
                      state->P[IND_ANGLE_Y][IND_ANGLE_Y],
                      state->P[IND_ANGLE_Z][IND_ANGLE_Z]};

  float P_bias[3] = {state->P[IND_BIAS_X][IND_BIAS_X],
                     state->P[IND_BIAS_Y][IND_BIAS_Y],
                     state->P[IND_BIAS_Z][IND_BIAS_Z]};
  // measurement noise
  float R_accel = 0.05f;

  float K_angle[3];
  float K_bias[3];
  for (int i = 0; i < 3; ++i) {
    K_angle[i] = P_angle[i] / (P_angle[i] + R_accel);
    K_bias[i] = P_bias[i] / (P_bias[i] + R_accel);
  }

  // half of error with K values multiplied
  float half_e[3];
  for (int i = 0; i < 3; ++i) {
    half_e[i] = 0.5f * e[i] * K_angle[i];
  }
  // update quaternion orientation with error
  state->q[0] += (-qi * half_e[0] - qj * half_e[1] - qk * half_e[2]);
  state->q[1] += (qw * half_e[0] + qj * half_e[2] - qk * half_e[1]);
  state->q[2] += (qw * half_e[1] - qi * half_e[2] + qk * half_e[0]);
  state->q[3] += (qw * half_e[2] + qi * half_e[1] - qj * half_e[0]);
  // gyro bias correction (drifting)
  for (int i = 0; i < 3; ++i) {
    state->bias[i] += e[i] * K_bias[i] * 0.01f; // small learning curve
  }
  // reduce uncertainty
  for (int i = 0; i < 3; ++i) {
    state->P[i][i] *= (1.0f - K_angle[i]);
    state->P[i + 3][i + 3] *= (1.0f - K_bias[i]);
  }
  // normalize for fp calculation
  float mag = sqrtf(state->q[0] * state->q[0] + state->q[1] * state->q[1] +
                    state->q[2] * state->q[2] + state->q[3] * state->q[3]);
  state->q[0] /= mag;
  state->q[1] /= mag;
  state->q[2] /= mag;
  state->q[3] /= mag; // normalization
}
// initalize EKF
void EKF_Init(EKF_State_t *state) {
  // intialize quaternion coord with (1,0,0,0)
  state->q[0] = 1.0f;
  state->q[1] = 0.0f;
  state->q[2] = 0.0f;
  state->q[3] = 0.0f;
  // reset bias to zero
  state->bias[0] = 0.0f;
  state->bias[1] = 0.0f;
  state->bias[2] = 0.0f;
  // intialize covariance (diagonal ekf)
  state->P[0][0] = 0.1f;
  state->P[1][1] = 0.1f;
  state->P[2][2] = 0.1f;

  // gyro bias uncertainty
  state->P[3][3] = 1e-4f;
  state->P[4][4] = 1e-4f;
  state->P[5][5] = 1e-4f;
  state->PID KP_R = 1.0f; // Roll proportional gain
  state->PID KI_R = 0.0f; // Unused
  state->PID KD_R = 0.0f; // Unused

  state->PID KP_P = 1.0f; // Pitch proportional gain
  state->PID KI_P = 0.0f; // Unused
  state->PID KD_P = 0.0f; // Unused

  state->PID KP_Y = 0.0f; // No yaw control yet
  state->PID KI_Y = 0.0f;
  state->PID KD_Y = 0.0f;
}

void quaternion_to_body_x(float q[4], float x_body[3]) {
  float qw = q[0], qi = q[1], qj = q[2], qk = q[3];
  x_body[0] = 1.0f - 2.0f * (qj * qj + qk * qk);
  x_body[1] = 2.0f * (qi * qj + qw * qk);
  x_body[2] = 2.0f * (qi * qk - qw * qj);
}

void quaternion_to_body_y(float q[4], float y_body[3]) {
  float qw = q[0], qi = q[1], qj = q[2], qk = q[3];
  y_body[0] = 2.0f * (qi * qj - qw * qk);
  y_body[1] = 1.0f - 2.0f * (qi * qi + qk * qk);
  y_body[2] = 2.0f * (qj * qk + qw * qi);
}

float compute_aileron(EKF_State_t *state) {
  float y_body[3];
  quaternion_to_body_y(state->q, y_body);

  // Roll error: y_body[2] tells how much right wing is up/down
  // Positive = right wing down, negative = right wing up
  float roll_error = -y_body[2];

  // Proportional gain
  float Kp = state->PID KP_R;

  // Aileron command
  float aileron = Kp * roll_error;

  // Clamp to ±1.0
  if (aileron > 1.0f)
    aileron = 1.0f;
  if (aileron < -1.0f)
    aileron = -1.0f;

  return aileron;
}

float compute_elevator(EKF_State_t *state) {
  float x_body[3];
  quaternion_to_body_x(state->q, x_body);

  // Pitch error: x_body[2] tells how much nose is up/down
  // Positive = nose up, negative = nose down
  float pitch_error = x_body[2];

  // Proportional gain
  float Kp = state->PID KP_P;

  // Elevator command
  float elevator = Kp * pitch_error;

  // Clamp to ±1.0
  if (elevator > 1.0f)
    elevator = 1.0f;
  if (elevator < -1.0f)
    elevator = -1.0f;

  return elevator;
}

// function that runs when rx_byte arrives
void process_rx_data(uint16_t old_pos, uint16_t new_pos) {
  static char line[MESSAGE_LEN];
  static uint16_t idx = 0;

  while (old_pos != new_pos) {
    char c = rx_dma_buf[old_pos++];
    if (old_pos >= RX_DMA_SIZE) {
      old_pos = 0; // Wrap around
    }

    if (c == '\n') {
      line[idx] = '\0';
      memcpy((void *)rx_buffer, line, idx + 1);
      data_ready = 1; // Overwrite old packet - newest wins!
      idx = 0;
    } else if (idx < MESSAGE_LEN - 1) {
      line[idx++] = c;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    uart2_tx_busy = 0;
  } else if (huart->Instance == USART1) {
    uart1_tx_busy = 0;
  }
}

void uart_output(uint8_t port, char *msg) {
  if (port == 1) {
    if (uart1_tx_busy)
      return;
    uart1_tx_busy = 1;
    int len = snprintf(uart1_tx_buffer, sizeof(uart1_tx_buffer), "%s\n", msg);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart1_tx_buffer, len);
  } else if (port == 2) {
    if (uart2_tx_busy)
      return;
    uart2_tx_busy = 1;
    int len = snprintf(uart2_tx_buffer, sizeof(uart2_tx_buffer), "%s", msg);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)uart2_tx_buffer, len);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
