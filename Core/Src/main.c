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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct { // quaternion coordinate w+xi+yj+zk
  float w;
  float i;
  float j;
  float k;
} Quat_t;
typedef struct {
  float q[4];    // store quaternion
  float bias[3]; // gyro bias (booty cheeks)
  float P[7][7]; // uncertainty 1-4 is for orientation uncertainty(w,i,j,k) the
                 // rest is for gyro bias(i,j,k)

  // PID State Variables
  float roll_integral;
  float pitch_integral;
  float yaw_integral;
  float last_roll_error;
  float last_pitch_error;
  float last_yaw_error;
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

/* USER CODE BEGIN PV */
// priv var
uint8_t rx_byte;        // temp variable for 1 char
uint8_t rx_buffer[128]; // full message storage
int rx_index = 0;       // current pos in rx_buffer
uint8_t data_ready = 0; // flag to check if we have full packet
EKF_State_t ekf;        // Global

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void EKF_Predict(EKF_State_t *state, float gi, float gj, float gk, float dt);
void EKF_Update(EKF_State_t *state, float ai, float aj, float ak);
void EKF_Init(EKF_State_t *state);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void logging(char *log_msg);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code
   ---------------------------------------------------------*/
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // listen for 1 byte, save to rx_byte, and interrupt when done
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  EKF_Init(&ekf);
  float sim_time, ax, ay, az, gx, gy, gz;
  float last_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // 1. Did we get a packet?
    if (data_ready == 1) {
      data_ready = 0; // Reset flag

      // 2. Try to read the 7 numbers
      // Format: Time, Ax, Ay, Az, Gx, Gy, Gz
      int items = sscanf((char *)rx_buffer, "%f,%f,%f,%f,%f,%f,%f", &sim_time,
                         &ax, &ay, &az, &gx, &gy, &gz);

      // 3. Only blink if the packet is PERFECT (items == 7)
      if (items == 7) {
        // lwk just there to check if it calculating
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        // Calculate time difference
        float dt = sim_time - last_time;
        if (dt < 0.001f || dt > 0.05f)
          dt = 0.01f; // safety net
        last_time = sim_time;

        // Run the Math
        EKF_Predict(&ekf, gx, gy, gz, dt);
        EKF_Update(&ekf, ax, ay, az);

        if (isnan(ekf.q[0]) || isnan(ekf.q[1]) || isnan(ekf.q[2]) ||
            isnan(ekf.q[3])) {
          // Reset EKF if it breaks
          EKF_Init(&ekf);
          continue; // Skip this iteration
        }

        float qw = ekf.q[0];
        float qx = ekf.q[1];
        float qy = ekf.q[2];
        float qz = ekf.q[3];

        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        float roll_est = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (qw * qy - qz * qx);
        if (sinp > 1.0f)
          sinp = 1.0f;
        if (sinp < -1.0f)
          sinp = -1.0f;
        float pitch_est = asinf(sinp);

        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        //             float yaw_est = atan2f(siny_cosp, cosy_cosp);

        // PID Gains
        // experimental values
        float kp_r = 0.4f;
        float kd_r = 0.45f;

        float kp_p = 0.6f;
        float kd_p = 0.25f;

        //			 float kp_y = 0.5f;
        //			 float ki_y = 0.0f;
        //			 float kd_y = 0.1f;

        // roll pid
        float roll_error = roll_est; // goal is 0.0f

        float roll_rate = gx;
        ekf.last_roll_error = roll_error;

        // pitch pid
        float target_pitch = 0.0f;
        float pitch_rate = gy * cos(roll_est) - gz * sin(roll_est);
        float pitch_error = target_pitch - pitch_est; // goal_is 0.0f

        ekf.last_pitch_error = pitch_error;

        ekf.roll_integral += roll_error * dt;
        ekf.pitch_integral += pitch_error * dt;

        // anti-windup
        if (ekf.roll_integral > 2.0f)
          ekf.roll_integral = 2.0f;
        if (ekf.roll_integral < -2.0f)
          ekf.roll_integral = -2.0f;
        if (ekf.pitch_integral > 2.0f)
          ekf.pitch_integral = 2.0f;
        if (ekf.pitch_integral < -2.0f)
          ekf.pitch_integral = -2.0f;

        float ki_r = 0.1f; // start small
        float ki_p = 0.1f;

        float aileron_cmd =
            -(kp_r * roll_error + ki_r * ekf.roll_integral - kd_r * roll_rate);
        aileron_cmd = aileron_cmd * (-1.0f);
        float elevator_cmd =
            kp_p * pitch_error + ki_p * ekf.pitch_integral - kd_p * pitch_rate;
        elevator_cmd = elevator_cmd * (-1.0f);

        // yaw pid
        //			 float yaw_error = yaw_est; //goal_is 0.0f
        //
        //			 // integral with anti windup
        //			 ekf.yaw_integral += yaw_error * dt;
        //			 if (ekf.yaw_integral > 2.0f) ekf.yaw_integral
        //= 2.0f; 			 if (ekf.yaw_integral < -2.0f)
        // ekf.yaw_integral = -2.0f; 			 float yaw_derivative =
        // 0.0f; 			 float yaw_rate = gz;
        // float rudder_cmd = -(kp_y
        //* yaw_error
        //			                    + ki_y * ekf.yaw_integral
        //			                    - kd_y * yaw_rate);
        //
        //			 ekf.last_yaw_error = yaw_error;

        // clamping
        if (aileron_cmd > 1.0f)
          aileron_cmd = 1.0f;
        if (aileron_cmd < -1.0f)
          aileron_cmd = -1.0f;
        if (elevator_cmd > 1.0f)
          elevator_cmd = 1.0f;
        if (elevator_cmd < -1.0f)
          elevator_cmd = -1.0f;
        //			 if (rudder_cmd > 1.0f) rudder_cmd = 1.0f;
        //			 if (rudder_cmd < -1.0f) rudder_cmd = -1.0f;

        char control_msg[128];
        snprintf(control_msg, sizeof(control_msg), "%.4f,%.4f\n", aileron_cmd,
                 elevator_cmd);
        HAL_UART_Transmit(&huart2, (uint8_t *)control_msg, strlen(control_msg),
                          10);

        char quaternion[128];
        snprintf(quaternion, sizeof(quaternion), "%.4f, %.4f, %.4f, %4f\n",
                 ekf.q[0], ekf.q[1], ekf.q[2], ekf.q[3]);
        logging(quaternion);
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
  // ekf is too confidenct if its just set as 1
  state->P[0][0] += 0.001f * dt;
  state->P[1][1] += 0.001f * dt;
  state->P[2][2] += 0.001f * dt;
  state->P[3][3] += 0.001f * dt;
  state->P[4][4] += 1e-6f * dt;
  state->P[5][5] += 1e-6f * dt;
  state->P[6][6] += 1e-6f * dt;

  // subtract bias :)
  float ui = gi - state->bias[0];
  float uj = gj - state->bias[1];
  float uk = gk - state->bias[2];
  // grab quaternion values into a float so its easy to read
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
  // normalize, need it so the flight calculation doesn't shrink to zero
  // bum ass rounding error why is 0.1+0.2 = 0.3000001 this is :/
  float msg = sqrtf(state->q[0] * state->q[0] + state->q[1] * state->q[1] +
                    state->q[2] * state->q[2] + state->q[3] * state->q[3]);
  state->q[0] /= msg;
  state->q[1] /= msg;
  state->q[2] /= msg;
  state->q[3] /= msg;
}
void EKF_Update(EKF_State_t *state, float ai, float aj, float ak) {
  // normalize data
  float a_mag = sqrtf(ai * ai + aj * aj + ak * ak);
  if (fabsf(a_mag - 1.0f) > 0.15f) {
    return; // ignore accel update during maneuver
  }
  if (a_mag < 0.01f)
    return; // avoid divide by zero.
  ai /= a_mag;
  aj /= a_mag;
  ak /= a_mag;
  // get quaternion
  float qw = state->q[0];
  float qi = state->q[1];
  float qj = state->q[2];
  float qk = state->q[3];
  // predict gravity
  float hi = 2.0f * (qi * qk - qw * qj);
  float hj = 2.0f * (qw * qi + qj * qk);
  float hk = qw * qw - qi * qi - qj * qj + qk * qk;
  // error vector
  float ex = (hj * ak - hk * aj);
  float ey = (hk * ai - hi * ak);
  float ez = (hi * aj - hj * ai);
  float P_angle = state->P[0][0]; // Uncertainty in orientation
  float P_bias = state->P[4][4];  // Uncertainty in bias
  // measurement noise
  float R_accel = 0.05f;
  // k value for which to trust
  float K_angle = P_angle / (P_angle + R_accel);
  float K_bias = P_bias / (P_bias + R_accel);
  // half of error with K values multiplied
  float half_ex = 0.5f * ex * K_angle;
  float half_ey = 0.5f * ey * K_angle;
  float half_ez = 0.5f * ez * K_angle;
  // update quaternion orientation with error
  state->q[0] += (-qi * half_ex - qj * half_ey - qk * half_ez);
  state->q[1] += (qw * half_ex + qj * half_ez - qk * half_ey);
  state->q[2] += (qw * half_ey - qi * half_ez + qk * half_ex);
  state->q[3] += (qw * half_ez + qi * half_ey - qj * half_ex);
  // gyro bias correction (drifting)
  state->bias[0] += -ex * K_bias * 0.01f; // Small learning rate
  state->bias[1] += -ey * K_bias * 0.01f;
  state->bias[2] += -ez * K_bias * 0.01f;
  // reduce uncertainty
  state->P[0][0] -= K_angle * state->P[0][0];
  state->P[1][1] -= K_angle * state->P[1][1];
  state->P[2][2] -= K_angle * state->P[2][2];
  state->P[3][3] -= K_angle * state->P[3][3];
  // normalize for fp calculation
  float mag = sqrtf(state->q[0] * state->q[0] + state->q[1] * state->q[1] +
                    state->q[2] * state->q[2] + state->q[3] * state->q[3]);
  state->q[0] /= mag;
  state->q[1] /= mag;
  state->q[2] /= mag;
  state->q[3] /= mag;
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
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      state->P[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  // INitialize PID state
  state->roll_integral = 0.0f;
  state->pitch_integral = 0.0f;
  state->yaw_integral = 0.0f;
  state->last_roll_error = 0.0f;
  state->last_pitch_error = 0.0f;
  state->last_yaw_error = 0.0f;
}

// function that runs when rx_byte arrives
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (rx_byte == '\n') {
      rx_buffer[rx_index] = '\0';
      data_ready = 1;
      rx_index = 0;
    } else {
      if (rx_index < 127) { // <--- Add this safety belt back
        rx_buffer[rx_index++] = rx_byte;
      }
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

void logging(char *log_msg) {
  char out_msg[128];
  snprintf(out_msg, sizeof(out_msg), "%s\n", log_msg);
  HAL_UART_Transmit(&huart1, (uint8_t *)out_msg, strlen(out_msg), 10);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
