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
#include "fdcan.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L4CD_api.h"
#include "imu.h"
#include "lsm6ds3tr-c_reg.h"
#include "lsm6ds3tr_platform.h"
#include "state.h"
#include "tof.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOF_ADDRESS VL53L4CD_DEFAULT_I2C_ADDRESS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct int_flag_s {
    bool TOF_INT, IMU_INT_1, IMU_INT_2 : 1;
} int_flag;
volatile int_flag INT_STATUS;
VL53L4CD_ResultsData_t results;
uint16_t tofResult;
uint16_t imu1NotReadyCount;
uint16_t imu2NotReadyCount;

// TODO: Move this to local variable after testing
state current_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
    case INT_VL53L4CD_Pin: {
        INT_STATUS.TOF_INT = true;
        break;
    }
    case INT1_LSM6DS3TR1_Pin: {
        INT_STATUS.IMU_INT_1 = true;
        break;
    }
    case INT1_LSM6DS3TR2_Pin: {
        INT_STATUS.IMU_INT_2 = true;
        break;
    }
    default: {
        printf("Unk int hit: %x\r\n", GPIO_Pin);
        break;
    }
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */
    int status;

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
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
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

    status = HAL_FDCAN_Start(&hfdcan1);

    if (status) {
        printf("Cannot start FDCAN: %x\r\n", status);
        return status;
    }

    printf("Serial initialized\r\n");

    status = TOF_Setup(TOF_ADDRESS);

    if (status) {
        printf("Cannot setup VL53L4CD: %x\r\n", status);
        return status;
    }

    status = VL53L4CD_SetRangeTiming(TOF_ADDRESS, 10, 0);

    if (status) {
        printf("Cannot set range of VL53L4CD: %x\r\n", status);
        return status;
    }

    status = VL53L4CD_StartRanging(TOF_ADDRESS);

    if (status) {
        printf("Cannot start ranging of VL53L4CD: %x\r\n", status);
        return status;
    }

    // FDCAN header
    FDCAN_TxHeaderTypeDef header;
    header.Identifier = SENSOR_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_FRAME_FD_NO_BRS;
    header.DataLength = FDCAN_DLC_BYTES_2;
    header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    lsm6ds3tr_handle handle1 = {.i2c_handle = &hi2c1,
                                .device_address = LSM6DS3TR_C_I2C_ADD_L};

    lsm6ds3tr_handle handle2 = {.i2c_handle = &hi2c1,
                                .device_address = LSM6DS3TR_C_I2C_ADD_H};

    // According to docs, LSM6DS3TR-C needs 15ms to startup, should be good
    // enough to just wait 15ms before setup.
    HAL_Delay(150);

    // printf("Setting up IMU1\r\n");
    stmdev_ctx_t imu1 = IMU_Setup(&handle1);
    // if (imu1.handle == 0) {
    //     // TODO: More logging
    //     return 127;
    // }
    //  printf("Calibrating IMU1\r\n");
    //  status = IMU_Calibrate(&imu1);
    //  if (status != HAL_OK) {
    //      printf("Failed to calibrate IMU1: %d\r\n", status);
    //      return -123;
    //  }

    // printf("Setting up IMU2\r\n");
    stmdev_ctx_t imu2 = IMU_Setup(&handle2);
    // if (imu2.handle == 0) {
    //     // TODO: More logging
    //     return 127;
    // }
    //  printf("Calibrating IMU2\r\n");
    //  status = IMU_Calibrate(&imu2);
    //  if (status != HAL_OK) {
    //      printf("Failed to calibrate IMU2: %d\r\n", status);
    //      return -123;
    //  }

    current_state = state_init();

    current_state.tof_distance = 10;
    current_state.current_accel_orthogonal = 0.f;

    imu1NotReadyCount = 0;
    imu2NotReadyCount = 0;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        if (INT_STATUS.TOF_INT) {
            INT_STATUS.TOF_INT = false;
            int status = VL53L4CD_GetResult(TOF_ADDRESS, &results);
            if (status) {
                printf("Cannot get result from VL53L4CD: %x\r\n", status);
                continue;
            }
            if (results.range_status == 0) {
                tofResult = results.distance_mm;
                printf("TOF result got: %d\r\n", tofResult);
                printf("Sending new CAN message\r\n");
                int status = HAL_FDCAN_AddMessageToTxFifoQ(
                    &hfdcan1, &header, (uint8_t *)(&tofResult));

                if (status != HAL_OK && status != 1) {
                    printf("Cannot add message to FDCAN: %x\r\n", status);
                    status = VL53L4CD_ClearInterrupt(TOF_ADDRESS);
                    if (status) {
                        printf("Cannot clear interrupt from VL53L4CD: %x\r\n",
                               status);
                        continue;
                    }
                }
            }
            status = VL53L4CD_ClearInterrupt(TOF_ADDRESS);
            if (status) {
                printf("Cannot clear interrupt from VL53L4CD: %x\r\n", status);
                continue;
            }

            state_update_vel(&current_state, tofResult, results.sigma_mm);
        } else if (INT_STATUS.IMU_INT_1) {
            // TODO: Implement 1D state Filter to fuse IMU with ToF
            INT_STATUS.IMU_INT_1 = false;

            int result = IMU_GetAccel(&imu1, current_state.imu1_results);
            if (result != HAL_OK && result != -23) {
                printf("Failed to get result from IMU1: %d\r\n", result);
            } else if (result == -23) {
                imu1NotReadyCount++;
            } else {
                imu1NotReadyCount = 0;
                printf("IMU1 Result retrieved: %x,%x,%x\r\n",
                       *(unsigned int *)&current_state.imu1_results[0],
                       *(unsigned int *)&current_state.imu1_results[1],
                       *(unsigned int *)&current_state.imu1_results[2]);
            }
        } else if (INT_STATUS.IMU_INT_2) {
            INT_STATUS.IMU_INT_2 = false;

            int result = IMU_GetAccel(&imu2, current_state.imu2_results);
            if (result != HAL_OK && result != -23) {
                printf("Failed to get result from IMU2: %d\r\n", result);
            } else if (result == 23) {
                imu2NotReadyCount++;
            } else {
                imu2NotReadyCount = 0;
                printf("IMU2 Result retrieved: %x,%x,%x\r\n",
                       *(unsigned int *)&current_state.imu2_results[0],
                       *(unsigned int *)&current_state.imu2_results[1],
                       *(unsigned int *)&current_state.imu2_results[2]);

                state_update_accel(&current_state);
                state_predict_next(&current_state);
            }
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    status = VL53L4CD_StopRanging(TOF_ADDRESS);
    if (status) {
        printf("Cannot stop ranging VL53L4CD: %x\r\n", status);
        return status;
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
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return
     * state
     */
    __disable_irq();
    printf("Error occured\r\n");
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
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
