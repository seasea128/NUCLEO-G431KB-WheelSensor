#include "imu.h"
#include "lsm6ds3tr-c_reg.h"
#include "lsm6ds3tr_platform.h"
#include <stdio.h>

#define IMU_CALIBRATION_COUNT 500

stmdev_ctx_t IMU_Setup(lsm6ds3tr_handle *handle) {
    stmdev_ctx_t imu_ctx = {.write_reg = platform_write,
                            .read_reg = platform_read,
                            .mdelay = platform_delay,
                            .handle = handle};
    uint8_t device_id;
    int32_t result;
    printf("Getting device ID\r\n");
    result = lsm6ds3tr_c_device_id_get(&imu_ctx, &device_id);
    if (result != HAL_OK) {
        // TODO: Handle error
        printf("IMU: Failed to get device ID: %d\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    if (device_id != LSM6DS3TR_C_ID) {
        // TODO: Handle error
        printf("Device ID unexpected: %x, expected %x\r\n", device_id,
               LSM6DS3TR_C_ID);
        return (stmdev_ctx_t){.handle = 0};
    }

    printf("Device ID: %x\r\n", device_id);

    // result = lsm6ds3tr_c_block_data_update_set(&imu_ctx, PROPERTY_ENABLE);
    // if (result != HAL_OK) {
    //     printf("Failed to set block data update: %ld\r\n", result);
    //     return (stmdev_ctx_t){.handle = 0};
    // }

    result = lsm6ds3tr_c_xl_data_rate_set(&imu_ctx, LSM6DS3TR_C_XL_ODR_833Hz);
    if (result != HAL_OK) {
        printf("Failed to set data rate: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    result = lsm6ds3tr_c_xl_full_scale_set(&imu_ctx, LSM6DS3TR_C_4g);
    if (result != HAL_OK) {
        printf("Failed to set scaling: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    printf("Finished setting update rate\r\n");

    result =
        lsm6ds3tr_c_xl_filter_analog_set(&imu_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
    if (result != HAL_OK) {
        printf("Failed to set analog filter: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    result = lsm6ds3tr_c_xl_lp1_bandwidth_set(&imu_ctx,
                                              LSM6DS3TR_C_XL_LP1_ODR_DIV_2);
    if (result != HAL_OK) {
        printf("Failed to set lp1 bandwidth: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    printf("Finished setting filter\r\n");

    // TODO: Set interrupt mode (pullup, etc)
    result = lsm6ds3tr_c_pin_mode_set(&imu_ctx, LSM6DS3TR_C_PUSH_PULL);
    if (result != HAL_OK) {
        printf("Failed to set pin mode: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }
    result = lsm6ds3tr_c_pin_polarity_set(&imu_ctx, LSM6DS3TR_C_ACTIVE_HIGH);
    if (result != HAL_OK) {
        printf("Failed to set pin polarity: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }
    result =
        lsm6ds3tr_c_int_notification_set(&imu_ctx, LSM6DS3TR_C_INT_LATCHED);
    if (result != HAL_OK) {
        printf("Failed to set interrupt notification mode: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    result =
        lsm6ds3tr_c_pin_int2_route_set(&imu_ctx, (lsm6ds3tr_c_int2_route_t){
                                                     .int2_drdy_xl = 0,
                                                     .int2_drdy_g = 0,
                                                     .int2_drdy_temp = 0,
                                                     .int2_fth = 0,
                                                     .int2_fifo_ovr = 0,
                                                     .int2_full_flag = 0,
                                                     .int2_step_count_ov = 0,
                                                     .int2_step_delta = 0,
                                                     .int2_iron = 0,
                                                     .int2_tilt = 0,
                                                     .int2_6d = 0,
                                                     .int2_double_tap = 0,
                                                     .int2_ff = 0,
                                                     .int2_wu = 0,
                                                     .int2_single_tap = 0,
                                                     .int2_inact_state = 0,
                                                     .int2_wrist_tilt = 0,
                                                 });

    result =
        lsm6ds3tr_c_pin_int1_route_set(&imu_ctx, (lsm6ds3tr_c_int1_route_t){
                                                     .int1_drdy_xl = 1,
                                                     .int1_drdy_g = 0,
                                                     .int1_boot = 0,
                                                     .int1_fth = 0,
                                                     .int1_fifo_ovr = 0,
                                                     .int1_full_flag = 0,
                                                     .int1_sign_mot = 0,
                                                     .int1_step_detector = 0,
                                                     .int1_timer = 0,
                                                     .int1_tilt = 0,
                                                     .int1_6d = 0,
                                                     .int1_double_tap = 0,
                                                     .int1_ff = 0,
                                                     .int1_wu = 0,
                                                     .int1_single_tap = 0,
                                                     .int1_inact_state = 0,
                                                     .den_drdy_int1 = 0,
                                                     .drdy_on_int1 = 0,
                                                 });

    printf("Finished setting interrupt\r\n");

    if (result != HAL_OK) {
        printf("Failed to set interrupts: %ld\r\n", result);
        return (stmdev_ctx_t){.handle = 0};
    }

    // If the sensor is started up before the MCU, the interrupt line will be
    // stuck at high, need to reset interrupt by reading data because STM32
    // doesn't have level-based interrupt, only edge interrupt.
    lsm6ds3tr_c_reg_t reg;
    lsm6ds3tr_c_status_reg_get(&imu_ctx, &reg.status_reg);
    if (reg.status_reg.xlda) {
        int16_t data[3];
        int result = lsm6ds3tr_c_acceleration_raw_get(&imu_ctx, data);
        if (result != HAL_OK) {
            printf("Failed to retrieve initial data: %ld\r\n", result);
            return (stmdev_ctx_t){.handle = 0};
        }
    }

    return imu_ctx;
}

int32_t IMU_Calibrate(stmdev_ctx_t *imu_dev) {
    // Take 10 value and average it to get calibration value
    lsm6ds3tr_handle *handle = (lsm6ds3tr_handle *)imu_dev->handle;
    int count = 0;
    float_t results[3] = {0.f, 0.f, 0.f};
    while (count < IMU_CALIBRATION_COUNT) {
        float_t accel[3] = {0.f, 0.f, 0.f};
        int result = IMU_GetAccel(imu_dev, accel);
        if (result == HAL_OK) {
            for (int i = 0; i < 3; i++) {
                results[i] += accel[i];
            }
            count++;
        } else if (result != -23) {
            return result;
        }
    }

    for (int i = 0; i < 3; i++) {
        results[i] /= IMU_CALIBRATION_COUNT;
        // handle->xl_calibration_val[i] = results[i];
    }

    return 0;
}

int32_t IMU_GetAccel(stmdev_ctx_t *imu_dev, float_t *results) {
    lsm6ds3tr_c_reg_t reg;
    lsm6ds3tr_c_status_reg_get(imu_dev, &reg.status_reg);
    if (!reg.status_reg.xlda) {
        return -23;
    }
    int16_t data[3];
    int result = lsm6ds3tr_c_acceleration_raw_get(imu_dev, data);
    if (result != HAL_OK) {
        return result;
    }

    lsm6ds3tr_handle *handle = (lsm6ds3tr_handle *)imu_dev->handle;

    for (int i = 0; i < 3; i++) {
        results[i] = lsm6ds3tr_c_from_fs4g_to_mg(data[i]) -
                     handle->xl_calibration_val[i];
    }

    return 0;
}
