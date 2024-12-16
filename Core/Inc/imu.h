#include "lsm6ds3tr_platform.h"

stmdev_ctx_t IMU_Setup(lsm6ds3tr_handle *handle);

int32_t IMU_Calibrate(stmdev_ctx_t *imu_dev);

int32_t IMU_GetAccel(stmdev_ctx_t *imu_dev, float_t *results);
