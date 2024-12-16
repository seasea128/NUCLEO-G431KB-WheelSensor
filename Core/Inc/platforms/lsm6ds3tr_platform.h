#ifndef LSM6DS3TR_PLATFORM
#define LSM6DS3TR_PLATFORM

#include "lsm6ds3tr-c_reg.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef struct lsm6ds3tr_handle_s {
    uint8_t device_address;
    I2C_HandleTypeDef *i2c_handle;
    float xl_calibration_val[3];
} lsm6ds3tr_handle;

/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                       uint16_t len);

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/** Optional (may be required by driver) **/
void platform_delay(uint32_t millisec);

#endif
