#include <lsm6ds3tr-c_reg.h>
#include <lsm6ds3tr_platform.h>
#include <stdint.h>

/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                       uint16_t len) {
    lsm6ds3tr_handle *casted_handle = (lsm6ds3tr_handle *)handle;
    return HAL_I2C_Mem_Write(casted_handle->i2c_handle,
                             casted_handle->device_address, reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    lsm6ds3tr_handle *casted_handle = (lsm6ds3tr_handle *)handle;
    return HAL_I2C_Mem_Read(casted_handle->i2c_handle,
                            casted_handle->device_address, reg,
                            I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

/** Optional (may be required by driver) **/
void platform_delay(uint32_t millisec) { HAL_Delay(millisec); }
