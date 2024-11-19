#ifndef TOF_USER
#define TOF_USER

#include <stdint.h>
#define VL53L4CD_DEFAULT_I2C_ADDRESS 0x52

#define VL53L4CD_ERROR_NONE ((uint8_t)0U)
#define VL53L4CD_ERROR_INCORRECT_SENSOR_ID_AT_ADDRESS ((uint8_t)252U)
#define VL53L4CD_ERROR_XTALK_FAILED ((uint8_t)253U)
#define VL53L4CD_ERROR_INVALID_ARGUMENT ((uint8_t)254U)
#define VL53L4CD_ERROR_TIMEOUT ((uint8_t)255U)

int TOF_Setup(uint16_t address);

#endif
