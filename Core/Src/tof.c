#include "tof.h"
#include "VL53L4CD_api.h"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"

int TOF_Setup(uint16_t address) {
    HAL_GPIO_WritePin(GPIOB, XSHUT_VL53L4CD_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOB, XSHUT_VL53L4CD_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    uint16_t sensor_ID;
    VL53L4CD_Error status = VL53L4CD_GetSensorId(address, &sensor_ID);
    if (status) {
        return status;
    } else if (sensor_ID != 0xebaa) {
        return VL53L4CD_ERROR_INCORRECT_SENSOR_ID_AT_ADDRESS;
    }

    return VL53L4CD_SensorInit(address);
}
