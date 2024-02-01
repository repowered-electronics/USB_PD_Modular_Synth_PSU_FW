#include "stm32f1xx_hal.h"

typedef struct display_struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr;
    uint8_t adc_range;
    float shunt; // shunt resistance in Ohms
    float current_LSB;
    uint16_t shunt_cal; //shunt cal value
    uint16_t config;
    uint16_t int_pin;
} display_t;