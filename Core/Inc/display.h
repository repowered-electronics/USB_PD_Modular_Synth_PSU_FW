#include "stm32f1xx_hal.h"
#include "u8g2/u8g2.h"

typedef struct display_struct {
    u8g2_t* oled;
    uint8_t addr;
    uint8_t adc_range;
    float shunt; // shunt resistance in Ohms
    float current_LSB;
    uint16_t shunt_cal; //shunt cal value
    uint16_t config;
    uint16_t int_pin;
} display_t;