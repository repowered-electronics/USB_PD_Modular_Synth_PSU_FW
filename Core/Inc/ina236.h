/**
  ******************************************************************************
  * @file           : ina236.h
  * @brief          : INA236 header file
  ******************************************************************************
  * @attention
  * Copyright (C) 2024  Repowered Electronics LLC
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU Lesser General Public License for more details.
  * 
  * You should have received a copy of the GNU Lesser General Public License
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.
  ******************************************************************************
  */
#ifndef _INA236_H_
#define _INA236_H_

#include "stm32f1xx_hal.h"

#define INA236_REG_SIZE_BYTES       2

/* REGISTER MAP */
#define INA236_REG_CONFIG           0x00
#define INA236_REG_SHUNT_VOLTAGE    0x01
#define INA236_REG_BUS_VOLTAGE      0x02
#define INA236_REG_POWER            0x03
#define INA236_REG_CURRENT          0x04
#define INA236_REG_CALIBRATION      0x05
#define INA236_REG_MASK_ENABLE      0x06
#define INA236_REG_ALERT_LIMIT      0x07
#define INA236_REG_MFG_ID           0x3E
#define INA236_REG_DEVICE_ID        0x3F

/* REGISTER DEFAULTS */
#define INA236_CONFIG_DFLT          0x4127
#define INA236_MFG_ID_DFLT          0x5449
#define INA236_DEVICE_ID_DFLT       0xA080

/* CONFIG REGISTER BIT SHIFTS */
#define RST_BIT                     15
#define ADCRANGE_BIT                12
#define AVG_BIT                     9
#define VBUSCT_BIT                  6
#define VSHCT_BIT                   3
#define MODE_BIT                    0

/* CONFIG REGISTER VALUES */
#define CONFIG_RST(x)               (x |= (1 << RST_BIT))
#define CONFIG_ADCRANGE_81_92MV(x)  (x &= ~(1 << ADCRANGE_BIT))
#define CONFIG_ADCRANGE_20_48MV(x)  (x |= (1 << ADCRANGE_BIT))
#define AVG_COUNT_1                 (0)
#define AVG_COUNT_4                 (1 << AVG_BIT)
#define AVG_COUNT_16                (2 << AVG_BIT)
#define AVG_COUNT_64                (3 << AVG_BIT)
#define AVG_COUNT_128               (4 << AVG_BIT)
#define AVG_COUNT_256               (5 << AVG_BIT)
#define AVG_COUNT_512               (6 << AVG_BIT)
#define AVG_COUNT_1024              (7 << AVG_BIT)
#define VBUS_CONV_TIME_140US        (0)
#define VBUS_CONV_TIME_204US        (1 << VBUSCT_BIT)
#define VBUS_CONV_TIME_332US        (2 << VBUSCT_BIT)
#define VBUS_CONV_TIME_588US        (3 << VBUSCT_BIT)
#define VBUS_CONV_TIME_1100US       (4 << VBUSCT_BIT)
#define VBUS_CONV_TIME_2116US       (5 << VBUSCT_BIT)
#define VBUS_CONV_TIME_4156US       (6 << VBUSCT_BIT)
#define VBUS_CONV_TIME_8244US       (7 << VBUSCT_BIT)
#define VBUS_CONV_TIME_204US        (1 << VBUSCT_BIT)
#define VSHUNT_CONV_TIME_332US      (2 << VSHCT_BIT)
#define VSHUNT_CONV_TIME_588US      (3 << VSHCT_BIT)
#define VSHUNT_CONV_TIME_1100US     (4 << VSHCT_BIT)
#define VSHUNT_CONV_TIME_2116US     (5 << VSHCT_BIT)
#define VSHUNT_CONV_TIME_4156US     (6 << VSHCT_BIT)
#define VSHUNT_CONV_TIME_8244US     (7 << VSHCT_BIT)
#define MODE_SHUTDOWN               (0)
#define MODE_SINGLE_SHUNT_VOLTAGE   (1)
#define MODE_SINGLE_BUS_VOLTAGE     (2)
#define MODE_SINGLE_SHUNT_AND_BUS   (3)
#define MODE_SHUTDOWN_ALSO          (4)
#define MODE_CONT_SHUNT_VOLTAGE     (5)
#define MODE_CONT_BUS_VOLTAGE       (6)
#define MODE_CONT_SHUNT_AND_BUS     (7)

#define INA236_VOLTAGE_LSB          0.0016

typedef struct ina236_struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr;
    uint8_t adc_range;
    float shunt; // shunt resistance in Ohms
    float current_LSB;
    uint16_t shunt_cal; //shunt cal value
    uint16_t config;
    uint16_t int_pin;
} INA236_t;

void ina236_set_reg(INA236_t* ina, uint8_t reg, uint16_t value);
uint16_t ina236_read_reg(INA236_t* ina, uint8_t reg);
void ina236_general_call_reset(INA236_t* ina);
void ina236_init(INA236_t* ina);
void ina236_reset(INA236_t* ina);
void ina236_set_shunt_range(INA236_t* ina, uint8_t range);
void ina236_set_shuntcal(INA236_t* ina);
float ina236_get_current(INA236_t* ina);
float ina236_get_voltage(INA236_t* ina);
float ina236_get_power(INA236_t* ina);
void ina236_set_current_limit(INA236_t* ina, float limit);
void ina236_set_alertSOL(INA236_t* ina);
#endif // _INA236_H_