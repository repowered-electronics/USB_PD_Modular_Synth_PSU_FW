/**
  ******************************************************************************
  * @file           : ina236.c
  * @brief          : INA236 functions and such
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
#include "ina236.h"

void ina236_set_reg(INA236_t* ina, uint8_t reg, uint16_t value){
    uint8_t reg_data[] = {reg, (value >> 8) & 0xFF, value & 0xFF};
    HAL_I2C_Master_Transmit(ina->hi2c, ina->addr << 1, reg_data, 3, 10); // transmit pointer byte + 2 data bytes
}


uint16_t ina236_read_reg(INA236_t* ina, uint8_t reg){
    HAL_I2C_Master_Transmit(ina->hi2c, ina->addr << 1, &reg, 1, 10); // transmit pointer byte first
    uint8_t regdata[2];
    HAL_I2C_Master_Receive(ina->hi2c, ina->addr << 1, regdata, 2, 10); // read 2 bytes of data
    return (uint16_t)((regdata[0] << 8) | regdata[1]);
}


void ina236_general_call_reset(INA236_t* ina){
    uint8_t data = 0x06;
    HAL_I2C_Master_Transmit(ina->hi2c, 0, &data, 1, 10); // reset all INA236's
}


void ina236_init(INA236_t* ina){
    ina->config = ina236_read_reg(ina, INA236_REG_CONFIG);
    ina236_set_reg(ina, INA236_REG_CONFIG, 0x43FF);
}


void ina236_reset(INA236_t* ina){
    // ina->config = ina236_read_reg(ina, INA236_REG_CONFIG);
    CONFIG_RST(ina->config);
    ina236_set_reg(ina, INA236_REG_CONFIG, ina->config);
    ina->config = ina236_read_reg(ina, INA236_REG_CONFIG);
}

void ina236_set_shunt_range(INA236_t* ina, uint8_t range){
    ina->config = ina236_read_reg(ina, INA236_REG_CONFIG);

    if(range == 0){
        ina->adc_range = 0;
        CONFIG_ADCRANGE_81_92MV(ina->config);
        ina->current_LSB = 0.0000025 / ina->shunt;
    }else{
        ina->adc_range = 1;
        CONFIG_ADCRANGE_20_48MV(ina->config);
        ina->current_LSB = 0.000000625 / ina->shunt;
    }
    ina236_set_reg(ina, INA236_REG_CONFIG, ina->config);
}


void ina236_set_shuntcal(INA236_t* ina) {
    float scal = 0.00512 / ((ina->shunt)*(ina->current_LSB));
    ina->shunt_cal = (int)scal;
    ina236_set_reg(ina, INA236_REG_CALIBRATION, ina->shunt_cal);
}

float ina236_get_current(INA236_t* ina){
    int16_t regval = ina236_read_reg(ina, INA236_REG_CURRENT);
    return ina->current_LSB * (float)regval;
}


float ina236_get_voltage(INA236_t* ina){
    uint16_t regval = ina236_read_reg(ina, INA236_REG_BUS_VOLTAGE);
    return INA236_VOLTAGE_LSB * (float)regval;
}


float ina236_get_power(INA236_t* ina){
    uint16_t regval = ina236_read_reg(ina, INA236_REG_POWER);
    return 32.0 * ina->current_LSB * (float)regval;
}

void ina236_set_current_limit(INA236_t* ina, float limit){
    int16_t SOL_lim = (limit) / ina->current_LSB;
    ina236_set_reg(ina, INA236_REG_ALERT_LIMIT, SOL_lim);
}

void ina236_set_alertSOL(INA236_t* ina) {
    ina236_set_reg(ina, INA236_REG_MASK_ENABLE, 0x8000);
}