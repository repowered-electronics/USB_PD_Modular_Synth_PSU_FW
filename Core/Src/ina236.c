
#include "ina236.h"

void ina236_set_reg(INA236_t* ina, uint8_t reg, uint16_t value){
    uint8_t reg_data[] = {reg, (value >> 8) & 0xFF, value & 0xFF};
    HAL_I2C_Master_Transmit(ina->hi2c, &reg, ina->addr << 1, 3, 10); // transmit pointer byte + 2 data bytes
}


uint16_t ina236_read_reg(INA236_t* ina, uint8_t reg){
    HAL_I2C_Master_Transmit(ina->hi2c, &reg, ina->addr << 1, 1, 10); // transmit pointer byte first
    uint8_t regdata[2];
    HAL_I2C_Master_Receive(ina->hi2c, regdata, ina->addr << 1, 2, 10); // read 2 bytes of data
    return (uint16_t)((regdata[0] << 8) | regdata[1]);
}


void ina236_general_call_reset(INA236_t* ina){
    uint8_t data = 0x06;
    HAL_I2C_Master_Transmit(ina->hi2c, &data, 0, 1, 10); // reset all INA236's
}


void ina236_init(INA236_t* ina){
    ina->config = ina236_read_reg(ina, INA236_REG_CONFIG);
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
    ina_set_reg(ina, INA236_REG_CONFIG, ina->config);
}


float ina236_get_current(INA236_t* ina){
    uint16_t regval = ina236_read_reg(ina, INA236_REG_CURRENT);
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