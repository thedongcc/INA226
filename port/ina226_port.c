#include "ina226_port.h"
#include "ina226_api.h"



ina226_err_t ina226_port_i2c_write_reg(ina226_i2c_port_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data, uint8_t size)
{
    return (ina226_err_t)HAL_I2C_Mem_Write(i2c.handle, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                                           (uint8_t*)&reg_data, size, 0xFF);
}

ina226_err_t ina226_port_i2c_read_reg(ina226_i2c_port_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t *reg_data, uint8_t size)
{
    return (ina226_err_t)HAL_I2C_Mem_Read(i2c.handle, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,
                                          (uint8_t*)reg_data, size, 0xFF);
}















