#include "ina226_api.h"

static ina226_err_t ina226_write_reg(ina226_handle_t* handle, uint8_t reg_addr, uint16_t reg_data, uint8_t size)
{
    ina226_err_t err = INA226_OK;

    err = handle->i2c_write_reg(handle->i2c, handle->dev_addr, reg_addr, INA226_LEU16(reg_data), size);
    if (err != INA226_OK)
    {
        return err;
    }

    return err;
}

static ina226_err_t ina226_read_reg(ina226_handle_t* handle, uint8_t reg_addr, uint16_t *reg_data, uint8_t size)
{
    ina226_err_t err = INA226_OK;

    err = handle->i2c_read_reg(handle->i2c, handle->dev_addr, reg_addr, reg_data, size);
    if (err != INA226_OK)
    {
        return err;
    }
    *reg_data = INA226_LEU16(*reg_data);

    return err;
}

ina226_err_t ina226_set_average_times(ina226_handle_t *handle, const ina226_avg_bit_t average_times)
{
    if (handle == NULL || average_times == 0 || average_times > INA226_AVG_BIT_1024)
    {
        return INA226_ERR_INVALID_PARAM;
    }


    ina226_err_t err = ina226_read_reg(handle, INA226_REG_CONFIG, &handle->regs.config,2);
    if (err != INA226_OK)
    {
        return err;
    }

    // 清除旧的AVG位
    handle->regs.config &= ~INA226_AVG_BIT_MASK;

    // 设置新的AVG位
    handle->regs.config |= average_times;

    // 写入Config Register
    err = ina226_write_reg(handle, INA226_REG_CONFIG, handle->regs.config,2);
    if (err != INA226_OK)
    {
        return err;
    }

    return INA226_OK;
}

ina226_err_t ina226_reset(ina226_handle_t *handle)
{
    if (handle == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    //获取当前配置
    ina226_err_t err = ina226_read_reg(handle, INA226_REG_CONFIG, &handle->regs.config,2);
    if (err != INA226_OK)
    {
        return err;
    }

    handle->regs.config |= INA226_RST_BIT_RESET;
    // 写入Reset Register
     err = ina226_write_reg(handle, INA226_REG_CONFIG, handle->regs.config,2);
     if (err != INA226_OK)
     {
         return err;
     }

    return INA226_OK;
}

static ina226_err_t ina226_set_calibration_reg(ina226_handle_t *handle)
{
    ina226_err_t err = INA226_OK;
    if (handle->current_lsb_Ampere == 0.0f || handle->res_shunt_ohm == 0.0f)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    //写入Calibration Register
    const uint16_t cal = (uint16_t)(0.00512f / handle->current_lsb_Ampere / handle->res_shunt_ohm);
    err = ina226_write_reg(handle, INA226_REG_CALIB, cal,2);
    if (err != INA226_OK)
    {
        return err;
    }

    err = ina226_read_reg(handle, INA226_REG_CALIB, &handle->regs.calib,2);
    if (err != INA226_OK)
    {
        return err;
    }

    if (handle->regs.calib != cal)
    {
        return INA226_ERR_CALIB;
    }

    return err;
}

ina226_err_t ina226_get_die_id(ina226_handle_t *handle, uint16_t *die_id)
{
    if (handle == NULL || die_id == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }
    ina226_err_t err = INA226_OK;

    // 读取Die ID Register
    err = ina226_read_reg(handle, INA226_REG_DIE_ID, die_id,2);
    if (err != INA226_OK)
    {
        return err;
    }

    return INA226_OK;
}

ina226_err_t ina226_get_shunt_volts(ina226_handle_t *handle, float *shunt_volts_uV)
{
    ina226_err_t err = INA226_OK;
    if (handle == NULL || shunt_volts_uV == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    // 读取Shunt Voltage Register
     err = ina226_read_reg(handle, INA226_REG_SHUNT_VOLTS, &handle->regs.shunt_volts,2);
     if (err != INA226_OK)
     {
         return err;
     }

    //计算电压,单位:uV
    *shunt_volts_uV = (float)handle->regs.shunt_volts * SHUNT_VOLTAGE_LSB;

    return err;
}

ina226_err_t ina226_get_current(ina226_handle_t *handle, float *current_mA)
{
    ina226_err_t err = INA226_OK;
    if (handle == NULL || current_mA == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    //读取Current Register
    err = ina226_read_reg(handle, INA226_REG_CURRENT, &handle->regs.current,2);
    if (err != INA226_OK)
    {
        return err;
    }

    //计算电流,单位:mA
    *current_mA = (float)handle->regs.current * handle->current_lsb_Ampere * 1e3f;

    return err;
}

ina226_err_t ina226_get_bus_voltage(ina226_handle_t *handle, float *bus_volts_mV)
{
    ina226_err_t err = INA226_OK;
    if (handle == NULL || bus_volts_mV == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    //读取Bus Voltage Register
    err = ina226_read_reg(handle, INA226_REG_BUS_VOLTS, &handle->regs.bus_volts,2);
    if (err != INA226_OK)
    {
        return err;
    }

    //计算电压,单位:mV
    *bus_volts_mV = (float)handle->regs.bus_volts * BUS_VOLTAGE_LSB;

    return err;
}

ina226_err_t ina226_get_power(ina226_handle_t *handle, float *power_mW)
{
    ina226_err_t err = INA226_OK;
    if (handle == NULL || power_mW == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    //读取Power Register
    err = ina226_read_reg(handle, INA226_REG_POWER, &handle->regs.power,2);
    if (err != INA226_OK)
    {
    return err;
    }

    //计算功率
    *power_mW = (float)handle->regs.power * handle->current_lsb_Ampere * 25.0f * 1000.0f;

    return err;
}

float ina226_calculate_current_lsb_raw(float max_expected_current)
{
    return max_expected_current / 32768.0f;
}

static ina226_err_t ina226_get_addr(ina226_handle_t *handle)
{
    if (handle->A0 >= INA226_ADDR_PIN_GND && handle->A0 <= INA226_ADDR_PIN_SCL &&
        handle->A1 >= INA226_ADDR_PIN_GND && handle->A1 <= INA226_ADDR_PIN_SCL)
    {
        const uint8_t base_addr = 0b01000000;
        handle->dev_addr = (base_addr | handle->A0 | handle->A1*4) << 1;
        return INA226_OK;
    }

    return INA226_ERR_INVALID_PARAM;
}

__attribute__((weak)) ina226_err_t ina226_init(ina226_handle_t *handle)
{
    ina226_err_t err = INA226_OK;

    if (handle == NULL)
    {
        return INA226_ERR_INVALID_PARAM;
    }

    handle->i2c_write_reg = ina226_port_i2c_write_reg;
    handle->i2c_read_reg = ina226_port_i2c_read_reg;

    //根据A0 A1引脚值获取I2C地址
    err = ina226_get_addr(handle);
    if (err != INA226_OK)
    {
        return err;
    }

    //读取Manufacturer ID Register 确认设备是否存在
    err = ina226_read_reg(handle, INA226_REG_MANUF_ID, &handle->regs.manuf_id,2);
    if (err != INA226_OK)
    {
        return err;
    }
    if (handle->regs.manuf_id != INA226_MANUF_ID)
    {
        return INA226_ERR_MANUF_ID;
    }

    //读取Config Register默认值
    err = ina226_read_reg(handle, INA226_REG_CONFIG, &handle->regs.config,2);
    if (err != INA226_OK)
    {
        return err;
    }

    //设置校准寄存器
    err = ina226_set_calibration_reg(handle);
    if (err != INA226_OK)
    {
        return err;
    }


    return INA226_OK;
}



