#ifndef INA226_API_H_
#define INA226_API_H_

#include <stdint.h>
#include "ina226_port.h"

#define INA226_MANUF_ID     (0x5449UL)  //Texas Instruments
#define SHUNT_VOLTAGE_LSB   (2.5f)    // 2.5uV
#define BUS_VOLTAGE_LSB     (1.25f)   // 1.25mV


#ifdef __CMSIS_COMPILER_H
#define INA226_LEU16(x) __REV16(x)
#elif defined(__GNUC__)
#define INA226_LEU16(x) __builtin_bswap16(x)
#else
static __inline uint16_t ina226_swap_u16(uint16_t x)
{
    return (x << 8) | (x >> 8);
}
#define INA226_LEU16(x) ina226_swap_u16(x)
#endif


typedef enum
{
    INA226_OK = 0,
    INA226_ERR_INVALID_PARAM = 1,
    INA226_ERR_I2C = 2,
    INA226_ERR_CALIB = 3,
    INA226_ERR_MANUF_ID = 4,
} ina226_err_t;

typedef enum
{
    INA226_ALERT_SOL = 0, //Shunt over-limit  分流电压过限
    INA226_ALERT_SUL = 1, //Shunt under-limit 分流电压欠限
    INA226_ALERT_BOL = 2, //Bus over-limit    总线电压过限
    INA226_ALERT_BUL = 3, //Bus under-limit   总线电压欠限
    INA226_ALERT_POL = 4, //Power over-limit  功率过限
} ina226_alert_func_t;

typedef enum
{
    INA226_REG_CONFIG = 0x00, //R/W Configuration register
    INA226_REG_SHUNT_VOLTS = 0x01, //R   Shunt voltage register
    INA226_REG_BUS_VOLTS = 0x02, //R   Bus voltage register
    INA226_REG_POWER = 0x03, //R   Power register
    INA226_REG_CURRENT = 0x04, //R   Current register
    INA226_REG_CALIB = 0x05, //R/W Calibration register
    INA226_REG_MASK = 0x06, //R/W Mask/enable register
    INA226_REG_ALERT = 0x07, //R   Alert register
    INA226_REG_MANUF_ID = 0xFE, //R   Manufacturer ID register
    INA226_REG_DIE_ID = 0xFF, //R   Die ID register
    INA226_REG_MAX = 10,
} ina226_regs_index;

typedef struct
{
    uint16_t config; //R/W Configuration register
    uint16_t shunt_volts; //R   Shunt voltage register
    uint16_t bus_volts; //R   Bus voltage register
    uint16_t power; //R   Power register
    uint16_t current; //R   Current register
    uint16_t calib; //R/W Calibration register
    uint16_t mask; //R/W Mask/enable register
    uint16_t alert; //R   Alert register
    uint16_t manuf_id; //R   Manufacturer ID register
    uint16_t die_id; //R   Die ID register
} ina226_regs_t;

typedef enum
{
    INA226_MODE_CONTINUOUS = 0,
    INA226_MODE_TRIGGERED = 1,
    INA226_MODE_POWER_DOWN = 2,
} ina226_mode_t;

typedef enum
{
    INA226_ADDR_PIN_GND = 0,
    INA226_ADDR_PIN_VS = 1,
    INA226_ADDR_PIN_SDA = 2,
    INA226_ADDR_PIN_SCL = 3,
} ina226_addr_pin_t;

typedef struct ina226_handle_t ina226_handle_t;

struct ina226_handle_t
{
    ina226_i2c_port_t i2c;
    ina226_addr_pin_t A0;
    ina226_addr_pin_t A1;
    ina226_alert_port_t alert; //Multi-functional alert, open-drain output.
    uint8_t dev_addr;
    float res_shunt_ohm;
    float current_lsb_Ampere;
    float max_expected_current_Ampere;
    ina226_mode_t mode;
    ina226_regs_t regs;
    ina226_err_t (*i2c_write_reg)(ina226_i2c_port_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data, uint8_t size);
    ina226_err_t (*i2c_read_reg)(ina226_i2c_port_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t* reg_data, uint8_t size);
};

ina226_err_t ina226_port_i2c_write_reg(ina226_i2c_port_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data, uint8_t size);
ina226_err_t ina226_port_i2c_read_reg(ina226_i2c_port_t i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t* reg_data, uint8_t size);

typedef enum
{
    INA226_RST_BIT_POS = 15U,
    INA226_RST_BIT_NORMAL = 0UL << INA226_RST_BIT_POS, /*!< 正常操作 -default*/
    INA226_RST_BIT_RESET = 1UL << INA226_RST_BIT_POS, /*!< 复位操作*/
} ina226_rst_bit_t;

typedef enum
{
    INA226_AVG_BIT_POS  = 9U,
    INA226_AVG_BIT_1    = 0UL << INA226_AVG_BIT_POS, /*!< 1次平均 -default*/
    INA226_AVG_BIT_4    = 1UL << INA226_AVG_BIT_POS, /*!< 4次平均*/
    INA226_AVG_BIT_16   = 2UL << INA226_AVG_BIT_POS, /*!< 16次平均*/
    INA226_AVG_BIT_64   = 3UL << INA226_AVG_BIT_POS, /*!< 64次平均*/
    INA226_AVG_BIT_128  = 4UL << INA226_AVG_BIT_POS, /*!< 128次平均*/
    INA226_AVG_BIT_256  = 5UL << INA226_AVG_BIT_POS, /*!< 256次平均*/
    INA226_AVG_BIT_512  = 6UL << INA226_AVG_BIT_POS, /*!< 512次平均*/
    INA226_AVG_BIT_1024 = 7UL << INA226_AVG_BIT_POS, /*!< 1024次平均*/
    INA226_AVG_BIT_MASK = INA226_AVG_BIT_1024,
} ina226_avg_bit_t;

ina226_err_t ina226_init(ina226_handle_t* handle);
ina226_err_t ina226_get_bus_voltage(ina226_handle_t *handle, float *bus_volts_mV);
ina226_err_t ina226_get_current(ina226_handle_t *handle, float *current_Ampere);
ina226_err_t ina226_get_power(ina226_handle_t *handle, float *power_mW);
ina226_err_t ina226_set_average_times(ina226_handle_t *handle, ina226_avg_bit_t average_times);
ina226_err_t ina226_reset(ina226_handle_t *handle);
ina226_err_t ina226_get_die_id(ina226_handle_t *handle, uint16_t *die_id);

#endif
