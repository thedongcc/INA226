#ifndef __INA226_EXAMPLE_PORT_H__
#define __INA226_EXAMPLE_PORT_H__




#include <stdint.h>
#include "i2c.h"



typedef struct
{
    I2C_HandleTypeDef *handle;
}ina226_i2c_port_t;

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
}ina226_alert_port_t;













#endif
