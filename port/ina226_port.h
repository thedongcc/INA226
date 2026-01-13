#ifndef INA226_PORT_H_
#define INA226_PORT_H_


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
