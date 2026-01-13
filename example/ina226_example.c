#include "ina226_api.h"

ina226_handle_t ina226_device;

void ina226_example_init(void)
{
    ina226_device.i2c.handle = &hi2c1;
    ina226_device.A0 = INA226_ADDR_PIN_GND;
    ina226_device.A1 = INA226_ADDR_PIN_GND;
    ina226_device.current_lsb_Ampere = 0.001f;
    ina226_device.res_shunt_ohm = 0.01f;


    ina226_err_t err = ina226_init(&ina226_device);
    if (err != INA226_OK)
    {
        printf("INA226 init error: %d\r\n", err);
        return;
    }

    err = ina226_set_average_times(&ina226_device, INA226_AVG_BIT_16);
    if (err != INA226_OK)
    {
        printf("INA226 set average times error: %d\r\n", err);
        return;
    }

}












