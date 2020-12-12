/*
 * SPDX-License-Identifier: GPL-3.0-only
 * Copyright 2020 Dominik Laton <dominik.laton@web.de>
 */
#include "ambient-condition-sensor.h"
#include "rtc_storage.h"
#include "sensor.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

void sensor::setup(enum sensor_interface::sensor_interface intf,
        enum sensor_type::sensor_type typ, ambient_condition_sensor *cont,
        enum storage_unit_type::storage_unit_type temp,
        enum storage_unit_type::storage_unit_type hum,
        enum storage_unit_type::storage_unit_type pres)
{
    interface = intf;
    type = typ;
    context = cont;
    storage_unit_temp = temp;
    storage_unit_hum = hum;
    storage_unit_pres = pres;
}

void sensor::prepare_interface()
{
    switch(interface) {
        case sensor_interface::I2C_2_14:
            {
                i2c.begin(2,14);
                break;
            }
        case sensor_interface::I2C_4_5:
            {
                i2c.begin(4,5);
                break;
            }
            {
                break;
            }
        default:
            break;
    }
}

void sensor::loop()
{
    prepare_interface();

    switch(type) {
        case sensor_type::BME_280:
            {
                Adafruit_BME280 bme;
                bme.begin(0x76, &i2c);
                delay(1000);

                context->get_storage().update_value(storage_unit_temp, bme.readTemperature(), 0.1);
                context->get_storage().update_value(storage_unit_pres,
                        (bme.readPressure()/100.0F), 0.5);
                context->get_storage().update_value(storage_unit_hum, bme.readHumidity(), 1.0);
                break;
            }
        default:
            break;
    }
}
// vim: ts=4 sw=4 et cindent
