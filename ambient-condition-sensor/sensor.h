/*
 * SPDX-License-Identifier: GPL-3.0-only
 * Copyright 2020 Dominik Laton <dominik.laton@web.de>
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

namespace storage_unit_type {
    enum storage_unit_type;
}
class ambient_condition_sensor;

namespace sensor_type {
    enum sensor_type {
        BME_280 = 0,
        ADC,
        LAST
    };
}

namespace sensor_interface {
    enum sensor_interface {
        I2C_2_14 = 0,
        I2C_4_5,
        ADC,
        LAST
    };
}

class sensor {
private:
    enum sensor_interface::sensor_interface interface;
    enum sensor_type::sensor_type type;
    enum storage_unit_type::storage_unit_type storage_unit_temp;
    enum storage_unit_type::storage_unit_type storage_unit_hum;
    enum storage_unit_type::storage_unit_type storage_unit_pres;
    enum storage_unit_type::storage_unit_type storage_unit_volt;

    void prepare_interface();

    TwoWire i2c;
    Adafruit_BME280 bme;

protected:
    ambient_condition_sensor *context=nullptr;

public:
    sensor() = default;
    ~sensor() = default;

    void setup(enum sensor_interface::sensor_interface intf,
               enum sensor_type::sensor_type typ, ambient_condition_sensor *cont,
               enum storage_unit_type::storage_unit_type temp=storage_unit_type::TEMPERATURE_0,
               enum storage_unit_type::storage_unit_type hum=storage_unit_type::HUMIDITY_0,
               enum storage_unit_type::storage_unit_type pres=storage_unit_type::PRESSURE_0,
               enum storage_unit_type::storage_unit_type volt=storage_unit_type::VOLTAGE);
    void loop();
};

#endif
// vim: ts=4 sw=4 et cindent
