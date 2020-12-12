/*
 * SPDX-License-Identifier: GPL-3.0-only
 * Copyright 2020 Dominik Laton <dominik.laton@web.de>
 */
#include "ambient-condition-sensor.h"
#include "rtc_storage.h"
#include "sensor.h"

void sensor::setup(enum sensor_interface::sensor_interface intf,
        enum sensor_type::sensor_type typ, ambient_condition_sensor *cont,
        enum storage_unit_type::storage_unit_type temp,
        enum storage_unit_type::storage_unit_type hum,
        enum storage_unit_type::storage_unit_type pres,
        enum storage_unit_type::storage_unit_type volt)
{
    interface = intf;
    type = typ;
    context = cont;
    storage_unit_temp = temp;
    storage_unit_hum = hum;
    storage_unit_pres = pres;
    storage_unit_volt = volt;

        switch(type) {
            case sensor_type::BME_280:
                {
                    prepare_interface();
                    bme.begin(0x76, &i2c);
                    bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X1,
                            Adafruit_BME280::SAMPLING_X1, Adafruit_BME280::SAMPLING_X1,
                            Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_1000);
                    bme.takeForcedMeasurement();
                    break;
                }
            default:
                break;
        }
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
        case sensor_interface::ADC:
            {
                break;
            }
        default:
            break;
    }
}

void sensor::loop()
{
    switch(type) {
        case sensor_type::BME_280:
            {
                context->get_storage().update_value(storage_unit_temp, bme.readTemperature(), 0.1);
                context->get_storage().update_value(storage_unit_pres,
                        (bme.readPressure()/100.0F), 0.5);
                context->get_storage().update_value(storage_unit_hum, bme.readHumidity(), 0.7);
                break;
            }
        case sensor_type::ADC:
            {
                float voltage;
                // NOTE: R1=47k R2=9k1 => 6.16V max voltage => 1024 ADC steps => 0.00602 V/step
                voltage = float(analogRead(0)) * 0.00602;
                context->get_storage().update_value(storage_unit_volt, voltage, 0.01);
                break;
            }
        default:
            break;
    }
}
// vim: ts=4 sw=4 et cindent
