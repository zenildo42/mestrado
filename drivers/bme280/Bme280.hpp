/**
 * @file       Bme280.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdint.h>
#include <stdbool.h>

#include "../../platform/inc/I2c.hpp"
#include "bme280_defs.h"

struct Bme280Data
{
    float temperature;
    float humidity;
    float pressure;
};

class Bme280
{
public:
    Bme280(I2c& i2c, uint8_t address);
    bool init(void);
    bool read(Bme280Data* data);
public:
    int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* buffer, uint16_t length);
    int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* buffer, uint16_t length);
    void delay_ms(uint16_t ms);
private:
    I2c i2c_;
    uint8_t address_;

    struct bme280_dev dev;
};

#endif /* BME280_H_ */
