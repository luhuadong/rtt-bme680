/*
 * Copyright (c) 2020, RudyLo <luhuadong@163.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-07     luhuadong    the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "sensor_bme680.h"

#define BME680_I2C_BUS_NAME       PKG_USING_BME680_SAMPLE_I2C_BUS_NAME
#define BME680_I2C_ADDRESS        PKG_USING_BME680_SAMPLE_I2C_ADDRESS

static int rt_hw_bme680_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.type = RT_SENSOR_INTF_I2C;
    cfg.intf.dev_name = BME680_I2C_BUS_NAME;
    cfg.intf.user_data = (void *)BME680_I2C_ADDRESS;
    rt_hw_bme680_init("be6", &cfg);
    
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_bme680_port);