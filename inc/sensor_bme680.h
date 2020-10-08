/*
 * Copyright (c) 2020, RudyLo <luhuadong@163.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-07     luhuadong    the first version
 */

#ifndef __SENSOR_BME680_H__
#define __SENSOR_BME680_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <sensor.h>
#include <board.h>
#include "bme680.h"

int rt_hw_bme680_init(const char *name, struct rt_sensor_config *cfg);

#endif /* __SENSOR_BME680_H__ */