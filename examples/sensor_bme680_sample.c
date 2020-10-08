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


static void bme680_read_baro(void)
{
    rt_device_t sensor = RT_NULL;
    struct rt_sensor_data sensor_data;

    sensor = rt_device_find("baro_be6");
    if (!sensor) 
    {
        rt_kprintf("Can't find 'baro_be6' device.\n");
        return;
    }

    if (rt_device_open(sensor, RT_DEVICE_FLAG_RDWR)) 
    {
        rt_kprintf("Open 'baro_be6' device failed.\n");
        return;
    }

    if (1 != rt_device_read(sensor, 0, &sensor_data, 1)) 
    {
        rt_kprintf("Read baro value failed.\n");
    }
    else {
    	rt_kprintf("[%d] Baro: %d.%02d hPa\n", sensor_data.timestamp, sensor_data.data.baro / 100, sensor_data.data.baro % 100);
    }
    
    rt_device_close(sensor);
}

static void bme680_read_temp(void)
{
    rt_device_t sensor = RT_NULL;
    struct rt_sensor_data sensor_data;

    sensor = rt_device_find("temp_be6");
    if (!sensor) 
    {
        rt_kprintf("Can't find 'temp_be6' device.\n");
        return;
    }

    if (rt_device_open(sensor, RT_DEVICE_FLAG_RDWR)) 
    {
        rt_kprintf("Open 'temp_be6' device failed.\n");
        return;
    }

    if (1 != rt_device_read(sensor, 0, &sensor_data, 1)) 
    {
        rt_kprintf("Read temp value failed.\n");
    }
    else {
    	rt_kprintf("[%d] Temp: %d.%02d C\n", sensor_data.timestamp, sensor_data.data.temp / 100, sensor_data.data.temp % 100);
    }
    
    rt_device_close(sensor);
}

static void bme680_read_humi(void)
{
    rt_device_t sensor = RT_NULL;
    struct rt_sensor_data sensor_data;

    sensor = rt_device_find("humi_be6");
    if (!sensor) 
    {
        rt_kprintf("Can't find 'humi_be6' device.\n");
        return;
    }

    if (rt_device_open(sensor, RT_DEVICE_FLAG_RDWR)) 
    {
        rt_kprintf("Open 'humi_be6' device failed.\n");
        return;
    }

    if (1 != rt_device_read(sensor, 0, &sensor_data, 1)) 
    {
        rt_kprintf("Read humi value failed.\n");
    }
    else {
    	rt_kprintf("[%d] Humi: %d.%03d %\n", sensor_data.timestamp, sensor_data.data.humi / 1000, sensor_data.data.humi % 1000);
    }
    
    rt_device_close(sensor);
}
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(bme680_read_baro, read bme680 pressure value);
MSH_CMD_EXPORT(bme680_read_temp, read bme680 temperature value);
MSH_CMD_EXPORT(bme680_read_humi, read bme680 humidity value);
#endif


#ifdef PKG_USING_BME680_SAMPLE_I2C

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

#endif