/*
 * Copyright (c) 2020, RudyLo <luhuadong@163.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-07     luhuadong    the first version
 */

#include "sensor_bme680.h"

#define DBG_TAG                        "sensor.bosch.bme680"
#ifdef PKG_USING_BME680_DEBUG
#define DBG_LVL                        DBG_LOG
#else
#define DBG_LVL                        DBG_ERROR
#endif
#include <rtdbg.h>

/* range */
#define SENSOR_BARO_RANGE_MIN          ( 300*100)
#define SENSOR_BARO_RANGE_MAX          (1100*100)
#define SENSOR_TEMP_RANGE_MIN          ( -40*100)
#define SENSOR_TEMP_RANGE_MAX          (  85*100)
#define SENSOR_HUMI_RANGE_MIN          (   0*100)
#define SENSOR_HUMI_RANGE_MAX          ( 100*100)
#define SENSOR_IAQ_RANGE_MIN           (0)
#define SENSOR_IAQ_RANGE_MAX           (500)

/* minial period (ms) */
#define SENSOR_PERIOD_MIN              (1000)

/* fifo max length */
#define SENSOR_FIFO_MAX                (1)


static struct bme680_dev _bme680_dev;
static struct rt_i2c_bus_device *i2c_bus_dev;

static void rt_delay_ms(uint32_t period)
{
    rt_thread_mdelay(period);
}

//dev->write(dev->dev_id, tmp_buff[0], &tmp_buff[1], (2 * len) - 1);

static int8_t rt_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
#if 1
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */
/*
    rt_size_t ret = rt_i2c_transfer(i2c_bus_dev, msgs, 2);
    if (ret != 2)
    {
        LOG_E("rt_i2c_transfer() return %d", ret);
        return -RT_ERROR;
    }
    */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }
#else
    rt_i2c_master_send(i2c_bus_dev, addr, RT_I2C_WR, data, len);

#endif
    return RT_EOK;
}

static int8_t rt_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _bme680_init(struct rt_sensor_intf *intf)
{
    int8_t rslt = BME680_OK;

    _bme680_dev.dev_id   = (rt_uint32_t)(intf->user_data) & 0xff;
    _bme680_dev.intf     = BME680_I2C_INTF;
    _bme680_dev.read     = rt_i2c_read_reg;
    _bme680_dev.write    = rt_i2c_write_reg;
    _bme680_dev.delay_ms = rt_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor 
     * or by performing a few temperature readings without operating the gas sensor.
     */
    _bme680_dev.amb_temp = 25;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        LOG_E("can not find device %s", intf->dev_name);
        return -RT_ERROR;
    }

    rslt = bme680_init(&_bme680_dev);
    if (rslt == BME680_E_NULL_PTR)
    {
        LOG_E("the device structure pointer is null");
        return -RT_ERROR;
    }
    else if (rslt == BME680_E_DEV_NOT_FOUND)
    {
        LOG_E("can't found device");
        return -RT_ERROR;
    }
    else if (rslt != BME680_OK)
    {
        LOG_E("bme680 init failed");
        return -RT_ERROR;
    }

    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    _bme680_dev.tph_sett.os_hum  = BME680_OS_2X;
    _bme680_dev.tph_sett.os_pres = BME680_OS_4X;
    _bme680_dev.tph_sett.os_temp = BME680_OS_8X;
    _bme680_dev.tph_sett.filter  = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    _bme680_dev.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    _bme680_dev.gas_sett.heatr_temp = 320; /* degree Celsius */
    _bme680_dev.gas_sett.heatr_dur  = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    _bme680_dev.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings, &_bme680_dev);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&_bme680_dev);

    return rslt;
}

static rt_err_t _bme680_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    int8_t rslt = 0;

    if (power == RT_SENSOR_POWER_DOWN)
    {
        _bme680_dev.power_mode = BME680_SLEEP_MODE;
        rslt = bme680_set_sensor_mode(&_bme680_dev);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        _bme680_dev.power_mode = BME680_FORCED_MODE;
        rslt = bme680_set_sensor_mode(&_bme680_dev);
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
    return rslt;
}

static rt_size_t _bme680_polling_get_data(struct rt_sensor_device *sensor, void *buf)
{
    struct bme680_field_data data;
    struct rt_sensor_data *sensor_data = buf;
    //struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->config.intf.user_data;

    if (BME680_OK != bme680_get_sensor_data(&data, &_bme680_dev))
    {
        LOG_E("Can not read from %s", sensor->info.model);
        return 0;
    }

    LOG_D("temp: %d, baro: %d, humi: %d, gas: %d", data.temperature, data.pressure, data.humidity, data.gas_resistance);

    rt_uint32_t timestamp = rt_sensor_get_ts();

    if (sensor->info.type == RT_SENSOR_CLASS_BARO)
    {
        sensor_data->type = RT_SENSOR_CLASS_BARO;
        sensor_data->data.baro = data.pressure;
        sensor_data->timestamp = timestamp;

        struct rt_sensor_data *temp_data = (struct rt_sensor_data *)sensor->module->sen[1]->data_buf;
        if (temp_data)
        {
            temp_data->type = RT_SENSOR_CLASS_TEMP;
            temp_data->data.temp = data.temperature;
            temp_data->timestamp = timestamp;
            sensor->module->sen[1]->data_len = sizeof(struct rt_sensor_data);
        }

        struct rt_sensor_data *humi_data = (struct rt_sensor_data *)sensor->module->sen[2]->data_buf;
        if (humi_data)
        {
            humi_data->type = RT_SENSOR_CLASS_HUMI;
            humi_data->data.humi = data.humidity;
            humi_data->timestamp = timestamp;
            sensor->module->sen[2]->data_len = sizeof(struct rt_sensor_data);
        }
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        sensor_data->type = RT_SENSOR_CLASS_TEMP;
        sensor_data->data.temp = data.temperature;
        sensor_data->timestamp = timestamp;

        struct rt_sensor_data *baro_data = (struct rt_sensor_data *)sensor->module->sen[0]->data_buf;
        if (baro_data)
        {
            baro_data->type = RT_SENSOR_CLASS_BARO;
            baro_data->data.baro = data.pressure;
            baro_data->timestamp = timestamp;
            sensor->module->sen[0]->data_len = sizeof(struct rt_sensor_data);
        }

        struct rt_sensor_data *humi_data = (struct rt_sensor_data *)sensor->module->sen[2]->data_buf;
        if (humi_data)
        {
            humi_data->type = RT_SENSOR_CLASS_HUMI;
            humi_data->data.humi = data.humidity;
            humi_data->timestamp = timestamp;
            sensor->module->sen[2]->data_len = sizeof(struct rt_sensor_data);
        }
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_HUMI)
    {
        sensor_data->type = RT_SENSOR_CLASS_HUMI;
        sensor_data->data.humi = data.humidity;
        sensor_data->timestamp = timestamp;

        struct rt_sensor_data *baro_data = (struct rt_sensor_data *)sensor->module->sen[0]->data_buf;
        if (baro_data)
        {
            baro_data->type = RT_SENSOR_CLASS_BARO;
            baro_data->data.baro = data.pressure;
            baro_data->timestamp = timestamp;
            sensor->module->sen[0]->data_len = sizeof(struct rt_sensor_data);
        }

        struct rt_sensor_data *temp_data = (struct rt_sensor_data *)sensor->module->sen[1]->data_buf;
        if (temp_data)
        {
            temp_data->type = RT_SENSOR_CLASS_TEMP;
            temp_data->data.temp = data.temperature;
            temp_data->timestamp = timestamp;
            sensor->module->sen[1]->data_len = sizeof(struct rt_sensor_data);
        }
    }

    return 1;
}

static rt_size_t bme680_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _bme680_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t bme680_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = BME680_CHIP_ID; /* _bme680_dev.chip_id */
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        return -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _bme680_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        /* TODO */
        result = -RT_EINVAL;
        break;
    default:
        return -RT_EINVAL;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    bme680_fetch_data,
    bme680_control
};

int rt_hw_bme680_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_baro = RT_NULL;
    rt_sensor_t sensor_temp = RT_NULL;
    rt_sensor_t sensor_humi = RT_NULL;
    struct rt_sensor_module *module = RT_NULL;

    if (_bme680_init(&cfg->intf) != RT_EOK)
    {
        return RT_ERROR;
    }
    
    module = rt_calloc(1, sizeof(struct rt_sensor_module));
    if (module == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    /*  barometric pressure sensor register */
    {
        sensor_baro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_baro == RT_NULL)
            goto __exit;

        sensor_baro->info.type       = RT_SENSOR_CLASS_BARO;
        sensor_baro->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_baro->info.model      = "bme680";
        sensor_baro->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_baro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = SENSOR_BARO_RANGE_MAX;
        sensor_baro->info.range_min  = SENSOR_BARO_RANGE_MIN;
        sensor_baro->info.period_min = SENSOR_PERIOD_MIN;
        sensor_baro->info.fifo_max   = SENSOR_FIFO_MAX;
        sensor_baro->data_len        = 0;

        rt_memcpy(&sensor_baro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_baro->ops = &sensor_ops;
        sensor_baro->module = module;
        
        result = rt_hw_sensor_register(sensor_baro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
    /* temperature sensor register */
    {
        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)
            goto __exit;

        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = "bme680";
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_temp->info.range_max  = SENSOR_TEMP_RANGE_MAX;
        sensor_temp->info.range_min  = SENSOR_TEMP_RANGE_MIN;
        sensor_temp->info.period_min = SENSOR_PERIOD_MIN;
        sensor_temp->info.fifo_max   = SENSOR_FIFO_MAX;
        sensor_temp->data_len        = 0;

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &sensor_ops;
        sensor_temp->module = module;
        
        result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
    /* humidity sensor register */
    {
        sensor_humi = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_humi == RT_NULL)
            goto __exit;

        sensor_humi->info.type       = RT_SENSOR_CLASS_HUMI;
        sensor_humi->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_humi->info.model      = "bme680";
        sensor_humi->info.unit       = RT_SENSOR_UNIT_PERMILLAGE;
        sensor_humi->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_humi->info.range_max  = SENSOR_HUMI_RANGE_MAX;
        sensor_humi->info.range_min  = SENSOR_HUMI_RANGE_MIN;
        sensor_humi->info.period_min = SENSOR_PERIOD_MIN;
        sensor_humi->info.fifo_max   = SENSOR_FIFO_MAX;
        sensor_humi->data_len        = 0;

        rt_memcpy(&sensor_humi->config, cfg, sizeof(struct rt_sensor_config));
        sensor_humi->ops = &sensor_ops;
        sensor_humi->module = module;
        
        result = rt_hw_sensor_register(sensor_humi, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    module->sen[0] = sensor_baro;
    module->sen[1] = sensor_temp;
    module->sen[2] = sensor_humi;
    module->sen_num = 3;

    LOG_I("sensor init success");
    return RT_EOK;
    
__exit:
    if(sensor_baro)
        rt_free(sensor_baro);
    if(sensor_temp)
        rt_free(sensor_temp);
    if(sensor_humi)
        rt_free(sensor_humi);
    if (module)
        rt_free(module);
    return -RT_ERROR;
}
