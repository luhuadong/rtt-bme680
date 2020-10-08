# rtt-bme680

BME680 sensor package for RT-Thread



## 1、介绍

BME680 环境传感器是一款四合一 MEMS 环境传感器，可测量 VOC（挥发性有机物）、温度、湿度、气压这四个参数，非常适用于监测空气质量。由于采用了 MEMS 技术，该传感器体积小、功耗低，因此也适用于低功耗场合，如可穿戴等。BME680 同时支持 I2C（地址可配置为 0x76 或 0x77）和 SPI 接口。

测量范围及精度：

- 温度测量范围：-40℃~+85℃
- 温度测量精度：±1.0℃(0~65℃)
- 湿度测量范围：0-100%r.H.
- 湿度测量精度：±3%r.H.(20-80% r.H.,25℃)
- 气压测量范围：300-1100hPa
- 气压测量精度：±0.6hPa(300-1100hPa,0~65℃)
- IAQ（室内空气质量）范围：0-500（值越大，空气质量越差）



### 1.1 特性

- 支持 sensor 设备驱动框架。
- 支持 I2C 和 SPI 接口。
- 线程安全。



### 1.2 工作模式

|    传感器    | 气压 | 温度 | 湿度 | IAQ  |
| :----------: | :--: | :--: | :--: | :--: |
| **通信接口** |      |      |      |      |
|     I2C      |  √   |  √   |  √   |  √   |
|     SPI      |      |      |      |      |
| **工作模式** |      |      |      |      |
|     轮询     |  √   |  √   |  √   |  √   |
|     中断     |      |      |      |      |
|     FIFO     |      |      |      |      |



### 1.3 目录结构

| 名称          | 说明                                                         |
| ------------- | ------------------------------------------------------------ |
| docs          | 文档目录                                                     |
| examples      | 例子目录                                                     |
| inc           | 头文件目录                                                   |
| src           | 源代码目录（对接 RT-Thread Sensor 接口）                     |
| BME680_driver | [官方驱动库](https://github.com/BoschSensortec/BME680_drive)（略微改动） |





### 1.4 许可证

- bme680 软件包遵循 Apache license v2.0 许可，详见 `LICENSE` 文件。
- BME680_driver 库遵循 BSD-3-Clause 许可，详见 `BME680_driver/LICENSE` 文件。



### 1.5 依赖

- RT-Thread 4.0+
- 使用 sensor 设备接口需要开启 sensor 设备驱动框架模块
- 需要完成 BSP 的 I2C 或 SPI 配置



## 2、获取 bme680 软件包

使用 bme680 package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages --->
    peripheral libraries and drivers --->
        [*] sensors drivers  --->
            [*] BME680: Digital 4-in-1 sensor with gas, humidity, pressure and temperature.
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。



## 3、使用 bme680 软件包

### 3.1 版本说明

| 版本   | 说明                                                 |
| ------ | ---------------------------------------------------- |
| latest | 暂时只支持 I2C 接口（SPI 接口及 IAQ 测量功能待更新） |

目前处于公测阶段，建议开发者使用 latest 版本。



### 3.2 配置选项

- 是否开启浮点补偿
- 是否开启 LOG 调试信息输出
- 是否开启 I2C 示例程序
  - 设置设备挂载的 I2C 总线名称
  - 选择 I2C 地址（0x76 或 0x77）



## 4、API 说明

bme680 软件包已对接 sensor 驱动框架，操作传感器模块之前，只需调用下面接口注册传感器设备即可。

```c
rt_err_t rt_hw_bme680_init(const char *name, struct rt_sensor_config *cfg);
```

| 参数      | 描述            |
| --------- | --------------- |
| name      | 传感器设备名称  |
| cfg       | sensor 配置信息 |
| **返回**  | ——              |
| RT_EOK    | 创建成功        |
| -RT_ERROR | 创建失败        |



#### 初始化示例

```c
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bme680.h"

#define BME680_I2C_BUS_NAME       "i2c1"
#define BME680_I2C_ADDRESS        0x77

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
```



#### 传感器测试

将上述 sensor 初始化示例代码加入工程，编译下载后即可进行测试。（注意：需要先配置好 i2c1 总线，并添加 sensor 组件）

**检查传感器是否初始化成功**

```shell
msh >list_device
device           type         ref count
-------- -------------------- ----------
humi_be6 Sensor Device        0
temp_be6 Sensor Device        0
baro_be6 Sensor Device        0
```

**查看 BME680 信息**

```shell
msh >sensor probe baro_be6
[I/sensor.cmd] device id: 0x61!

msh >sensor info
vendor    :Bosch
model     :bme680
unit      :pa
range_max :110000
range_min :30000
period_min:1000ms
fifo_max  :1
```

**使用 sensor 命令读取气压数据**

```shell
msh >sensor read 1
[I/sensor.cmd] num:  0, press:100120 pa, timestamp:1764982
```

**使用示例程序读取气压数据**

```shell
msh >bme680_read_baro
[1817747] Baro: 1001.26 hPa
```



## 5、注意事项

暂无。



## 6、相关文档

见 docs 目录。



## 7、联系方式

- 维护：luhuadong@163.com
- 主页：<https://github.com/luhuadong/rtt-bme680>
