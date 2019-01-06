# GD32F107

## 简介

GD32F107为个人DIY开发板，板载资源主要如下：

| 硬件      | 描述          |
| --------- | ------------- |
| 芯片型号  | GD32F107RCT6  |
| CPU       | ARM Cortex M3 |
| 主频      | 108M          |
| 片内SRAM  | 96K          |
| 片内FLASH | 256K         |

## 编译说明

GD32F107板级包支持MDK5，以下是具体版本信息：

| IDE/编译器 | 已测试版本                   |
| ---------- | ---------------------------- |
| MDK4       | 未测试                      |
| MDK5       | MDK526                       |
| IAR        | 未测试             |
| GCC        | 未测试 |

## 烧写及执行

供电方式：开发板使用 Micro USB 接口电源。

下载程序：下载程序到开发板需要一套 JLink 或者使用 GD-Link 工具。

串口连接：使用USB线连接到Micro USB(UART0)(板载UART转USB芯片)。

### 运行结果

如果编译 & 烧写无误，当复位设备后，会在串口上看到RT-Thread的启动logo信息：

```bash
 \ | /
- RT -     Thread Operating System
 / | \     4.0.0 build Jan  6 2019
 2006 - 2018 Copyright by rt-thread team
msh >

```
## 驱动支持情况及计划

| 驱动      | 支持情况 |            备注            |
| --------- | -------- | :------------------------: |
| UART      | 支持     |          UART0~4(仅测试UART0,PB6,PB7)           |
| GPIO      | 支持   |              仅支持64脚芯片              |
| IIC       | 未支持   |                            |
| SPI       | 未支持     |                    |
| ETH       | 未支持     |                            |
| LCD       | 未支持     |  |
| SPI FLASH | 未支持     |                            |

### IO在板级支持包中的映射情况

| IO号 | 板级包中的定义 |
| ---- | -------------- |
| PC6  | LED1           |
| PC7  | LED2           |
| PC8  | LED3           |
| PC9  | LED4           |
| PA0  | KEY            |

## 联系人信息

维护人：[William](https://github.com/gw826943555)
本项目仅供个人学习使用。
