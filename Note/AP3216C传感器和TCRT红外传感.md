# AP3216C传感器和TCRT红外传感

## 一、实验一：AP3216C 三合一环境传感器

### 1. 传感器简介

AP3216C 是一款三合一环境传感器，内部集成：

| 模块 | 功能 |
| ---- | ---- |
| ALS（Ambient Light Sensor） | 数字环境光传感器，检测光照强度 |
| PS（Proximity Sensor） | 距离/接近传感器 |
| IR LED | 红外 LED |

芯片通过 **I2C（IIC）接口** 与 Hi3861 连接。

### 2. 实验功能

- 通过 `AP3216C_Init()` 初始化传感器，`AP3216C_ReadData(&ir, &als, &ps)` 循环读取三组数据；
- 数据同时输出到**串口**（printf）和 **SSD1306 OLED 屏**（`SSD1306_ShowStr`）；
- 实现"光控灯"：当光照强度 `als < 50` 时，GPIO6 输出高电平点亮 LED，否则熄灭。

### 3. 涉及的 GPIO 操作流程

```c
GpioInit();                                                        // 1. 初始化GPIO
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_6, WIFI_IOT_IO_FUNC_GPIO_6_GPIO);  // 2. 引脚复用为普通GPIO
GpioSetDir(WIFI_IOT_IO_NAME_GPIO_6, WIFI_IOT_GPIO_DIR_OUT);        // 3. 设置方向（输出）
GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_6, 0);                      // 4. 输出电平
```

## 二、实验二：TCRT5000 红外循迹传感器

### 1. 红外对管原理

- 红外循迹传感器由**发射器 + 接收器**组成，基于**红外线反射原理**工作；
- 照射到**白色/亮色**表面 → 反射强 → 接收器收到大信号（输出高电平）；
- 照射到**黑色/暗色**表面 → 反射弱 → 接收器收到小信号（输出低电平）；
- 应用：小车底部安装传感器检测地面黑线，实现自动循迹导航。

### 2. 硬件连接

- 左传感器 → GPIO13，右传感器 → GPIO14（同时也接 STM32 的 PA11/PA12，方便后续扩展）；
- 两个引脚均配置为**输入模式**，程序只需读取电平。

### 3. 软件定时器

**概念**：基于系统 Tick 时钟中断、由软件模拟的定时器，经过设定的 Tick 数后触发用户回调函数。硬件定时器数量有限，LiteOS-M 提供软件定时器扩展定时器数量。

**支持的功能**：

1. 静态裁切（可通过宏关闭）
2. 创建（`osTimerNew`）
3. 启动（`osTimerStart`）
4. 停止（`osTimerStop`）
5. 删除
6. 获取剩余 Tick 数

**运作机制**：

- 软件定时器使用系统的一个队列和一个任务资源，遵循队列规则，先进先出；
- 定时短的定时器靠近队列头，优先触发；
- Tick 中断到来时扫描全局链表记录超时定时器，中断结束后唤醒软件定时器任务（优先级最高），在该任务中统一执行回调函数。

**本实验中的用法**：

```c
id1 = osTimerNew(Timer1_Callback, osTimerPeriodic, &exec1, NULL);  // 创建周期定时器
status = osTimerStart(id1, timerDelay_1);                          // 启动，周期触发回调
```

> 注意：Hi3861 上 `1U = 10ms`，`timerDelay_1 = 5U` 实际是 50ms 触发一次（教程注释"10秒"为笔误）；若要 2 秒采集一次应设为 `200U`。

定时器回调中调用 `get_tcrt5000_value()`，用 `GpioGetInputVal()` 读取 GPIO13/14 电平，低电平打印 `left/right black`，高电平打印 `left/right white`。
