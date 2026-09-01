#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"

/************** 引脚定义 **************/
#define GPIO_IR_L   13    // 左红外传感器（桌沿检测）
#define GPIO_IR_R   14    // 右红外传感器（桌沿检测）

/************** 参数配置 **************/
#define TURN_TIME     1000000  // 危险时转向时长（us），需实测校准

uint8_t uart_sendbuf[20];

/***通信协议***/
/*
函数功能 : 发送至stm32的数据协议
参数     : 电机实际转速的一百倍，例如: 设置转速为1rad/s，则传入100
*/
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    // 小车运动方向 前进（正转）: 0   后退（反转） 1
    if (motorA < 0) {
        A_dir = 1;
        motorA = -motorA;
    } else {
        A_dir = 0;
    }
    if (motorB < 0) {
        B_dir = 1;
        motorB = -motorB;
    } else {
        B_dir = 0;
    }

    // 限制幅度 -150 ~150
    if (motorA > 150)
    {
        motorA = 150;
    }
    if (motorB > 150)
    {
        motorB = 150;
    }

    // 数据协议
    uart_sendbuf[0] = 0xFC;   // 帧头
    uart_sendbuf[1] = A_dir;  // 左轮方向  0正转，1反转
    uart_sendbuf[2] = motorA; // 左轮速度
    uart_sendbuf[3] = B_dir;  // 右轮方向  0正转，1反转
    uart_sendbuf[4] = motorB; // 右轮速度
    uart_sendbuf[5] = 0xFD;   // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-80, -80);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(30, 30);
}

// 小车左转（原地旋转）
void car_left(void)
{
    stm32motor_control(-80, 80);
}

// 小车右转（原地旋转）
void car_right(void)
{
    stm32motor_control(80, -80);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}

/************** 桌沿检测 **************/
/*
本车 TCRT5000 实测极性：悬空/过桌沿 = VALUE1（危险），贴桌面 = VALUE0（安全）。
加权滤波：读到危险 +2，读到安全 -1，净累计 4 次才确认。
- 缝隙/深色纹路：只有一两次危险读数，很快被安全读数衰减掉 → 不触发
- 真桌沿：危险读数占多数，即使传感器骑在边界抖动也能累计到阈值 → 触发
返回值 : 1 = 检测到桌沿（危险），0 = 正常
*/
int EdgeDetected(WifiIotGpioValue *left, WifiIotGpioValue *right)
{
    static uint8_t edge_cnt = 0;

    GpioGetInputVal(GPIO_IR_L, left);
    GpioGetInputVal(GPIO_IR_R, right);

    if (*left == WIFI_IOT_GPIO_VALUE1 || *right == WIFI_IOT_GPIO_VALUE1) {
        edge_cnt += 2;
        if (edge_cnt > 10) edge_cnt = 10;
    } else {
        if (edge_cnt > 0) edge_cnt--;
    }

    if (edge_cnt >= 2) {     // 净累计4次（约需2~4个危险读数）才确认
        edge_cnt = 0;
        return 1;
    }
    return 0;
}

/************** 防跌落主任务 **************/
static void avoid_task(void)
{
    WifiIotGpioValue ir_left, ir_right;

    car_stop();           // 上电先停住
    sleep(2);             // 等 STM32 端初始化完成

    while (1)
    {
        /* 桌沿检测 */
                if (EdgeDetected(&ir_left, &ir_right))
        {
            car_backward();
            usleep(1000000);          // ★ 先固定退0.4秒，保证最小后退距离（可调）

            int safe_cnt = 0;
            for (int i = 0; i < 40; i++) {          // 再检查：没安全就继续退
                WifiIotGpioValue l, r;
                GpioGetInputVal(GPIO_IR_L, &l);
                GpioGetInputVal(GPIO_IR_R, &r);
                if (l == WIFI_IOT_GPIO_VALUE0 && r == WIFI_IOT_GPIO_VALUE0) {
                    if (++safe_cnt >= 2) break;
                } else {
                    safe_cnt = 0;
                }
                usleep(50000);
            }

            // 哪边悬空就往反方向转
            if (ir_left == WIFI_IOT_GPIO_VALUE1) {
                car_right();
            } else {
                car_left();
            }
            usleep(TURN_TIME);
            continue;
        }

        /* 一切安全 → 前进 */
        car_forward();
        usleep(20000);              // 每 20ms 巡检一次
    }
}

/************** 初始化与任务创建 **************/
static void correspondence(void)
{
    WatchDogDisable();   // 关闭看门狗（长延时任务防止被复位）

    GpioInit(); // GPIO功能初始化

    /************ 通讯串口初始化 ************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // GPIO_12复用为UART2_RX

    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200,   // 波特率
        .dataBits = 8,        // 数据位
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);

    /************ 红外传感器引脚（输入） ************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIO_IR_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIO_IR_R, WIFI_IOT_GPIO_DIR_IN);

    /************ 创建防跌落任务 ************/
    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.name = "avoid_task";
    attr.priority = 25;

    if (osThreadNew((osThreadFunc_t)avoid_task, NULL, &attr) == NULL)
    {
        printf("Failed to create avoid_task!\n");
    }
}

APP_FEATURE_INIT(correspondence); // 启动任务