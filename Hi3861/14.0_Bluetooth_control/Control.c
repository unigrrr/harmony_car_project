/*
 * ble_car_demo_speed.c
 * 基于已验证可用的官方例程，加入速度调节功能。
 *
 * 硬件链路：手机App --BLE--> JDY-16 --UART1(GPIO_0/1)--> Hi3861
 *           --UART2(GPIO_11/12)--> STM32 --> 电机
 *
 * 蓝牙指令集：
 *   'W' 前进   'S' 后退   'A' 左转   'D' 右转   'O' 停止
 *   '1'~'5' 速度档位（40/70/100/125/150）
 *   '+' 加速10   '-' 减速10（调速后立即按当前运动状态生效）
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"
#include "hi_task.h"
#include "wifiiot_errno.h"

uint8_t uart_sendbuf[20];
uint8_t bluetooth_flag[1000];      // 蓝牙接收缓冲

static int  g_speed    = 100;      // 当前速度 0~150（默认100，与原例程前进速度一致）
static char g_last_cmd = 'O';      // 当前运动状态，调速后立即按此重新执行

/***通信协议***/
/*
发送至stm32的数据协议
参数: 电机速度rad/s的一百倍，例如: 设置转速为1rad/s则传入100
*/
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    // 确认旋转方向 正转: 0 反转: 1
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

    // 限制幅度 -150 ~ 150
    if (motorA > 150) {
        motorA = 150;
    }
    if (motorB > 150) {
        motorB = 150;
    }

    // 数据协议
    uart_sendbuf[0] = 0xFC;    // 帧头
    uart_sendbuf[1] = A_dir;   // 左轮方向  0正转, 1反转
    uart_sendbuf[2] = motorA;  // 左轮速度
    uart_sendbuf[3] = B_dir;   // 右轮方向  0正转, 1反转
    uart_sendbuf[4] = motorB;  // 右轮速度
    uart_sendbuf[5] = 0xFD;    // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车运动函数：全部跟随 g_speed（转向保持原例程约 1/3 反转的手感）
void car_backward(void)
{
    stm32motor_control(-g_speed, -g_speed);
}

void car_forward(void)
{
    stm32motor_control(g_speed, g_speed);
}

void car_left(void)
{
    stm32motor_control(-g_speed / 3, g_speed);
}

void car_right(void)
{
    stm32motor_control(g_speed, -g_speed / 3);
}

void car_stop(void)
{
    stm32motor_control(0, 0);
}

// 执行方向指令（接收处理和调速后复用）
static void apply_cmd(char cmd)
{
    switch (cmd)
    {
        case 'O': car_stop();     break;
        case 'W': car_forward();  break;
        case 'A': car_left();     break;
        case 'D': car_right();    break;
        case 'S': car_backward(); break;
        default:  return;         // 非方向指令不更新状态
    }
    g_last_cmd = cmd;
}

// 调速：立即按当前运动状态重新生效
static void speed_apply(void)
{
    apply_cmd(g_last_cmd);
    printf("[BLE] speed=%d\r\n", g_speed);
}

static void car_mode_bluetooth(void)         // 蓝牙模式
{
    while (1)
    {
        UartRead(WIFI_IOT_UART_IDX_1, bluetooth_flag, 1000);
        if (bluetooth_flag[0] != 0)
        {
            switch (bluetooth_flag[0])       // 判断接收的字符
            {
                /* 方向/停止 */
                case 'O':
                case 'W':
                case 'A':
                case 'D':
                case 'S':
                    apply_cmd(bluetooth_flag[0]);
                    break;

                /* 速度档位 */
                case '1': g_speed = 40;  speed_apply(); break;
                case '2': g_speed = 70;  speed_apply(); break;
                case '3': g_speed = 100; speed_apply(); break;
                case '4': g_speed = 125; speed_apply(); break;
                case '5': g_speed = 150; speed_apply(); break;

                /* 速度微调 */
                case '+':
                    g_speed += 10;
                    if (g_speed > 150) g_speed = 150;
                    speed_apply();
                    break;
                case '-':
                    g_speed -= 10;
                    if (g_speed < 0) g_speed = 0;
                    speed_apply();
                    break;

                default:
                    break;
            }
            bluetooth_flag[0] = 0;           // 清空缓冲字符
        }
        hi_sleep(50);
    }
}

/*****任务创建*****/
static void Control(void)
{
    uint32_t ret;

    GpioInit();   // GPIO功能初始化

    /********************蓝牙初始化********************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD); // GPIO_0复用为UART1_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD); // GPIO_1复用为UART1_RX

    WifiIotUartAttribute uart_attr1 = {
        .baudRate = 9600,      // 保持你验证可用的 9600，勿动
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    ret = UartInit(WIFI_IOT_UART_IDX_1, &uart_attr1, NULL);
    if (ret != WIFI_IOT_SUCCESS)
    {
        printf("Failed to init uart! Err code = %d\n", ret);
        return;
    }
    printf("ble uart OK!");

    /********************通讯串口初始化********************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // GPIO_12复用为UART2_RX

    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    ret = UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
    if (ret != WIFI_IOT_SUCCESS)
    {
        printf("Failed to init uart! Err code = %d\n", ret);
        return;
    }
    printf("uart OK!");

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;

    attr.name = "car_mode_bluetooth";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)car_mode_bluetooth, NULL, &attr) == NULL)
    {
        printf("Falied to create car_mode_bluetooth!\n");
    }
}

/*****启动任务（添加在整个文件的最末尾）*****/
APP_FEATURE_INIT(Control);   // 启动任务