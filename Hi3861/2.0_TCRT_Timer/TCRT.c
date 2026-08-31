#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include <stdlib.h>
#include <memory.h>
#include "cmsis_os2.h"
#include "hi_io.h"
#include "hi_time.h"

#define GPIOL 13
#define GPIOR 14

uint32_t exec1;            // 时间变量
osTimerId_t id1;           // 创建时间ID
uint32_t timerDelay_1;     // 创建延时时间
osStatus_t status;         // 创建返回参数

// 获取红外传感器输出的电平高低
void get_tcrt5000_value(void)
{
    WifiIotGpioValue id_status;    // 声明变量id_status

    GpioGetInputVal(GPIOL, &id_status);    // 获取GPIO13引脚的输入电平值

    // 如果GPIO13输入电平值是低电平，串口打印"left black"，说明左边的红外传感器识别到了黑色（此时传感器灯熄灭）
    if (id_status == WIFI_IOT_GPIO_VALUE0) {
        printf("left black\r\n");
    } else {
        printf("left white\r\n");
    }

    GpioGetInputVal(GPIOR, &id_status);    // 获取GPIO14引脚的输入电平值

    // 如果GPIO14输入电平值是低电平，串口打印"right black"，说明右边的红外传感器识别到了黑色（此时传感器灯熄灭）
    if (id_status == WIFI_IOT_GPIO_VALUE0) {
        printf("right black\r\n");
    } else {
        printf("right white\r\n");
    }
}

/***** 定时器1 回调函数 *****/
void Timer1_Callback(void *arg)    // 定时器超时后会回调这个函数
{
    (void)arg;
    // 循环执行获取左右两个传感器值的任务，并且每次获取之间会等待2秒钟
    get_tcrt5000_value();

// 定时器关闭
// status = osTimerStop(id1);
// if (status == osOK)
//     printf("定时关闭成功\n");
}

/***** 任务一 *****/
static void TCRTTask(void)
{
    printf("start test tcrt5000\r\n");

    /***** 定时器创建 *****/
    exec1 = 1U;
    // 创建定时器并获取ID
    id1 = osTimerNew(Timer1_Callback, osTimerPeriodic, &exec1, NULL);
    if (id1 != NULL) {
        // Hi3861 1U=10ms, 100U=1S
        // 设置定时器超时时间10秒
        timerDelay_1 = 5U;
        // 定时器开启并获取返回参数
        status = osTimerStart(id1, timerDelay_1);
        if (status != osOK) {
            printf("Timer could not be started\r\n");
        } else {
            printf("定时开启成功！\n");
        }
    }
}

void TCRT(void)
{
    GpioInit();    // 初始化GPIO
    // GPIO功能复用为普通GPIO功能
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    // 设置GPIO功能为输入
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_GPIO_DIR_IN);

    osThreadAttr_t attr;
    attr.name = "TCRTTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = 25;

    if (osThreadNew((osThreadFunc_t)TCRTTask, NULL, &attr) == NULL) {
        printf("Falied to create RobotCarTestTask!\n");
    }
}

APP_FEATURE_INIT(TCRT);    // 启动任务