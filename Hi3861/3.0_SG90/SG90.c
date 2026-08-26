#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"

osMutexId_t mutex_id;
#define GPIO2 2
uint8_t flag;    // 舵机旋转角度标志位

// SG90舵机控制：周期20ms（20000微秒），高电平500~2500us控制角度
void set_angle(unsigned int duty) {
    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT); // 设置GPIO2为输出模式

    // GPIO2输出x微秒高电平
    GpioSetOutputVal(GPIO2, 1);               // 输出高电平
    hi_udelay(duty);

    // GPIO2输出20000-x微秒低电平
    GpioSetOutputVal(GPIO2, 0);               // 输出低电平
    hi_udelay(20000 - duty);
}

// 0度
void engine_run_0(void)
{
    for (int i = 0; i < 10; i++) {
        set_angle(500);
    }
}

// 45度
void engine_run_45(void)
{
    for (int i = 0; i < 10; i++) {
        set_angle(1000);
    }
}

// 90度
void engine_run_90(void)
{
    for (int i = 0; i < 10; i++) {
        set_angle(1500);
    }
}

// 135度
void engine_run_135(void)
{
    for (int i = 0; i < 10; i++) {
        set_angle(2000);
    }
}

// 180度
void engine_run_180(void)
{
    for (int i = 0; i < 10; i++) {
        set_angle(2500);
    }
}

// 线程1：最高优先级，转90度
static void thread1(void)
{
    osDelay(100U);
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread1 is running.\r\n");
        flag = 90;
        engine_run_90();
        osDelay(500U);
        osMutexRelease(mutex_id);
    }
}

// 线程2：中优先级，只打印不操作舵机
static void thread2(void)
{
    osDelay(100U);
    while (1) {
        printf("thread2 is running.\r\n");
        switch(flag) {
            case 90:
                printf("SG90 turn 90 du.\r\n");
                break;
            case 180:
                printf("SG90 turn 180 du.\r\n");
                break;
            default:
                break;
        }
        flag = 0;
        osDelay(100);
    }
}

// 线程3：低优先级，转180度
static void thread3(void)
{
    while (1) {
        osMutexAcquire(mutex_id, osWaitForever);
        printf("thread3 is running.\r\n");
        flag = 180;
        engine_run_180();
        osDelay(300U);
        osMutexRelease(mutex_id);
    }
}

// 任务创建
static void SG90(void)
{
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_GPIO_DIR_OUT);

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.name = "thread1";
    attr.priority = 26;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Failed to create thread1!\n");
    }

    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Failed to create thread2!\n");
    }

    attr.name = "thread3";
    attr.priority = 24;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL) {
        printf("Failed to create thread3!\n");
    }

    mutex_id = osMutexNew(NULL);
    if (mutex_id == NULL) {
        printf("Failed to create Mutex!\n");
    }
}

APP_FEATURE_INIT(SG90);