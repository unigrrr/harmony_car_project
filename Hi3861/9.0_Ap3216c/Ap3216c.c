#include <stdio.h>
#include <unistd.h>
#include<string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ap3216c.h"
#include"hal_bsp_ssd1306.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"

/**
 * ir 人体红外传感器
 * als 光强传感器
 * ps 接近传感器
 */
void Task1(void)
{
    AP3216C_Init();    // 三合一传感器初始化
    SSD1306_Init();
    SSD1306_CLS();
    printf("i2c_ap3216c_demo()!");
    uint8_t displayBuff[30] = {0};
    uint16_t ir = 0, als = 0, ps = 0;

    SSD1306_ShowStr(0, 0, (uint8_t *)"   QST CAR   ", 16);

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_6, WIFI_IOT_IO_FUNC_GPIO_6_GPIO); // 设为普通GPIO
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_6, WIFI_IOT_GPIO_DIR_OUT);       // 输出模式
    GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_6, 0);                     // 初始熄灭

    while (1)
    {
        AP3216C_ReadData(&ir, &als, &ps);
        printf("ir = %d    als = %d    ps = %d\r\n", ir, als, ps);

        if (als < 50)
        {
            GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_6, 1); // 高电平点亮
        }
        else
        {
            GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_6, 0); // 低电平熄灭
        }

        memset(displayBuff, 0, sizeof(displayBuff));
        sprintf((char *)displayBuff, "I:%d A:%d P:%d", ir, als, ps);
        SSD1306_ShowStr(0, 3, (uint8_t *)displayBuff, 16);

        sleep(1);  // 1s
    }
}

static void i2c_ap3216c_demo(void)
{
    osThreadAttr_t options;
    options.name = "thread_1";
    options.attr_bits = 0;
    options.cb_mem = NULL;
    options.cb_size = 0;
    options.stack_mem = NULL;
    options.stack_size = 1024;
    options.priority = osPriorityNormal;
    osThreadId_t Task1_ID;
    Task1_ID = osThreadNew((osThreadFunc_t)Task1, NULL, &options);
    if (Task1_ID != NULL)
    {
        printf("ID = %d, Create Task1_ID is OK!", Task1_ID);
    }
}

APP_FEATURE_INIT(i2c_ap3216c_demo);