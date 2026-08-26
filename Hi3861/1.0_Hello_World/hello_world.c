#include<stdio.h>
#include<unistd.h>
#include"ohos_init.h"
#include"cmsis_os2.h"

static void thread1(void);
static void thread2(void);   // 声明两个任务函数

static void Hello_World(void) {
    osThreadAttr_t attr;     // 定义一个任务属性结构体
    
    attr.attr_bits = 0U;     // 设置为0，表示不支持osThreadJoin（等待任务结束）
    attr.cb_mem = NULL;      // 控制块内存指针，NULL表示由系统自动分配
    attr.cb_size = 0U;       // 控制块大小，0表示使用默认值
    attr.stack_mem = NULL;   // 任务栈内存指针，NULL表示由系统自动分配
    attr.stack_size = 1024 * 4; // 任务栈大小 = 4096字节（4KB）
    // 创建任务1
    attr.name = "thread1";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL) {
        printf("Failed to create thread1!\n");
    }

    // 创建任务2
    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL) {
        printf("Failed to create thread2!\n");
    }
}

static void thread1(void) {
    while (1) {                            // 无限循环
        printf("Hello World!\r\n");
        usleep(1000000);                   // 延时1秒（微秒为单位）
    }
}

static void thread2(void) {
    sleep(1);                              // 休眠1秒（秒为单位）
    while (1) {
        printf("Hello QST!\r\n");
        usleep(3000000);                   // 延时3秒
    }
}

APP_FEATURE_INIT(Hello_World);