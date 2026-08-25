# 20026.8.25
## 一、STM连接与烧录
### 1.硬件连接
- ST-LINK连接电脑USB口和开发板接口
- 烧录完成后，拔掉 ST-LINK，再打开小车电源
### 2.烧录
-Build或Rebuild后Download
-Keil编译为.HEX后通过ST-LINK搬运只芯片内部地址
## 二、小车车灯点亮
```c
#include "stm32f10x.h"
#include "sys.h"

int main(void)
{
    Stm32_Clock_Init(9);  
    MY_NVIC_PriorityGroupConfig(2);
    uart_init(115200);
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    colorful_led_Init();
    printf("QST青软\r\n");

    while(1)
    {
        // 前灯6个全亮
        L_ws2812_rgb(1, WS_WHITE);
        L_ws2812_rgb(2, WS_WHITE);
        L_ws2812_rgb(3, WS_WHITE);
        L_ws2812_rgb(4, WS_WHITE);
        L_ws2812_rgb(5, WS_WHITE);
        L_ws2812_rgb(6, WS_WHITE);
        L_ws2812_refresh(led_num);

        // 后灯6个全亮
        R_ws2812_rgb(1, WS_WHITE);
        R_ws2812_rgb(2, WS_WHITE);
        R_ws2812_rgb(3, WS_WHITE);
        R_ws2812_rgb(4, WS_WHITE);
        R_ws2812_rgb(5, WS_WHITE);
        R_ws2812_rgb(6, WS_WHITE);
        R_ws2812_refresh(led_num);
    }
}
```
## 三、linux基础命令
### 1.常用指令
- ls 展示当前目录下的文件
- cd 目录名 进入知名目录
- cd ~进入主目录
- cd -回到上一步操作
- cd ..回到上一级目录
- ip a查询模拟机ip地址
- sudo apt update更新软件包
### 2.vscode远程连接
1. 安装 VSCode 插件：**Remote - SSH**
2. 新建远程
3. 输入：`用户名@IP地址`
4. 输入密码
此后可在windows的VSCode终端编辑Linux代码
