#include "stm32f10x.h"
#include "sys.h"

int main(void)
  { 
		Stm32_Clock_Init(9);						//外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====中断优先级分组		
		uart_init(115200);	            //=====串口初始化为115200
		JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

		colorful_led_Init();            //=====炫彩灯初始化

		printf("QST青软\r\n");
		/**主要程序**/
	while(1)
	{  
		L_ws2812_rgb(1, WS_WHITE);
        L_ws2812_rgb(2, WS_WHITE);
        L_ws2812_rgb(3, WS_WHITE);
        L_ws2812_rgb(4, WS_WHITE);
        L_ws2812_rgb(5, WS_WHITE);
        L_ws2812_rgb(6, WS_WHITE);
        L_ws2812_refresh(led_num);

        R_ws2812_rgb(1, WS_WHITE);
        R_ws2812_rgb(2, WS_WHITE);
        R_ws2812_rgb(3, WS_WHITE);
        R_ws2812_rgb(4, WS_WHITE);
        R_ws2812_rgb(5, WS_WHITE);
        R_ws2812_rgb(6, WS_WHITE);
        R_ws2812_refresh(led_num);
	}
}
	

