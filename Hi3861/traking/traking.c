/*****************************************************************************/
/*  鸿蒙小车 - 循迹主程序（v22：双黑时序 + v11手感 + 快速转入）                      */
/*  平台: Hi3861 (OpenHarmony LiteOS)，STM32 侧只跑电机PI闭环                  */
/*                                                                             */
/*  需求(用户重申):                                                             */
/*    循迹纠偏保持不变; 全程只认"双黑"这一种事件:                              */
/*      第1次 0.5秒内双黑 -> 向左转(岔口1)                                     */
/*      第2次 0.5秒内双黑 -> 向右转(岔口2)                                     */
/*      之后   0.3秒内双黑 -> 停车(终点)                                       */
/*  "X秒内双黑": 一侧压黑后, 另一侧在X秒内也压黑(含同时双黑), 即算一次        */
/*                                                                             */
/*  注意: 车要放在起点双线之后发车(起点线会被当成第1次双黑)。                  */
/*  差速只出现在: 循迹纠偏 + 岔口转入; 转入为双轮前进弧线(内慢外快)            */
/*****************************************************************************/
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
#include "hi_time.h"

/************** 引脚 **************/
#define GPIO_IR_L   13    /* 左红外 */
#define GPIO_IR_R   14    /* 右红外 */
#define MOTOR_UART  WIFI_IOT_UART_IDX_2

/************** 速度参数（与STM32侧联调好的值，勿随意改） **************/
#define SPD_BASE    40    /* 巡线基准速度 */
/* 纠偏手感(以你上传的"手感好"版本为准): 压线时内侧轮=CORRECT_LO, 外侧轮=CORRECT_HI */
#define CORRECT_LO  10    /* 纠偏: 内侧轮速度(你上传的"手感好"版本=10) */
#define CORRECT_HI  100   /* 纠偏: 外侧轮加速到该值 */
#define CAP_INNER   0     /* 岔口转入内侧轮速度(0=内侧停, 急转; 要更猛改-20) */
#define CAP_OUTER   100   /* 岔口转入外侧轮速度(快速果断转入; 过头减小) */

/* 左右轮个体差异补偿(%): 直行往左偏=右轮偏快 -> TRIM_B调小，每次±3~5 */
#define TRIM_A      100   /* 左轮(A路)补偿 */
#define TRIM_B      100   /* 右轮(B路)补偿 */

/************** 时间与阈值（控制周期 10ms） **************/
#define LOOP_US         10000
#define BLACK_DB        2     /* 单侧压黑确认拍数(x10ms), 去毛刺 */
#define FORK_WIN        80    /* 岔口1双黑配对窗口(x10ms) = 0.5秒 */
#define FORK2_WIN       100    /* 岔口2双黑配对窗口(x10ms) = 0.7秒, 单独可调 */
#define END_WIN         30    /* 终点双黑配对窗口(x10ms) = 0.3秒 */
#define SUPPRESS_TICKS  150   /* 岔口2转弯后抑制双黑检测的拍数, 防同一路口重复触发 */
#define SUPPRESS2_TICKS 6200  /* 岔口1转弯后的抑制拍数(60秒): 期间任何双黑都不算岔口2,
                                 彻底排除大圈弯道误触发; 若60秒内就到岔口2会不转, 再调小 */
#define CAPTURE_MIN_MS  120   /* 岔口转入找线段最大时间(找线快就提前进下一段) */
#define CAPTURE_MAX_MS  2500  /* 岔口转入超时保护 */
#define QUIET_MS        400   /* 转入完成后的静默循迹期 */

/************** 与STM32的串口协议（STM32侧已实现，勿改帧格式） **************/
static void stm32motor_control(int a_speed, int b_speed)
{
    unsigned char msg[6];
    int as, bs;
    a_speed = a_speed * TRIM_A / 100;
    b_speed = b_speed * TRIM_B / 100;
    if (a_speed > 150)  a_speed = 150;
    if (a_speed < -150) a_speed = -150;
    if (b_speed > 150)  b_speed = 150;
    if (b_speed < -150) b_speed = -150;
    msg[0] = 0xFC;
    msg[1] = (a_speed >= 0) ? 0 : 1;  as = (a_speed >= 0) ? a_speed : -a_speed;
    msg[2] = (unsigned char)as;
    msg[3] = (b_speed >= 0) ? 0 : 1;  bs = (b_speed >= 0) ? b_speed : -b_speed;
    msg[4] = (unsigned char)bs;
    msg[5] = 0xFD;
    UartWrite(MOTOR_UART, msg, sizeof(msg));
}

static void car_stop(void)
{
    stm32motor_control(0, 0);
}

/************** 传感器：黑=1（模块亮黄灯），白=0 **************/
static int see_black(int gpio)
{
    WifiIotGpioValue v = WIFI_IOT_GPIO_VALUE0;
    GpioGetInputVal(gpio, &v);
    return v == WIFI_IOT_GPIO_VALUE1;
}

/************** 循迹（纠偏逻辑保持不变） **************/
static void follow_spd(int base)
{
    int lb = see_black(GPIO_IR_L);
    int rb = see_black(GPIO_IR_R);
    if (lb && !rb)      stm32motor_control(CORRECT_LO, CORRECT_HI);  /* 左压线: 向左修回 */
    else if (rb && !lb) stm32motor_control(CORRECT_HI, CORRECT_LO);  /* 右压线: 向右修回 */
    else                stm32motor_control(base, base);        /* 双白/双黑: 直行 */
}

static void follow(void)
{
    follow_spd(SPD_BASE);
}

/* 静默循迹 ms 毫秒(转弯后稳定用, 不做任何事件检测) */
static void quiet_follow(int ms)
{
    int t = 0;
    while (t < ms) {
        follow();
        usleep(LOOP_US);
        t += 10;
    }
}

/************** 双黑事件检测 **************/
/* 一侧压黑满 BLACK_DB 拍后记为"确认压黑"; 当一侧确认压黑且另一侧在 win 拍内
   也确认压过黑 -> 触发一次双黑事件。同一次压黑簇只触发一次:
   触发后必须由 bb_reset() 复位(主循环处理完事件后调用) */
static int g_bbreset = 0;

static void bb_reset(void)
{
    g_bbreset = 1;
}

static int bb_event(int win)
{
    static int runL = 0, runR = 0;    /* 当前连续压黑拍数 */
    static int cbL = 99, cbR = 99;    /* 距左/右"确认压黑"的拍数 */
    static int fired = 0, wgap = 0;   /* 簇内只报一次; 双白5拍后才重新待命 */
    int l, r, ev = 0;

    if (g_bbreset) {                  /* 事件已消费, 清掉陈旧压线数据 */
        g_bbreset = 0;
        runL = runR = 0;
        cbL = cbR = 99;
        wgap = 0;                     /* fired 不清: 靠双白5拍重新待命, 防同簇连发 */
    }
    l = see_black(GPIO_IR_L);
    r = see_black(GPIO_IR_R);

    if (l) { if (runL < 99) runL++; if (runL == BLACK_DB) cbL = 0; }
    else   { runL = 0; }
    if (r) { if (runR < 99) runR++; if (runR == BLACK_DB) cbR = 0; }
    else   { runR = 0; }

    if (!fired) {
        if ((runL >= BLACK_DB && cbR <= win) ||
            (runR >= BLACK_DB && cbL <= win)) {
            ev = 1;
            fired = 1;                /* 本次压黑簇只报一次 */
            printf("[TRACK] BB event, cbL=%d cbR=%d\r\n", cbL, cbR);
        }
    }
    if (!l && !r) {
        if (wgap < 99) wgap++;
        if (wgap >= 5) fired = 0;     /* 双白5拍 = 这一簇结束, 重新待命 */
    } else {
        wgap = 0;
    }

    if (runL == 0 && cbL < 99) cbL++;   /* 没压着才开始数间隔 */
    if (runR == 0 && cbR < 99) cbR++;
    return ev;
}

/************** 岔口转入(唯一的转向动作，闭环): 双轮都前进、内侧慢外侧快，弧线入弯。
   两段: 先"找线"(岔侧传感器见到黑)再"过线"(由黑变白, 岔路线到车底中间) */
static void capture(int side)
{
    int gpio = (side == 0) ? GPIO_IR_L : GPIO_IR_R;
    int t = 0, cnt = 0;

    /* 段1: 找线(最多 CAPTURE_MIN_MS, 同时也充入弯的最小转向时间) */
    while (t < CAPTURE_MIN_MS) {
        if (side == 0) stm32motor_control(CAP_INNER, CAP_OUTER);   /* 左转 */
        else           stm32motor_control(CAP_OUTER, CAP_INNER);   /* 右转 */
        if (see_black(gpio)) {
            if (++cnt >= 2) break;
        } else {
            cnt = 0;
        }
        usleep(LOOP_US);
        t += 10;
    }

    /* 段2: 过线(岔侧由黑变白, 线到两传感器中间; 超时兜底) */
    t = 0; cnt = 0;
    while (t < CAPTURE_MAX_MS) {
        if (side == 0) stm32motor_control(CAP_INNER, CAP_OUTER);
        else           stm32motor_control(CAP_OUTER, CAP_INNER);
        if (!see_black(gpio)) {
            if (++cnt >= 3) break;
        } else {
            cnt = 0;
        }
        usleep(LOOP_US);
        t += 10;
    }
    quiet_follow(QUIET_MS);           /* 稳定一小段再恢复检测 */
}

/************** 主任务 **************/
static void tracking_task(void *arg)
{
    int st = 0;          /* 0=等岔口1(左转) 1=等岔口2(右转) 2=等终点(停车) 3=已停车 */
    int suppress = 0;    /* 双黑检测抑制剩余拍数 */
    int tick = 0;        /* 运行拍数(x10ms), 日志用 */
    (void)arg;

    car_stop();
    sleep(2);                     /* 等 STM32 侧初始化完 */
    printf("[TRACK] start, v25\r\n");

    while (1) {
        int ev = 0;

        if (st == 3) {
            car_stop();
            usleep(200000);
            continue;
        }

        if (suppress > 0) {
            suppress--;
        } else {
            ev = bb_event(st == 2 ? END_WIN : (st == 1 ? FORK2_WIN : FORK_WIN));
        }

        if (ev) {
            bb_reset();
            if (st == 0) {
                printf("[TRACK] fork1 turn L @%d.%ds\r\n", tick / 100, (tick % 100) / 10);
                capture(0);
                st = 1;
                suppress = SUPPRESS2_TICKS;   /* 岔口1后静默更久: 大圈弯道不算岔口2 */
            } else if (st == 1) {
                printf("[TRACK] fork2 turn R @%d.%ds\r\n", tick / 100, (tick % 100) / 10);
                capture(1);
                st = 2;
                suppress = SUPPRESS_TICKS;
            } else {
                car_stop();
                st = 3;
                printf("[TRACK] FINISH! @%d.%ds\r\n", tick / 100, (tick % 100) / 10);
            }
        } else {
            follow();
        }

        if (tick < 2000000000) tick++;
        usleep(LOOP_US);
    }
}

/************** 初始化与任务创建 **************/
static void tracking_main(void)
{
    WatchDogDisable();

    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200,
        .dataBits = 8,
        .parity = 0,
        .stopBits = 1,
    };
    UartInit(MOTOR_UART, &uart_attr2, NULL);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    GpioSetDir(GPIO_IR_L, WIFI_IOT_GPIO_DIR_IN);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIO_IR_R, WIFI_IOT_GPIO_DIR_IN);

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.name = "tracking_task";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)tracking_task, NULL, &attr) == NULL) {
        printf("[TRACK] create task failed\r\n");
    }
}

APP_FEATURE_INIT(tracking_main);