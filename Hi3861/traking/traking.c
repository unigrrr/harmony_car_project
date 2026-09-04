/*****************************************************************************/
/*  鸿蒙小车 - 循迹主程序（锐角岔路版 v7：修复原地误打转）                      */
/*  平台: Hi3861 (OpenHarmony LiteOS)，STM32 侧只跑电机PI闭环                  */
/*                                                                             */
/*  v7 改动(针对"有时突然原地转两圈"):                                          */
/*   根因: 强冲(内侧倒转)纠偏时车斜跨线 -> 双黑凑满消抖 -> 误判死路单线 ->       */
/*         uturn等不到传感器条件 -> 原地转满超时 ≈ 两圈。                        */
/*   1. 掉头前死路确认: 判到单线后慢速前行300ms，前方再见黑=误判，取消掉头     */
/*   2. 转向超时4000->2000ms并加日志: 即使误判，最多转一圈就放弃               */
/*                                                                             */
/*  v8 改动: 取消"压线沿强冲"。强冲是掩盖STM32电机响应慢的补丁，副作用是        */
/*   内侧轮倒转导致斜跨线误判横线。治本是STM32侧: 控制周期20ms + Velocity_KP   */
/*   加大(>=2.5)。若纠偏仍嫌慢，调 CORRECT_LO/CORRECT_HI 即可                  */
/*                                                                             */
/*  v5 改动(针对"检测到黑线反应太慢"):                                          */
/*   1. 压线的白->黑边沿瞬间强冲 KICK_MS 毫秒(KICK_LO/KICK_HI)，立即开始转，    */
/*      随后回落到 CORRECT_LO/CORRECT_HI 正常纠偏；设 KICK_MS=0 可关闭         */
/*   2. 根治延迟需配合 STM32 侧: 控制周期 100ms -> 20ms                         */
/*      (stm32f10x_it.c 里 millis%100 改 millis%20；control_system.c 里        */
/*       OverflowTime 100 改 20。PI每秒调节速率不变，无需重调)                 */
/*                                                                             */
/*  历史改动:                                                                   */
/*   - 纠偏单档: 压线就固定差速修一点，不分强弱、不回找                         */
/*   - 双黑确认 50ms(MARK_DEBOUNCE=5)，防纠偏晃动斜压线误判横线                 */
/*   - 转向动作全部传感器闭环: capture/uturn/hairpin，动作间无停顿             */
/*                                                                             */
/*  转向动作保持传感器闭环:                                                     */
/*   - capture 并入岔路: 向岔侧强转，岔侧传感器由黑变白(线到车底中间)即交还循迹 */
/*   - uturn   掉头180: 转出横线 -> 右传感器碰线(90°) -> 双黑(180°) -> 停      */
/*   - hairpin 恢复急转: 对侧传感器第1次碰线=主路跳过，第2次=岔路才停          */
/*                                                                             */
/*  赛道: 起点(两条横线) -> 岔口1(锐角,走左) -> 岔口2(锐角,走右) -> 终点(双线) */
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

/************** 传感器极性 **************/
/* 按你们最新实测: 黑线 = VALUE1, 白地 = VALUE0。如果现场反过来，改这里即可 */
#define BLACK_VAL   WIFI_IOT_GPIO_VALUE1

/************** 车速（rad/s x100，范围 ±150） **************/
#define SPD_BASE    40    /* 直行速度 */
#define SPD_SLOW    30    /* 过横线判别窗口内的慢速 */
#define CORRECT_LO  0     /* 纠偏时内侧轮速度(越小转越急，可调负=倒转) */
#define CORRECT_HI  90    /* 纠偏时外侧轮速度(协议上限150) */
#define SPD_TURN    120   /* 原地转向速度 */
#define CAP_INNER   0     /* 并入岔路时内侧轮速度 */
#define CAP_OUTER   110   /* 并入岔路时外侧轮速度 */

/* 左右轮个体差异补偿(%): 直行时车往左偏=右轮偏快 -> 把TRIM_B调小(如95)
   或TRIM_A调大(如105)，每次±3~5地调。所有电机指令统一生效 */
#define TRIM_A      100   /* 左轮(A路)补偿 */
#define TRIM_B      100   /* 右轮(B路)补偿 */

/************** 时间与阈值（控制周期 10ms） **************/
#define LOOP_US         10000
#define MARK_DEBOUNCE   5     /* 双黑消抖次数(x10ms): 加严防纠偏晃动误判横线 */
#define DOUBLE_WIN_MS   600   /* 单线/双线判别窗口: 略大于"过两条横线"的时间 */
#define FORK_PULSE_MIN  6     /* 岔口脉冲最少持续次数(x10ms): 大于弯道纠偏压线、小于岔口斜扫压线 */
#define CAPTURE_MIN_MS  200   /* 并入最小时间: 先离开岔口触发点 */
#define CAPTURE_MAX_MS  2500  /* 并入超时保护 */
#define REJECT_WIN_MS   500   /* 直行过岔窗口(ACT_REJECT 时用) */
#define QUIET_MS        400   /* 动作完成后的静默循迹期 */
#define TURN_MAX_MS     2000  /* 转向各阶段超时保护(约1圈): 等不到条件最多转2秒 */
#define CONFIRM_MS      300   /* 死路确认: 判到单线后再前行，看前方是否还有线 */
#define RECOVERY_EN     1     /* 1=走错自动掉头恢复; 0=走错直接停车 */

/************** 岔口配置（按现场实际填！） **************/
typedef enum { ACT_CAPTURE, ACT_REJECT } ForkAct;
typedef struct { int side; ForkAct act; } ForkCfg;
static const ForkCfg g_forks[] = {
    { 0, ACT_CAPTURE },   /* 岔口1: 岔路在左，走左 */
    { 1, ACT_CAPTURE },   /* 岔口2: 岔路在右，走右 */
};
#define FORK_NUM  ((int)(sizeof(g_forks) / sizeof(g_forks[0])))

/************** 电机指令（与 STM32 协议一致，勿改） **************/
static uint8_t uart_sendbuf[20];

void stm32motor_control(int motorA, int motorB)   /* motorA=左轮, motorB=右轮 */
{
    uint8_t A_dir = 0, B_dir = 0;
    if (motorA < 0) { A_dir = 1; motorA = -motorA; }
    if (motorB < 0) { B_dir = 1; motorB = -motorB; }
    motorA = motorA * TRIM_A / 100;          /* 左右轮个体差异补偿 */
    motorB = motorB * TRIM_B / 100;
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;

    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = A_dir;
    uart_sendbuf[2] = motorA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = motorB;
    uart_sendbuf[5] = 0xFD;
    UartWrite(MOTOR_UART, (unsigned char *)uart_sendbuf, 6);
}

static void car_stop(void) { stm32motor_control(0, 0); }

/************** 传感器 **************/
static int see_black(int gpio)
{
    WifiIotGpioValue v;
    GpioGetInputVal(gpio, &v);
    return (v == BLACK_VAL);
}

static int both_black(void)
{
    return see_black(GPIO_IR_L) && see_black(GPIO_IR_R);
}

/************** 循迹纠偏（单档差速，无强冲） **************/
/* 左压线=车偏右了向左修；右压线=车偏左了向右修。
   已取消"压线沿强冲": 那是掩盖STM32电机响应慢的补丁，内侧轮倒转会造成
   斜跨线->误判横线->原地打转。若纠偏仍嫌慢，治本在STM32侧(20ms控制周期、
   Velocity_KP>=2.5)，或把CORRECT_LO调小(可负)、CORRECT_HI调大 */
static void follow_spd(int base)
{
    int l = see_black(GPIO_IR_L);
    int r = see_black(GPIO_IR_R);

    if (l && !r)       stm32motor_control(CORRECT_LO, CORRECT_HI);  /* 向左修 */
    else if (!l && r)  stm32motor_control(CORRECT_HI, CORRECT_LO);  /* 向右修 */
    else if (!l && !r) stm32motor_control(base, base);              /* 居中直行 */
    /* 双黑: 横线事件，交给事件层处理 */
}

static void follow(void) { follow_spd(SPD_BASE); }

/************** 横线事件：双黑消抖，触发一次返回1 **************/
static int marker_event(void)
{
    static int cnt = 0;
    if (both_black()) {
        if (cnt < 255) cnt++;
        if (cnt == MARK_DEBOUNCE) return 1;
    } else {
        cnt = 0;
    }
    return 0;
}

/************** 单线/双线判别：返回 1=单线 2=双线 **************/
static int classify_marker(void)
{
    int t = 0, cnt = 0;

    while (both_black()) {                 /* 慢速开出当前横线 */
        follow_spd(SPD_SLOW);
        usleep(LOOP_US);
        t += 10;
        if (t > DOUBLE_WIN_MS) return 1;
    }
    while (t < DOUBLE_WIN_MS) {            /* 窗口期内看有没有第二条 */
        follow_spd(SPD_SLOW);
        if (both_black()) {
            if (++cnt >= MARK_DEBOUNCE) return 2;
        } else {
            cnt = 0;
        }
        usleep(LOOP_US);
        t += 10;
    }
    return 1;
}

/************** 死路确认：真死路横线前方无线；误判时前方还有线 **************/
/* 判到"单线"后慢速前行 CONFIRM_MS，期间任一传感器再见黑 => 是误判，不掉头 */
static int confirm_dead_end(void)
{
    int t = 0;
    while (t < CONFIRM_MS) {
        follow_spd(SPD_SLOW);
        if (see_black(GPIO_IR_L) || see_black(GPIO_IR_R)) return 0;
        usleep(LOOP_US);
        t += 10;
    }
    return 1;
}

/************** 锐角岔口检测：岔侧长脉冲 **************/
static int fork_pulse_check(void)
{
    static int cntL = 0, cntR = 0;
    int l = see_black(GPIO_IR_L);
    int r = see_black(GPIO_IR_R);
    int ret = 0;

    if (l && !r)       { cntL++; cntR = 0; }
    else if (!l && r)  { cntR++; cntL = 0; }
    else               { cntL = 0; cntR = 0; }

    if (cntL >= FORK_PULSE_MIN) { cntL = 0; ret |= 1; }
    if (cntR >= FORK_PULSE_MIN) { cntR = 0; ret |= 2; }
    return ret;
}

/************** 动作原语（全部传感器闭环，动作间不停顿） **************/
static void cross_straight(void)          /* 直行开过横线区，防二次触发 */
{
    int t = 0;
    stm32motor_control(SPD_BASE, SPD_BASE);
    while (both_black()) {
        usleep(LOOP_US);
        t += 10;
        if (t > 800) break;
    }
    usleep(120000);
}

static void quiet_follow(int ms)          /* 静默循迹：不响应任何事件 */
{
    int t = 0;
    while (t < ms) {
        follow();
        usleep(LOOP_US);
        t += 10;
    }
}

/* 并入岔路(闭环): 向岔侧强转，岔侧传感器由黑变白(线到车底中间)即交还循迹 */
static void capture(int side)
{
    int gpio = (side == 0) ? GPIO_IR_L : GPIO_IR_R;
    int t = 0, white_cnt = 0;

    while (t < CAPTURE_MAX_MS) {
        if (side == 0) stm32motor_control(CAP_INNER, CAP_OUTER);
        else           stm32motor_control(CAP_OUTER, CAP_INNER);
        usleep(LOOP_US);
        t += 10;
        if (t < CAPTURE_MIN_MS) continue;
        if (!see_black(gpio)) {
            if (++white_cnt >= 3) break;
        } else {
            white_cnt = 0;
        }
    }
    quiet_follow(QUIET_MS);
}

static void reject(void)
{
    int t = 0;
    while (t < REJECT_WIN_MS) {
        stm32motor_control(SPD_BASE, SPD_BASE);
        usleep(LOOP_US);
        t += 10;
    }
    quiet_follow(QUIET_MS);
}

/* 掉头180(闭环，免标定): 转出横线 -> 右传感器碰线(90°) -> 双黑(180°) -> 停 */
static void uturn_180(void)
{
    int t;
    stm32motor_control(SPD_TURN, -SPD_TURN);

    t = 0;
    while (both_black()) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] uturn s1 timeout\r\n"); goto done; }
    }
    t = 0;
    while (!see_black(GPIO_IR_R)) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] uturn s2 timeout\r\n"); goto done; }
    }
    t = 0;
    while (see_black(GPIO_IR_R)) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] uturn s3 timeout\r\n"); goto done; }
    }
    t = 0;
    while (!both_black()) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] uturn s4 timeout\r\n"); goto done; }
    }
done:
    cross_straight();
}

/* 恢复急转(闭环): 对侧传感器第1次碰线=主路跳过，第2次=岔路才停 */
static void hairpin(int side)
{
    int wait_gpio = (side == 0) ? GPIO_IR_R : GPIO_IR_L;
    int t;

    if (side == 0) stm32motor_control(-SPD_TURN, SPD_TURN);
    else           stm32motor_control(SPD_TURN, -SPD_TURN);

    t = 0;
    while (!see_black(wait_gpio)) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] hairpin s1 timeout\r\n"); goto done; }
    }
    t = 0;
    while (see_black(wait_gpio)) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] hairpin s2 timeout\r\n"); goto done; }
    }
    t = 0;
    while (!see_black(wait_gpio)) {
        usleep(LOOP_US);
        if ((t += 10) > TURN_MAX_MS) { printf("[TRACK] hairpin s3 timeout\r\n"); goto done; }
    }
done:
    quiet_follow(QUIET_MS);
}

/************** 主状态机 **************/
typedef enum { ST_START, ST_RUN, ST_END, ST_DONE } TrackState;

static void tracking_task(void)
{
    TrackState st = ST_START;
    int fork_idx = 0;
    int recovering = 0;

    car_stop();
    sleep(2);                     /* 等 STM32 侧初始化完 */
    printf("[TRACK] start\r\n");

    while (1) {
        if (st == ST_DONE) {
            car_stop();
            usleep(200000);
            continue;
        }

        /* ---- 1. 横线事件：起点双线 / 终点双线 / 死路单线 ---- */
        if (marker_event()) {
            int type = classify_marker();       /* 1=单线 2=双线 */

            if (recovering) {
                /* 掉头返回途中不应遇横线，忽略 */
            } else if (type == 2) {
                if (st == ST_START) {
                    st = ST_RUN;
                    printf("[TRACK] pass start\r\n");
                } else {
                    car_stop();
                    st = ST_DONE;
                    printf("[TRACK] FINISH!\r\n");
                }
            } else {
                if (st == ST_RUN || st == ST_END) {
                    if (confirm_dead_end()) {      /* 前方真没线才是死路 */
                        printf("[TRACK] wrong end, uturn\r\n");
#if RECOVERY_EN
                        uturn_180();
                        recovering = 1;
#else
                        car_stop();
                        st = ST_DONE;
#endif
                    } else {
                        printf("[TRACK] false marker, ignore\r\n");
                    }
                }
            }
        }
        /* ---- 2. 掉头恢复途中：岔路出现在相反侧，急转并入 ---- */
        else if (recovering) {
            int mask = fork_pulse_check();
            if (mask) {
                int side = (mask & 1) ? 0 : 1;
                int i, missed = -1;
                for (i = 0; i < FORK_NUM; i++) {
                    if (g_forks[i].side == 1 - side) missed = i;
                }
                if (missed < 0) missed = (fork_idx > 0) ? fork_idx - 1 : 0;
                hairpin(side);
                fork_idx = missed + 1;
                if (fork_idx >= FORK_NUM) st = ST_END;
                else if (st == ST_START) st = ST_RUN;
                recovering = 0;
                printf("[TRACK] recovered, fork_idx=%d\r\n", fork_idx);
            } else {
                follow();
            }
        }
        /* ---- 3. 岔口检测（锐角岔路 = 岔侧长脉冲） ---- */
        else if (st != ST_END && fork_idx < FORK_NUM) {
            int mask = fork_pulse_check();
            int need = g_forks[fork_idx].side;
            if ((need == 0 && (mask & 1)) || (need == 1 && (mask & 2))) {
                if (g_forks[fork_idx].act == ACT_CAPTURE) capture(need);
                else                                      reject();
                printf("[TRACK] fork %d done\r\n", fork_idx + 1);
                fork_idx++;
                if (st == ST_START) st = ST_RUN;
                if (fork_idx >= FORK_NUM) st = ST_END;
            } else {
                follow();
            }
        }
        /* ---- 4. 正常循迹 ---- */
        else {
            follow();
        }

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
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(MOTOR_UART, &uart_attr2, NULL);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIO_IR_L, WIFI_IOT_GPIO_DIR_IN);
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
        printf("Failed to create tracking_task!\r\n");
    }
}

APP_FEATURE_INIT(tracking_main);