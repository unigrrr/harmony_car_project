/*****************************************************************************/
/*  鸿蒙小车 - 自动避障 + 禁区识别（黑色胶带）整合控制程序                      */
/*  运行平台: Hi3861 (OpenHarmony LiteOS)                                     */
/*  注意: Hi3861 的 osDelay() 单位是 10ms，osDelay(100)=1秒                  */
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

/************** 引脚定义 **************/
#define GPIO_TRIG    7
#define GPIO_ECHO    8
#define GPIO_STEER   2
#define GPIO_IR_L    13
#define GPIO_IR_R    14
#define MOTOR_UART   WIFI_IOT_UART_IDX_2

/************** 参数配置（单位：osDelay 的 10ms tick）**************/
#define SAFE_DISTANCE_CM     15.0f
#define DANGER_DISTANCE_CM   10.0f
#define SPEED_FORWARD        40
#define SPEED_BACKWARD       80
#define SPEED_TURN           110     // 加大转速

/* osDelay 单位是 10ms，所以 100 = 1秒 */
#define BACKWARD_DELAY       150     // 后退 1.0 秒
#define TURN_DELAY           150     // 转向 1.5 秒
#define STOP_DELAY           30      // 停下等 0.3 秒

#define ANGLE_0_US   500
#define ANGLE_45_US  1000
#define ANGLE_90_US  1500
#define ANGLE_135_US 2000
#define ANGLE_180_US 2500

typedef enum {
    STATE_IDLE = 0,
    STATE_FORWARD,
    STATE_AVOID_OBSTACLE,
    STATE_AVOID_FORBIDDEN,
    STATE_EDGE_DETECTED
} CarState;

static CarState g_carState = STATE_IDLE;
static uint8_t uart_sendbuf[20];
static const int scan_angles[]  = {0, 45, 90, 135, 180};
static const int scan_duties[]  = {ANGLE_0_US, ANGLE_45_US, ANGLE_90_US, ANGLE_135_US, ANGLE_180_US};
#define SCAN_NUM  5

#define BLACK_IS_VALUE0   0
#define CONFIRM_OBSTACLE     3
#define CONFIRM_FORBIDDEN    1

void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0, B_dir = 0;
    if (motorA < 0) { A_dir = 1; motorA = -motorA; }
    if (motorB < 0) { B_dir = 1; motorB = -motorB; }
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

void car_forward(void)  { stm32motor_control(SPEED_FORWARD, SPEED_FORWARD); }
void car_backward(void) { stm32motor_control(-SPEED_BACKWARD, -SPEED_BACKWARD); }
void car_left(void)     { stm32motor_control(-SPEED_TURN, SPEED_TURN); }
void car_right(void)    { stm32motor_control(SPEED_TURN, -SPEED_TURN); }
void car_stop(void)     { stm32motor_control(0, 0); }

void set_angle(unsigned int duty)
{
    GpioSetDir(GPIO_STEER, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(GPIO_STEER, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(GPIO_STEER, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

void steer_to_angle(int angle_idx)
{
    int duty = scan_duties[angle_idx];
    for (int i = 0; i < 15; i++) set_angle(duty);
}

void steer_center(void) { steer_to_angle(2); }

float GetDistance(void)
{
    unsigned long start_time = 0, time = 0;
    float distance = 0.0f;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    unsigned int timeout = 0;

    GpioSetDir(GPIO_ECHO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIO_TRIG, WIFI_IOT_GPIO_DIR_OUT);

    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE0);

    while (1) {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1) {
            start_time = hi_get_us();
            flag = 1;
            break;
        }
        if (++timeout > 30000) return 999.0f;
    }

    timeout = 0;
    while (1) {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            break;
        }
        if (++timeout > 30000) return 999.0f;
    }

    distance = (float)time * 0.034f / 2.0f;
    if (distance < 2.0f) distance = 2.0f;
    if (distance > 400.0f) distance = 999.0f;
    return distance;
}

void ScanDistance(float *distances)
{
    for (int i = 0; i < SCAN_NUM; i++) {
        steer_to_angle(i);
        osDelay(30);              // 等300ms = osDelay(30)
        distances[i] = GetDistance();
        printf("[SCAN] %d deg -> %.1f cm\r\n", scan_angles[i], distances[i]);
    }
    steer_center();
}

void GetInfraredStatus(WifiIotGpioValue *left, WifiIotGpioValue *right)
{
    GpioGetInputVal(GPIO_IR_L, left);
    GpioGetInputVal(GPIO_IR_R, right);
}

int ForbiddenDetected(WifiIotGpioValue *left, WifiIotGpioValue *right)
{
    GetInfraredStatus(left, right);
#if BLACK_IS_VALUE0
    if (*left == WIFI_IOT_GPIO_VALUE0 || *right == WIFI_IOT_GPIO_VALUE0)
        return 1;
#else
    if (*left == WIFI_IOT_GPIO_VALUE1 || *right == WIFI_IOT_GPIO_VALUE1)
        return 1;
#endif
    return 0;
}

int ForbiddenEscapeDecision(WifiIotGpioValue left, WifiIotGpioValue right)
{
#if BLACK_IS_VALUE0
    if (left == WIFI_IOT_GPIO_VALUE0 && right == WIFI_IOT_GPIO_VALUE0) return 2;
    if (left == WIFI_IOT_GPIO_VALUE0) return 1;
    if (right == WIFI_IOT_GPIO_VALUE0) return 0;
#else
    if (left == WIFI_IOT_GPIO_VALUE1 && right == WIFI_IOT_GPIO_VALUE1) return 2;
    if (left == WIFI_IOT_GPIO_VALUE1) return 1;
    if (right == WIFI_IOT_GPIO_VALUE1) return 0;
#endif
    return 2;
}

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

    if (edge_cnt >= 4) {
        edge_cnt = 0;
        return 1;
    }
    return 0;
}

int AvoidObstacleDecision(void)
{
    float distances[SCAN_NUM];
    int best_dir = 2;
    float max_dist = 0.0f;

    printf("[AVOID] Start scanning 5 directions...\r\n");
    ScanDistance(distances);

    for (int i = 0; i < SCAN_NUM; i++) {
        if (distances[i] > max_dist && distances[i] < 900.0f) {
            max_dist = distances[i];
            best_dir = i;
        }
    }

    printf("[AVOID] Best direction: %d deg, distance: %.1f cm\r\n",
           scan_angles[best_dir], max_dist);

    if (max_dist < SAFE_DISTANCE_CM) return 2;
    if (best_dir < 2) return 0;
    if (best_dir > 2) return 1;
    return 2;
}

static void auto_nav_task(void)
{
    WifiIotGpioValue debug_l, debug_r;
    GetInfraredStatus(&debug_l, &debug_r);
    printf("[IR_RAW] LEFT=%d RIGHT=%d\r\n", debug_l, debug_r);

    WifiIotGpioValue ir_left, ir_right;
    float front_dist = 0.0f;
    int action = 0;
    static int obstacle_cnt = 0;
    static int forbidden_cnt = 0;

    printf("[AUTO_NAV] Started!\r\n");
    steer_center();
    car_stop();
    osDelay(200);                 // sleep(2) → osDelay(200) = 2秒
    g_carState = STATE_FORWARD;

    while (1)
    {

        WifiIotGpioValue test_l, test_r;
        GpioGetInputVal(GPIO_IR_L, &test_l);
        GpioGetInputVal(GPIO_IR_R, &test_r);
        printf("[IR] L=%d R=%d\r\n", test_l, test_r);  // 持续打印
        /* ===== 1. 禁区识别 ===== */
        if (ForbiddenDetected(&ir_left, &ir_right))
        {
            forbidden_cnt++;
            if (forbidden_cnt < CONFIRM_FORBIDDEN) {
                car_forward();
                osDelay(1);       // 50ms巡检
                continue;
            }
            printf("[FORBIDDEN] Confirmed! L=%d R=%d\r\n", ir_left, ir_right);
            g_carState = STATE_AVOID_FORBIDDEN;
            forbidden_cnt = 0;

            car_stop(); osDelay(10);   // 等0.1秒
            car_backward(); osDelay(BACKWARD_DELAY);   // 后退1秒

            action = ForbiddenEscapeDecision(ir_left, ir_right);
            if (action == 0) car_left();
            else if (action == 1) car_right();
            else car_left();

            osDelay(TURN_DELAY);      // 转向1.5秒
            car_stop();
            g_carState = STATE_FORWARD;
            continue;
        }
        else { forbidden_cnt = 0; }

        /* ===== 2. 桌沿检测 ===== */
        if (EdgeDetected(&ir_left, &ir_right))
        {
            printf("[EDGE] Detected! L=%d R=%d\r\n", ir_left, ir_right);
            g_carState = STATE_EDGE_DETECTED;
            car_backward(); osDelay(150);   // 后退1秒

            int safe_cnt = 0;
            for (int i = 0; i < 40; i++) {
                WifiIotGpioValue l, r;
                GpioGetInputVal(GPIO_IR_L, &l);
                GpioGetInputVal(GPIO_IR_R, &r);
                if (l == WIFI_IOT_GPIO_VALUE0 && r == WIFI_IOT_GPIO_VALUE0) {
                    if (++safe_cnt >= 2) break;
                } else safe_cnt = 0;
                osDelay(5);       // 50ms
            }

            if (ir_left == WIFI_IOT_GPIO_VALUE1) car_right();
            else car_left();
            osDelay(TURN_DELAY);
            car_stop();
            g_carState = STATE_FORWARD;
            continue;
        }

        /* ===== 3. 前方避障 ===== */
        front_dist = GetDistance();
        printf("[DIST] Front: %.1f cm\r\n", front_dist);

        /* 3.1 危险距离 */
        if (front_dist < DANGER_DISTANCE_CM && front_dist > 1.0f)
        {
            printf("[DANGER] %.1f cm -> Backward fixed\r\n", front_dist);
            g_carState = STATE_AVOID_OBSTACLE;
            obstacle_cnt = 0;

            car_backward();
            osDelay(BACKWARD_DELAY);   // 后退1秒

            car_stop();
            osDelay(STOP_DELAY);       // 停0.3秒
            printf("[AVOID] Stopped. Start scanning...\r\n");

            action = AvoidObstacleDecision();

            if (action == 0) {
                printf("[AVOID] -> Turn LEFT\r\n");
                car_left();
            } else if (action == 1) {
                printf("[AVOID] -> Turn RIGHT\r\n");
                car_right();
            } else {
                printf("[AVOID] All blocked -> Backward & Turn LEFT\r\n");
                car_backward(); osDelay(30); car_left();
            }
            osDelay(TURN_DELAY);       // 转向1.5秒
            car_stop();

            g_carState = STATE_FORWARD;
            continue;
        }

        /* 3.2 安全距离内 */
        else if (front_dist < SAFE_DISTANCE_CM && front_dist > 1.0f)
        {
            obstacle_cnt++;
            printf("[WARN] Obstacle count: %d/%d, dist: %.1f cm\r\n",
                   obstacle_cnt, CONFIRM_OBSTACLE, front_dist);

            if (obstacle_cnt < CONFIRM_OBSTACLE) {
                car_forward();
                osDelay(5);
                continue;
            }

            printf("[AVOID] Confirmed! -> Backward fixed\r\n");
            g_carState = STATE_AVOID_OBSTACLE;
            obstacle_cnt = 0;

            car_backward();
            osDelay(BACKWARD_DELAY);

            car_stop();
            osDelay(STOP_DELAY);
            printf("[AVOID] Stopped. Start scanning...\r\n");

            action = AvoidObstacleDecision();

            if (action == 0) {
                printf("[AVOID] -> Turn LEFT\r\n");
                car_left();
            } else if (action == 1) {
                printf("[AVOID] -> Turn RIGHT\r\n");
                car_right();
            } else {
                printf("[AVOID] All blocked -> Backward & Turn LEFT\r\n");
                car_backward(); osDelay(30); car_left();
            }
            osDelay(TURN_DELAY);
            car_stop();

            g_carState = STATE_FORWARD;
            continue;
        }
        else { obstacle_cnt = 0; }

        /* ===== 4. 正常前进 ===== */
        g_carState = STATE_FORWARD;
        car_forward();
        osDelay(1);       // 50ms巡检
    }
}

static void auto_navigation_init(void)
{
    WatchDogDisable();
    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(GPIO_TRIG, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(GPIO_ECHO, WIFI_IOT_GPIO_DIR_IN);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(GPIO_STEER, WIFI_IOT_GPIO_DIR_OUT);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIO_IR_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIO_IR_R, WIFI_IOT_GPIO_DIR_IN);

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200, .dataBits = 8, .stopBits = 1, .parity = 0,
    };
    UartInit(MOTOR_UART, &uart_attr2, NULL);

    osThreadAttr_t attr = {0};
    attr.stack_size = 1024 * 8;
    attr.name = "auto_nav_task";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)auto_nav_task, NULL, &attr) == NULL) {
        printf("Failed to create auto_nav_task!\n");
    }
}

APP_FEATURE_INIT(auto_navigation_init);