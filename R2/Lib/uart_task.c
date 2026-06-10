#include "uart_task.h"
#include "arm.h"
#include <string.h>

/* ==================== DMA 接收缓冲区 ==================== */
uint8_t uart1_rx_buf[CTRL_FRAME_LEN];

/* ==================== 解析后的协议变量 ==================== */

/* 抬升 */
uint16_t lift_front_target = 0;
uint16_t lift_back_target  = 0;
uint8_t  lift_mode         = 0;
uint16_t lift_front_actual = 0;
uint16_t lift_back_actual  = 0;

/* 机械臂 (协议值 mrad, 初始化为原点等效值, 已计入 arm_*_update 内的方向转换) */
int16_t left_pitch1  = 0,     left_pitch2  = 2990,  left_pitch3  = -1440;
int16_t right_pitch1 = 0,     right_pitch2 = 2990,  right_pitch3 = -1440;

/* 吸盘 / 武器 */
uint8_t left_sucker    = 0;
uint8_t right_sucker   = 0;
uint8_t weapon_pitch   = 0;
uint8_t weapon_gripper = 0;

/* 旧兼容 */
float   upstairs_chassis_speed = 0.0f;
uint8_t up_dis_front = 0;
uint8_t up_dis_back  = 0;

/* Arm position setpoints (from deleted arm.c) */
float fb_des = 100.0f, lr_des = 0.0f, ud_des = 50.0f, end_des = -1.57f;

/* ==================== TX 帧 ==================== */

/* 状态帧 27 bytes */
static uint8_t stat_buf[STAT_FRAME_LEN];

/* USART3 传感器帧 */
static uint8_t uart3_sensor_buf[UART3_SENSOR_FRAME_LEN] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEE};

/* ==================== 状态帧组装 ==================== */

static void build_status_frame(void)
{
    stat_buf[0]  = 0xCC;
    stat_buf[1]  = (uint8_t)(lift_front_actual >> 8);
    stat_buf[2]  = (uint8_t)(lift_front_actual & 0xFF);
    stat_buf[3]  = (uint8_t)(lift_back_actual >> 8);
    stat_buf[4]  = (uint8_t)(lift_back_actual & 0xFF);
    // TODO: ToF 距离 (当前填 0xFFFF 表示无效)
    stat_buf[5]  = 0xFF; stat_buf[6]  = 0xFF;  // tof_front_front
    stat_buf[7]  = 0xFF; stat_buf[8]  = 0xFF;  // tof_front_back
    stat_buf[9]  = 0xFF; stat_buf[10] = 0xFF;  // tof_back_front
    stat_buf[11] = 0xFF; stat_buf[12] = 0xFF;  // tof_back_back
    /* 左臂当前角度 (mrad, 从 CAN2 反馈读取, 方向已适配协议正=上) */
    {
        int16_t lp1 = (int16_t)( arm_left_motor[0].para.pos * 1000.0f);
        int16_t lp2 = (int16_t)(-arm_left_motor[1].para.pos * 1000.0f);  // ID2电机上=负,取反
        int16_t lp3 = (int16_t)( arm_left_motor[2].para.pos * 1000.0f);
        stat_buf[13] = (uint8_t)(lp1 >> 8);
        stat_buf[14] = (uint8_t)(lp1 & 0xFF);
        stat_buf[15] = (uint8_t)(lp2 >> 8);
        stat_buf[16] = (uint8_t)(lp2 & 0xFF);
        stat_buf[17] = (uint8_t)(lp3 >> 8);
        stat_buf[18] = (uint8_t)(lp3 & 0xFF);
    }
    /* 右臂当前角度 (mrad, 从 CAN2 反馈读取, ID4/6 取反) */
    {
        int16_t rp1 = (int16_t)(-arm_right_motor[0].para.pos * 1000.0f);  // ID4取反
        int16_t rp2 = (int16_t)( arm_right_motor[1].para.pos * 1000.0f);
        int16_t rp3 = (int16_t)(-arm_right_motor[2].para.pos * 1000.0f);  // ID6取反
        stat_buf[19] = (uint8_t)(rp1 >> 8);
        stat_buf[20] = (uint8_t)(rp1 & 0xFF);
        stat_buf[21] = (uint8_t)(rp2 >> 8);
        stat_buf[22] = (uint8_t)(rp2 & 0xFF);
        stat_buf[23] = (uint8_t)(rp3 >> 8);
        stat_buf[24] = (uint8_t)(rp3 & 0xFF);
    }
    stat_buf[25] = 0;    // reserved
    stat_buf[26] = 0xEE;
}

/* ==================== 传感器帧 ==================== */

static void update_sensor_buf(void)
{
    uart3_sensor_buf[3] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET) ? 0x00 : 0x01;
    uart3_sensor_buf[2] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)  == GPIO_PIN_SET) ? 0x00 : 0x01;
    uart3_sensor_buf[1] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET) ? 0x00 : 0x01;
}

/* ==================== 初始化 ==================== */

void uart_rx_init(void)
{
    HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, CTRL_FRAME_LEN);
}

void uart_tx_ch5_notify(void)
{
    // 新协议无需独立通知帧, 状态帧在自动模式持续回传即表示已切换
}

/* ==================== 控制帧解析 ==================== */

static inline int16_t read_int16(const uint8_t *buf)
{
    return (int16_t)((buf[0] << 8) | buf[1]);
}

static inline uint16_t read_uint16(const uint8_t *buf)
{
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

static void parse_ctrl_frame(void)
{
    // 帧头尾校验
    if (uart1_rx_buf[0] != 0xCC || uart1_rx_buf[CTRL_FRAME_LEN - 1] != 0xEE)
    {
        HAL_UART_DMAStop(&huart1);
        memset(uart1_rx_buf, 0, CTRL_FRAME_LEN);
        HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, CTRL_FRAME_LEN);
        return;
    }

    // 底盘速度: int16, 10000 = 1 m/s or 1 rad/s
    int16_t vx_raw = read_int16(&uart1_rx_buf[1]);
    int16_t vy_raw = read_int16(&uart1_rx_buf[3]);
    int16_t wz_raw = read_int16(&uart1_rx_buf[5]);

    set_vx =  (float)vx_raw;
    set_vy = -(float)vy_raw;
    set_vw = -(float)wz_raw;

    // 抬升目标
    lift_front_target = read_uint16(&uart1_rx_buf[7]);
    lift_back_target  = read_uint16(&uart1_rx_buf[9]);
    lift_mode         = uart1_rx_buf[11];

    // 机械臂 pitch (mrad)
    left_pitch1  = read_int16(&uart1_rx_buf[12]);
    left_pitch2  = read_int16(&uart1_rx_buf[14]);
    left_pitch3  = read_int16(&uart1_rx_buf[16]);
    right_pitch1 = read_int16(&uart1_rx_buf[18]);
    right_pitch2 = read_int16(&uart1_rx_buf[20]);
    right_pitch3 = read_int16(&uart1_rx_buf[22]);

    // 吸盘
    left_sucker  = uart1_rx_buf[24];
    right_sucker = uart1_rx_buf[25];

    // 武器头
    weapon_pitch   = uart1_rx_buf[26];
    weapon_gripper = uart1_rx_buf[27];

    // [28] reserved, ignored
}

/* ==================== 主任务 ==================== */

void uart_task(void *parameter)
{
    while (1)
    {
        if (sys_enabled && ch_high(5))
        {
            parse_ctrl_frame();

            // 状态帧 50Hz → USART2 TX (上位机 RX)
            build_status_frame();
            HAL_UART_Transmit(&huart2, stat_buf, STAT_FRAME_LEN, HAL_MAX_DELAY);
        }

        // USART3 传感器帧 5Hz (每 200ms, 即 10 个周期)
        static uint8_t sensor_divider = 0;
        if (++sensor_divider >= 10)
        {
            sensor_divider = 0;
            update_sensor_buf();
            HAL_UART_Transmit(&huart3, uart3_sensor_buf, UART3_SENSOR_FRAME_LEN, HAL_MAX_DELAY);
        }

        vTaskDelay(20);  // 50Hz
    }
}
