#include "uart_task.h"
#include "arm.h"
#include "chassis.h"
#include <string.h>

/* 限幅宏 */
#define LIMIT_VEL(x, max) ((x) > (max) ? (max) : ((x) < -(max) ? -(max) : (x)))
#define LIMIT_U16( x, max) ((x) > (max) ? (max) : (x))
#define LIMIT_ARM(x)      ((x) > 3141 ? 3141 : ((x) < -3141 ? -3141 : (x)))

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

/* 状态帧 23 bytes (协议 §3) */
static uint8_t stat_buf[STAT_FRAME_LEN];

/* ==================== 状态帧组装 ==================== */

/**
 * 组装 23 字节状态帧 (协议 §3) → USART2 TX
 *
 * 升降实际高度从 dm_motor[4~7] 编码器实时计算:
 *   前升降 = (FR+FL)/2, 后升降 = (BL+BR)/2
 *   未使能电机不计入, 全未使能返回 0xFFFF
 *   pos(rad) → mm: pos × RACK_MM_PER_RAD × lift_dir[i]
 */
static void build_status_frame(void)
{
    /* ---- 升降实际高度 — 从 POS 电机编码器读取 (软件解卷绕) ---- */
    {
        float front_sum = 0.0f, back_sum = 0.0f;
        int   front_cnt = 0, back_cnt  = 0;
        static const float lift_dir[4]   = { LIFT_DIR_FR, LIFT_DIR_FL, LIFT_DIR_BL, LIFT_DIR_BR };
        static const float lift_offs[4]  = { LIFT_OFFSET_FR, LIFT_OFFSET_FL, LIFT_OFFSET_BL, LIFT_OFFSET_BR };

        /* DM_4310 POS 反馈 p_int→pos 固定 ±12.5rad (单圈), 超出回卷。
         * 一卷 = 512.5mm。平时以 prev 为锚追踪帧间位移, 首帧用 target 锚定。 */
        static float prev_mm[4];
        static bool  prev_valid[4];
        const float wrap_mm = 2.0f * 12.5f * RACK_MM_PER_RAD;  // 512.5mm

        /* 前: dm_motor[4]=FR, [5]=FL */
        for (int i = 4; i <= 5; i++)
        {
            if (dm_motor[i].start_flag)
            {
                float raw = dm_motor[i].para.pos * RACK_MM_PER_RAD * lift_dir[i - 4]
                          - lift_offs[i - 4] * RACK_MM_PER_RAD;
                /* 首帧: 以 target 锚定; 后续: 以 prev 追踪 */
                if (!prev_valid[i - 4])
                {
                    float tgt = (float)lift_target_mm[i - 4];
                    while (raw - tgt > wrap_mm * 0.5f) raw -= wrap_mm;
                    while (tgt - raw > wrap_mm * 0.5f) raw += wrap_mm;
                    prev_mm[i - 4]    = raw;
                    prev_valid[i - 4] = true;
                }
                else
                {
                    float diff = raw - prev_mm[i - 4];
                    if      (diff >  wrap_mm * 0.5f) raw -= wrap_mm;
                    else if (diff < -wrap_mm * 0.5f) raw += wrap_mm;
                    prev_mm[i - 4] = raw;
                }
                front_sum += raw;
                front_cnt++;
            }
        }
        /* 后: dm_motor[6]=BL, [7]=BR */
        for (int i = 6; i <= 7; i++)
        {
            if (dm_motor[i].start_flag)
            {
                float raw = dm_motor[i].para.pos * RACK_MM_PER_RAD * lift_dir[i - 4]
                          - lift_offs[i - 4] * RACK_MM_PER_RAD;
                if (!prev_valid[i - 4])
                {
                    float tgt = (float)lift_target_mm[i - 4];
                    while (raw - tgt > wrap_mm * 0.5f) raw -= wrap_mm;
                    while (tgt - raw > wrap_mm * 0.5f) raw += wrap_mm;
                    prev_mm[i - 4]    = raw;
                    prev_valid[i - 4] = true;
                }
                else
                {
                    float diff = raw - prev_mm[i - 4];
                    if      (diff >  wrap_mm * 0.5f) raw -= wrap_mm;
                    else if (diff < -wrap_mm * 0.5f) raw += wrap_mm;
                    prev_mm[i - 4] = raw;
                }
                back_sum += raw;
                back_cnt++;
            }
        }

        lift_front_actual = front_cnt ? (uint16_t)(front_sum / front_cnt) : 0xFFFF;
        lift_back_actual  = back_cnt  ? (uint16_t)(back_sum  / back_cnt)  : 0xFFFF;
    }

    /* ========== 组装 (协议 §3, 23 bytes) ========== */
    stat_buf[0]  = 0xCC;
    stat_buf[1]  = (uint8_t)(lift_front_actual >> 8);
    stat_buf[2]  = (uint8_t)(lift_front_actual & 0xFF);
    stat_buf[3]  = (uint8_t)(lift_back_actual >> 8);
    stat_buf[4]  = (uint8_t)(lift_back_actual & 0xFF);

    /* 光电/TOF 由独立 MCU 处理, 此处占位 */
    stat_buf[5]  = 0x00;
    stat_buf[6]  = 0x00;
    stat_buf[7]  = 0x00;
    stat_buf[8]  = 0x00;

    /* 左臂 (mrad, ID2 取反) */
    {
        int16_t lp1 = (int16_t)( arm_left_motor[0].para.pos * 1000.0f);
        int16_t lp2 = (int16_t)(-arm_left_motor[1].para.pos * 1000.0f);
        int16_t lp3 = (int16_t)( arm_left_motor[2].para.pos * 1000.0f);
        stat_buf[9]  = (uint8_t)(lp1 >> 8);
        stat_buf[10] = (uint8_t)(lp1 & 0xFF);
        stat_buf[11] = (uint8_t)(lp2 >> 8);
        stat_buf[12] = (uint8_t)(lp2 & 0xFF);
        stat_buf[13] = (uint8_t)(lp3 >> 8);
        stat_buf[14] = (uint8_t)(lp3 & 0xFF);
    }
    /* 右臂 (mrad, ID4/6 取反) */
    {
        int16_t rp1 = (int16_t)(-arm_right_motor[0].para.pos * 1000.0f);
        int16_t rp2 = (int16_t)( arm_right_motor[1].para.pos * 1000.0f);
        int16_t rp3 = (int16_t)(-arm_right_motor[2].para.pos * 1000.0f);
        stat_buf[15] = (uint8_t)(rp1 >> 8);
        stat_buf[16] = (uint8_t)(rp1 & 0xFF);
        stat_buf[17] = (uint8_t)(rp2 >> 8);
        stat_buf[18] = (uint8_t)(rp2 & 0xFF);
        stat_buf[19] = (uint8_t)(rp3 >> 8);
        stat_buf[20] = (uint8_t)(rp3 & 0xFF);
    }

    stat_buf[21] = 0;    // reserved
    stat_buf[22] = 0xEE;
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

    /* 限幅: ±2 m/s, ±3.2 rad/s (协议 §2.1) */
    set_vx =  (float)LIMIT_VEL(vx_raw, 20000);
    set_vy = -(float)LIMIT_VEL(vy_raw, 20000);
    set_vw = -(float)LIMIT_VEL(wz_raw, 32000);

    // 抬升目标 — 限幅 0~420mm (齿条满行程)
    lift_front_target = LIMIT_U16(read_uint16(&uart1_rx_buf[7]), 420);
    lift_back_target  = LIMIT_U16(read_uint16(&uart1_rx_buf[9]), 420);
    lift_mode         = uart1_rx_buf[11];

    // 机械臂 pitch (mrad) — 硬限幅 ±180°=±3141 mrad (协议 §2.4.4)
    left_pitch1  = LIMIT_ARM(read_int16(&uart1_rx_buf[12]));
    left_pitch2  = LIMIT_ARM(read_int16(&uart1_rx_buf[14]));
    left_pitch3  = LIMIT_ARM(read_int16(&uart1_rx_buf[16]));
    right_pitch1 = LIMIT_ARM(read_int16(&uart1_rx_buf[18]));
    right_pitch2 = LIMIT_ARM(read_int16(&uart1_rx_buf[20]));
    right_pitch3 = LIMIT_ARM(read_int16(&uart1_rx_buf[22]));

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

        vTaskDelay(20);  // 50Hz
    }
}
