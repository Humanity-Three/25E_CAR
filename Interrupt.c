#include "Interrupt.h"
#include "ti_msp_dl_config.h"

volatile int32_t g_left_motor_pulse  = 0;
volatile int32_t g_right_motor_pulse = 0;

static int32_t prev_left_pulse  = 0;
static int32_t prev_right_pulse = 0;
// ========== 10ms 定时器中断 (TIMG7 / TIMER_0) ==========
void TIMG7_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    g_sys_tick_10ms++;
}
// ========== 50ms 定时器中断 (TIMG6 / MOTOR_ENCODER_READ) ==========
// 计算每 50ms 的脉冲增量 = 速度值，用于 PID
void TIMG6_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(MOTOR_ENCODER_READ_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);

    /* 当前脉冲数 */
    int32_t cur_left  = g_left_motor_pulse;
    int32_t cur_right = g_right_motor_pulse;

    /* 计算增量（速度），外部可以随时读取 */
    volatile int32_t left_speed  = cur_left  - prev_left_pulse;
    volatile int32_t right_speed = cur_right - prev_right_pulse;

    prev_left_pulse  = cur_left;
    prev_right_pulse = cur_right;
}

// ========== GPIO 中断 - PORTA (PA26 = B1, 左轮 B 相) ==========
void GROUP0_IRQHandler(void)
{
    if (DL_GPIO_readInterruptStatus(GPIOA, ENCODER_READ_ENCODER_B1_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_READ_ENCODER_B1_PIN);
        /* B↑：读 A1 (PB23)，A1 为高 → +1，A1 为低 → -1 */
        if (DL_GPIO_readPins(GPIOB, ENCODER_READ_ENCODER_A1_PIN))
            g_left_motor_pulse++;
        else
            g_left_motor_pulse--;
    }
}

// ========== GPIO 中断 - PORTB (PB23=左A1, PB24=右B2, PB27=右A2) ==========
void GROUP1_IRQHandler(void)
{
    /* 左轮 A1 ↑：读 B1 (PA26)，B1 为低 → +1，B1 为高 → -1 */
    if (DL_GPIO_readInterruptStatus(GPIOB, ENCODER_READ_ENCODER_A1_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_READ_ENCODER_A1_PIN);
        if (DL_GPIO_readPins(GPIOA, ENCODER_READ_ENCODER_B1_PIN) == 0)
            g_left_motor_pulse++;
        else
            g_left_motor_pulse--;
    }

    /* 右轮 B2 ↑：读 A2 (PB27)，A2 为高 → +1，A2 为低 → -1 */
    if (DL_GPIO_readInterruptStatus(GPIOB, ENCODER_READ_ENCODER_B2_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_READ_ENCODER_B2_PIN);
        if (DL_GPIO_readPins(GPIOB, ENCODER_READ_ENCODER_A2_PIN))
            g_right_motor_pulse++;
        else
            g_right_motor_pulse--;
    }

    /* 右轮 A2 ↑：读 B2 (PB24)，B2 为低 → +1，B2 为高 → -1 */
    if (DL_GPIO_readInterruptStatus(GPIOB, ENCODER_READ_ENCODER_A2_PIN)) {
        DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_READ_ENCODER_A2_PIN);
        if (DL_GPIO_readPins(GPIOB, ENCODER_READ_ENCODER_B2_PIN) == 0)
            g_right_motor_pulse++;
        else
            g_right_motor_pulse--;
    }
}
