#include "control.h"
#include "ti_msp_dl_config.h"

#define TRACK_CENTER            350
#define TRACK_KP_NUM            9
#define TRACK_KD_NUM            18
#define TRACK_GAIN_DEN          10
#define TRACK_INTEGRAL_LIMIT    300
#define TRACK_KI_NUM            0

static TrackStatus g_track_status;
static int16_t g_last_error;
static int16_t g_integral;
static int8_t g_last_line_side;

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void Motor_Init(void)
{
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_STBY_PIN);
    DL_TimerG_startCounter(MOTOR_PWM_INST);
    Motor_SetSpeed(0, 0);
}

void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    left_speed = clamp_i16(left_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    right_speed = clamp_i16(right_speed, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

    if (left_speed > 0) {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN_1_PIN);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN_2_PIN);
    } else if (left_speed < 0) {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN_1_PIN);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN_2_PIN);
        left_speed = (int16_t)-left_speed;
    } else {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN_1_PIN | MOTOR_AIN_2_PIN);
    }

    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, (uint32_t)left_speed, DL_TIMER_CC_0_INDEX);

    if (right_speed > 0) {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN_1_PIN);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN_2_PIN);
    } else if (right_speed < 0) {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN_1_PIN);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN_2_PIN);
        right_speed = (int16_t)-right_speed;
    } else {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN_1_PIN | MOTOR_BIN_2_PIN);
    }

    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, (uint32_t)right_speed, DL_TIMER_CC_1_INDEX);
}

void Tracking_Reset(void)
{
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        g_track_status.sensor[i] = 0U;
    }

    g_track_status.sensor_mask = 0U;
    g_track_status.position = TRACK_CENTER;
    g_track_status.error = 0;
    g_track_status.turn = 0;
    g_track_status.line_found = 0U;
    g_last_error = 0;
    g_integral = 0;
    g_last_line_side = 0;
}

void Tracking_Init(void)
{
    Tracking_Reset();
}

static uint8_t Tracking_UpdatePosition(void)
{
    static const int16_t weights[GRAYSCALE_SENSOR_CHANNELS] = {
        0, 100, 200, 300, 400, 500, 600, 700
    };
    uint8_t i;
    uint8_t count = 0U;
    int32_t weighted_sum = 0;

    Grayscale_Sensor_Read_All(g_track_status.sensor);
    g_track_status.sensor_mask =
        Grayscale_Sensor_BuildMask(g_track_status.sensor);

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (g_track_status.sensor[i] != 0U) {
            weighted_sum += weights[i];
            count++;
        }
    }

    if (count == 0U) {
        g_track_status.line_found = 0U;
        return 0U;
    }

    g_track_status.position = (int16_t)(weighted_sum / count);
    g_track_status.error = (int16_t)(g_track_status.position - TRACK_CENTER);
    g_track_status.line_found = 1U;

    if (g_track_status.error > 40) {
        g_last_line_side = 1;
    } else if (g_track_status.error < -40) {
        g_last_line_side = -1;
    }

    return 1U;
}

static int16_t Tracking_CalcTurn(void)
{
    int16_t derivative;
    int32_t turn;

    g_integral = (int16_t)(g_integral + g_track_status.error);
    g_integral = clamp_i16(
        g_integral, -TRACK_INTEGRAL_LIMIT, TRACK_INTEGRAL_LIMIT);

    derivative = (int16_t)(g_track_status.error - g_last_error);
    g_last_error = g_track_status.error;

    turn = ((int32_t)TRACK_KP_NUM * g_track_status.error) +
           ((int32_t)TRACK_KD_NUM * derivative) +
           ((int32_t)TRACK_KI_NUM * g_integral);
    turn /= TRACK_GAIN_DEN;

    return clamp_i16((int16_t)turn, -TRACK_TURN_LIMIT, TRACK_TURN_LIMIT);
}

void Tracking_Run(void)
{
    int16_t left_speed;
    int16_t right_speed;

    if (Tracking_UpdatePosition() == 0U) {
        g_integral = 0;
        g_track_status.turn =
            (g_last_line_side >= 0) ? TRACK_TURN_LIMIT : -TRACK_TURN_LIMIT;

        if (g_last_line_side >= 0) {
            Motor_SetSpeed(TRACK_SEARCH_SPEED, -TRACK_SEARCH_SPEED);
        } else {
            Motor_SetSpeed(-TRACK_SEARCH_SPEED, TRACK_SEARCH_SPEED);
        }
        return;
    }

    g_track_status.turn = Tracking_CalcTurn();

    left_speed = (int16_t)(TRACK_BASE_SPEED + g_track_status.turn);
    right_speed = (int16_t)(TRACK_BASE_SPEED - g_track_status.turn);
    Motor_SetSpeed(left_speed, right_speed);
}

const TrackStatus *Tracking_GetStatus(void)
{
    return &g_track_status;
}
