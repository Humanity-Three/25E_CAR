#include <stdint.h>
#include "control.h"
#include "ti_msp_dl_config.h"
#include "grayscale_sensor.h"
#include "Delay.h"
#include "OLED.h"

typedef enum {
    STATE_IDLE = 0,
    STATE_INPUT,
    STATE_RUNNING,
#if 0
    /* K230 targeting states are reserved for the next stage. */
    STATE_COLLIMATION,
    STATE_COLLIMATION_INPUT,
    STATE_RUNNING_COLLIMATION,
#endif
    STATE_STOP
} car_STATE;

typedef struct {
    uint8_t key1;
    uint8_t key2;
} car_KEY;

static car_STATE current_state = STATE_IDLE;
static car_STATE next_state = STATE_IDLE;
static int number_of_circles = 0;
static int true_number_of_circles = 0;

static uint8_t Key_ReadPressed(uint32_t pin)
{
    return (DL_GPIO_readPins(KEY_PORT, pin) == 0U) ? 1U : 0U;
}

static car_KEY Key_Read(void)
{
    car_KEY key;

    key.key1 = Key_ReadPressed(KEY_KEY_1_PIN);
    key.key2 = Key_ReadPressed(KEY_KEY_2_PIN);
    return key;
}

static void move_to_next_state(car_KEY key)
{
    next_state = current_state;

    switch (current_state) {
    case STATE_IDLE:
        number_of_circles = 0;
        true_number_of_circles = 0;
        if (key.key1 != 0U) {
            next_state = STATE_INPUT;
        }
        break;

    case STATE_INPUT:
        if (key.key2 != 0U) {
            Tracking_Reset();
            next_state = STATE_RUNNING;
        }
        break;

    case STATE_RUNNING:
        if (key.key2 != 0U) {
            next_state = STATE_STOP;
        } else if ((number_of_circles > 0) &&
                   (true_number_of_circles >= number_of_circles)) {
            next_state = STATE_STOP;
        }
        break;

    case STATE_STOP:
    default:
        next_state = STATE_IDLE;
        break;
    }
}

static void station_action(car_KEY key)
{
    static uint8_t key1_last = 0U;

    switch (current_state) {
    case STATE_IDLE:
        Motor_SetSpeed(0, 0);
        break;

    case STATE_INPUT:
        if ((key.key1 != 0U) && (key1_last == 0U)) {
            number_of_circles++;
            if (number_of_circles > 5) {
                number_of_circles = 0;
            }
        }
        Motor_SetSpeed(0, 0);
        break;

    case STATE_RUNNING:
        Tracking_Run();
        break;

    case STATE_STOP:
    default:
        number_of_circles = 0;
        true_number_of_circles = 0;
        Tracking_Reset();
        Motor_SetSpeed(0, 0);
        break;
    }

    key1_last = key.key1;
}

int main(void)
{
    car_KEY key;

    SYSCFG_DL_init();
    Grayscale_Sensor_Init();
    Motor_Init();
    Tracking_Init();
    OLED_Init();
    OLED_ShowString(0, 0, (char *)"25E READY", OLED_8X16);
    OLED_ShowString(0, 16, (char *)"KEY1 INPUT", OLED_6X8);
    OLED_Update();

    while (1) {
        key = Key_Read();
        current_state = next_state;
        move_to_next_state(key);
        station_action(key);
        Delay_ms(5);
    }
}
