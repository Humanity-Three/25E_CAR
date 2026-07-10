#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include <stdint.h>

extern volatile uint32_t g_sys_tick_10ms;
extern volatile int32_t g_left_motor_pulse;
extern volatile int32_t g_right_motor_pulse;

#endif
