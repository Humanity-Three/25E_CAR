#include "control.h"

volatile uint32_t g_sys_tick_10ms = 0;

#define Kp 0.5
#define Ki 0.01
#define Kd 0.01
