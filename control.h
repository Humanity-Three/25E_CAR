#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "grayscale_sensor.h"

#define MOTOR_SPEED_MAX         2500
#define TRACK_BASE_SPEED        1150
#define TRACK_TURN_LIMIT        900
#define TRACK_SEARCH_SPEED      650

typedef struct {
    uint16_t sensor[GRAYSCALE_SENSOR_CHANNELS];
    uint8_t sensor_mask;
    int16_t position;
    int16_t error;
    int16_t turn;
    uint8_t line_found;
} TrackStatus;

void Motor_Init(void);
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

void Tracking_Init(void);
void Tracking_Reset(void);
void Tracking_Run(void);
const TrackStatus *Tracking_GetStatus(void);

#endif
