#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

//=====================================================================================
//  引脚配置接口 - 以 SysConfig 生成的 ti_msp_dl_config.h 为准
//  引脚组: GRAY_SENSOR (PORT=GPIOA)
//  AD0=PA14, AD1=PA15, AD2=PA16, OUT=PA17
//=====================================================================================

// 所有引脚共用同一个端口 GPIOA（由 GRAY_SENSOR_PORT 定义）
#define SENSOR_AD0_PORT         GRAY_SENSOR_PORT
#define SENSOR_AD0_PIN          GRAY_SENSOR_GRAY_SENSOR_AD0_PIN

#define SENSOR_AD1_PORT         GRAY_SENSOR_PORT
#define SENSOR_AD1_PIN          GRAY_SENSOR_GRAY_SENSOR_AD1_PIN

#define SENSOR_AD2_PORT         GRAY_SENSOR_PORT
#define SENSOR_AD2_PIN          GRAY_SENSOR_GRAY_SENSOR_AD2_PIN

#define GrayS_OUT_PORT          GRAY_SENSOR_PORT
#define GrayS_OUT_PIN           GRAY_SENSOR_GRAY_SENSOR_DATA_PIN

//=====================================================================================
//  GPIO 操作抽象接口 (GPIO Operation Macros)
//=====================================================================================
#define GRAYSCALE_PIN_WRITE(port, pin, state) do { \
    if(state) DL_GPIO_setPins(port, pin); \
    else DL_GPIO_clearPins(port, pin); \
} while(0)

#define SENSOR_AD0_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD0_PORT, SENSOR_AD0_PIN, state)
#define SENSOR_AD1_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD1_PORT, SENSOR_AD1_PIN, state)
#define SENSOR_AD2_WRITE(state)  GRAYSCALE_PIN_WRITE(SENSOR_AD2_PORT, SENSOR_AD2_PIN, state)

#define SENSOR_OUT_READ()        (!!(DL_GPIO_readPins(GrayS_OUT_PORT, GrayS_OUT_PIN)))

//=====================================================================================
//  驱动函数接口 (Driver API)
//=====================================================================================

#define GRAYSCALE_SENSOR_CHANNELS   8   // 传感器通道总数 Number of sensor channels

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif // __GRAYSCALE_SENSOR_H
