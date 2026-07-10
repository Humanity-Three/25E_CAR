#include "MPU6050.h"
#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "control.h"
#include <math.h>

// 定义原始数据变量
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t temp_raw;

// 定义实际物理量
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temp;

// 姿态角
float Yaw = 0;
float Pitch = 0;
float Roll = 0;

// 互补滤波参数
#define GYRO_SCALE 131.0f      // 陀螺仪量程±250°/s
#define ACCEL_SCALE 16384.0f   // 加速度计量程±2g
#define DT 0.01f               // 采样周期 10ms
#define ALPHA 0.98f            // 互补滤波系数

// 陀螺仪零漂校准参数
#define GYRO_CALIB_SAMPLES 200 // 校准采样次数
static float gyro_x_offset = 0;
static float gyro_y_offset = 0;
static float gyro_z_offset = 0;

// 通过加速度计计算的角度
float accel_pitch, accel_roll;

// 中断保护宏（注释掉以避免长时间关中断导致主循环饿死）
// #define I2C_ENTER_CRITICAL()    __asm(" CPSID I ")
// #define I2C_EXIT_CRITICAL()     __asm(" CPSIE I ")
#define I2C_ENTER_CRITICAL()
#define I2C_EXIT_CRITICAL()

//=====================================================================
// 硬件 I2C1 通信（根据 sysconfig：I2C1, SDA=PA10, SCL=PA11）
//=====================================================================

/**
  * 函    数：硬件I2C阻塞发送
  * 参    数：inst I2C实例
  * 参    数：devAddr 从机地址（7位）
  * 参    数：data 数据缓冲区
  * 参    数：len 数据长度
  * 返 回 值：无
  * 说    明：使用硬件I2C发送数据，自动处理FIFO补充
  */
static void I2C_TransmitBlocking(I2C_Regs *inst, uint8_t devAddr, uint8_t *data, uint16_t len)
{
    uint16_t remaining = len;
    uint16_t idx = 0;
    uint16_t chunk;

    DL_I2C_resetControllerTransfer(inst);

    chunk = (remaining > 8) ? 8 : remaining;
    DL_I2C_fillControllerTXFIFO(inst, &data[idx], chunk);
    idx += chunk;
    remaining -= chunk;

    DL_I2C_startControllerTransfer(inst, devAddr, DL_I2C_CONTROLLER_DIRECTION_TX, len);

    while (remaining > 0) {
        while (DL_I2C_isControllerTXFIFOFull(inst));
        chunk = (remaining > 8) ? 8 : remaining;
        DL_I2C_fillControllerTXFIFO(inst, &data[idx], chunk);
        idx += chunk;
        remaining -= chunk;
    }
    while (DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_BUSY);
}

/**
  * 函    数：硬件I2C阻塞读取（先写寄存器地址，再重复起始+读）
  * 参    数：inst I2C实例
  * 参    数：devAddr 从机地址（7位）
  * 参    数：regaddr 寄存器地址
  * 参    数：rxBuf 接收缓冲区
  * 参    数：rxLen 要读取的字节数
  * 返 回 值：0=成功
  */
static uint8_t I2C_ReadRegBlocking(I2C_Regs *inst, uint8_t devAddr, uint8_t regaddr, uint8_t *rxBuf, uint8_t rxLen)
{
    uint8_t i;

    DL_I2C_resetControllerTransfer(inst);

    /* Step 1: 发送寄存器地址（带START，不带STOP，准备重复起始） */
    DL_I2C_fillControllerTXFIFO(inst, &regaddr, 1);
    DL_I2C_startControllerTransferAdvanced(inst, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);
    while (DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_BUSY);

    /* 检查错误（如NACK） */
    if (DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return 1;
    }

    /* Step 2: 重复起始 + 读取数据（自动STOP） */
    DL_I2C_startControllerTransfer(inst, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_RX, rxLen);
    while (DL_I2C_getControllerStatus(inst) & DL_I2C_CONTROLLER_STATUS_BUSY);

    /* 从RX FIFO读取数据 */
    for (i = 0; i < rxLen; i++) {
        rxBuf[i] = DL_I2C_receiveControllerData(inst);
    }

    return 0;
}

//=====================================================================
// MPU6050 I2C 读写（硬件I2C1）
//=====================================================================

static uint8_t MPU6050_WriteReg(uint8_t addr, uint8_t regaddr, uint8_t num, uint8_t *regdata)
{
    uint16_t i;
    uint8_t buf[256];   /* 寄存器地址 + 数据 */
    uint16_t len = 1 + num;

    buf[0] = regaddr;
    for (i = 0; i < num; i++) {
        buf[1 + i] = regdata[i];
    }

    I2C_ENTER_CRITICAL();
    I2C_TransmitBlocking(MPU_6050_I2C_INST, addr, buf, len);
    I2C_EXIT_CRITICAL();

    return 0;
}

static uint8_t MPU6050_ReadData(uint8_t addr, uint8_t regaddr, uint8_t num, uint8_t *Read)
{
    uint8_t ret;

    I2C_ENTER_CRITICAL();
    ret = I2C_ReadRegBlocking(MPU_6050_I2C_INST, addr, regaddr, Read, num);
    I2C_EXIT_CRITICAL();

    return ret;
}

void MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    MPU6050_WriteReg(MPU6050_ADDR, reg, 1, &data);
}

uint8_t MPU6050_Read_Reg(uint8_t reg)
{
    uint8_t val = 0;
    MPU6050_ReadData(MPU6050_ADDR, reg, 1, &val);
    return val;
}

void MPU6050_Read_Burst(uint8_t reg, uint8_t *data, uint8_t len)
{
    MPU6050_ReadData(MPU6050_ADDR, reg, len, data);
}

static uint8_t MPU6050_Read_Burst_Retry(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t retry;
    for(retry = 0; retry < 3; retry++)
    {
        if(MPU6050_ReadData(MPU6050_ADDR, reg, len, data) == 0)
            return 0;  // 成功
    }
    return 1;  // 失败
}

//=====================================================================
// 陀螺仪零漂校准
//=====================================================================

void MPU6050_Calibrate_Gyro(void)
{
    int i;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t raw_x, raw_y, raw_z;
    uint8_t buf[6];

    for (i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        MPU6050_Read_Burst(MPU6050_GYRO_XOUT_H, buf, 6);
        raw_x = (buf[0] << 8) | buf[1];
        raw_y = (buf[2] << 8) | buf[3];
        raw_z = (buf[4] << 8) | buf[5];
        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;
        Delay_ms(5);
    }

    gyro_x_offset = (float)(sum_x / GYRO_CALIB_SAMPLES) / GYRO_SCALE;
    gyro_y_offset = (float)(sum_y / GYRO_CALIB_SAMPLES) / GYRO_SCALE;
    gyro_z_offset = (float)(sum_z / GYRO_CALIB_SAMPLES) / GYRO_SCALE;
}

//=====================================================================
// 初始化
//=====================================================================

uint8_t MPU6050_Init(void)
{
    uint8_t whoami;
    uint8_t reg_val;

    // 硬件 I2C1 的 GPIO 由 SYSCFG_DL_GPIO_init() 配置，无需手动初始化
    Delay_ms(10);

    // 复位 MPU6050
    reg_val = 0x80;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &reg_val);
    Delay_ms(100);

    // 唤醒：选择时钟源为 X 轴陀螺仪
    reg_val = 0x01;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &reg_val);
    Delay_ms(10);

    // 验证通信
    whoami = MPU6050_Read_Reg(MPU6050_WHO_AM_I);
    if (whoami != 0x68 && whoami != 0x69) {
        return 0;
    }

    // 配置
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &reg_val);
    reg_val = 0x03;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_CONFIG, 1, &reg_val);
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &reg_val);
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &reg_val);
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_2, 1, &reg_val);

    Delay_ms(100);

    // 零漂校准
    MPU6050_Calibrate_Gyro();

    return 1;
}

//=====================================================================
// 数据读取
//=====================================================================

void MPU6050_Read_Accel(void)
{
    uint8_t buf[6];
    MPU6050_Read_Burst(MPU6050_ACCEL_XOUT_H, buf, 6);

    accel_x_raw = (buf[0] << 8) | buf[1];
    accel_y_raw = (buf[2] << 8) | buf[3];
    accel_z_raw = (buf[4] << 8) | buf[5];

    accel_x = (float)accel_x_raw / ACCEL_SCALE;
    accel_y = (float)accel_y_raw / ACCEL_SCALE;
    accel_z = (float)accel_z_raw / ACCEL_SCALE;
}

void MPU6050_Read_Gyro(void)
{
    uint8_t buf[6];
    MPU6050_Read_Burst(MPU6050_GYRO_XOUT_H, buf, 6);

    gyro_x_raw = (buf[0] << 8) | buf[1];
    gyro_y_raw = (buf[2] << 8) | buf[3];
    gyro_z_raw = (buf[4] << 8) | buf[5];

    gyro_x = (float)gyro_x_raw / GYRO_SCALE - gyro_x_offset;
    gyro_y = (float)gyro_y_raw / GYRO_SCALE - gyro_y_offset;
    gyro_z = (float)gyro_z_raw / GYRO_SCALE - gyro_z_offset;
}

void MPU6050_Read_Data(void)
{
    uint8_t buf[14] = {0};
    MPU6050_Read_Burst_Retry(MPU6050_ACCEL_XOUT_H, buf, 14);  //带重试的I2C读取

    accel_x_raw = (buf[0] << 8) | buf[1];
    accel_y_raw = (buf[2] << 8) | buf[3];
    accel_z_raw = (buf[4] << 8) | buf[5];

    temp_raw = (buf[6] << 8) | buf[7];
    temp = (float)temp_raw / 340.0f + 36.53f;

    gyro_x_raw = (buf[8] << 8) | buf[9];
    gyro_y_raw = (buf[10] << 8) | buf[11];
    gyro_z_raw = (buf[12] << 8) | buf[13];

    accel_x = (float)accel_x_raw / ACCEL_SCALE;
    accel_y = (float)accel_y_raw / ACCEL_SCALE;
    accel_z = (float)accel_z_raw / ACCEL_SCALE;

    gyro_x = (float)gyro_x_raw / GYRO_SCALE - gyro_x_offset;
    gyro_y = (float)gyro_y_raw / GYRO_SCALE - gyro_y_offset;
    gyro_z = (float)gyro_z_raw / GYRO_SCALE - gyro_z_offset;
}

//=====================================================================
// 姿态角计算（互补滤波）
//=====================================================================

void MPU6050_Get_Angle(void)
{
    static uint32_t last_tick = 0;
    uint32_t current_tick = g_sys_tick_10ms;
    float dt = (float)(current_tick - last_tick) * 0.01f;  // 实际时间间隔（秒）
    if(dt <= 0.0f || dt > 0.1f) dt = 0.01f;                // 合理性检查
    last_tick = current_tick;

    accel_pitch = atan2(accel_y, accel_z) * 180.0f / 3.1415926f;
    accel_roll = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / 3.1415926f;

    Pitch = ALPHA * (Pitch + gyro_y * dt) + (1.0f - ALPHA) * accel_pitch;
    Roll  = ALPHA * (Roll  - gyro_x * dt) + (1.0f - ALPHA) * accel_roll;

    // Yaw 积分 + 漂移抑制：静止时缓慢衰减漂移，转动时正常积分
    float delta = gyro_z * dt;
    if(delta > 2.0f)  delta = 2.0f;    // 限幅：单次变化不超过 2°，防止 I2C 错误突变
    if(delta < -2.0f) delta = -2.0f;

    if(fabsf(gyro_z) > 0.1f)
        Yaw += delta;             // 正在转动，正常积分
    else
        Yaw *= 0.998f;            // 静止时缓慢拉回零，抵消漂移

    if (Yaw > 180.0f)      Yaw -= 360.0f;
    else if (Yaw < -180.0f) Yaw += 360.0f;
}
