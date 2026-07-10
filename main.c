#include <stdio.h>
#include <stdlib.h>
#include "MPU6050.h"
#include "OLED.h"
#include "Interrupt.h"
#include "ti_msp_dl_config.h”







 NVIC_EnableIRQ(TIMG7_IRQn);     // 10ms 定时器
    NVIC_EnableIRQ(TIMG6_IRQn);     // 50ms 定时器

    DL_TimerG_startTimer(TIMER_0_INST);            // 启动 10ms 定时器
    DL_TimerG_startTimer(MOTOR_ENCODER_READ_INST); // 启动 50ms 定时器


    OLED_Init();
    MPU6050_Init();


/*三段式状态机状态编码*/
typedef enum{
    STATE_IDLE,//待机
    STATE_INPUT,//等待输入
    STATE_RUNNING,//循迹模式
    STATE_COLLIMATION,//打靶模式
    STATE_COLLIMATION_INPUT,//自行瞄准等待输入
    STATE_RUNNING_COLLIMATION,//自行瞄准模式
    STATE_STOP
} car_STATE;

static car_STATE current_state = STATE_IDLE;
static car_STATE next_state = STATE_IDLE;
static int number_of_circles = 0;//设定的圈数
static int true_number_of_circles = 0;//实际的圈数

typedef struct{
    uint8_t key1;
    uint8_t key2;
}car_KEY;

void move_to_next_state(car_KEY key){
    next_state = current_state;
    number_of_circles = 0;//设定的圈数
    switch(current_state){
case STATE_IDLE:
number_of_circles= 0;
true_number_of_circles=0;
if(key.key1 == 0){
    next_state = STATE_INPUT;
}else if(key.key2 == 0){
    next_state = STATE_COLLIMATION;
}else if(key.key1 == 0 && key.key2 == 0){
    next_state = STATE_COLLIMATION_INPUT;
}else{
     next_state = STATE_IDLE;
}
break;



case STATE_INPUT:
/*while(key.key2==1){
    if(key.key1==0){
        number_of_circles++;
        if(number_of_circles>5){
            number_of_circles=0;
        }
    }
}*/
if(key.key2==0){
    next_state=STATE_RUNNING;
}
break;




case STATE_COLLIMATION_INPUT:
/*while(key.key2==1){
    if(key.key1==0){
        number_of_circles++;
        if(number_of_circles>5){
            number_of_circles=0;
        }
    }
}*/
if(key.key2==0){
    next_state=STATE_RUNNING_COLLIMATION;
}
break;




case STATE_RUNNING:
if (true_number_of_circles == number_of_circles){
    next_state = STATE_STOP;
}
break;




case STATE_COLLIMATION:
if(key.key2==0){
    next_state=STATE_STOP;
}
break;




case STATE_RUNNING_COLLIMATION:
if (true_number_of_circles == number_of_circles){
    next_state = STATE_STOP;
}break;

case STATE_STOP:
number_of_circles=0;
true_number_of_circles=0;
next_state=STATE_IDLE;
break;

    }
}





void station_action(car_KEY key)
{
switch(current_state){

case STATE_IDLE:
number_of_circles=0;//目标圈数归零
true_number_of_circles=0;//实际圈数归零
OLED_Clear();
OLED_ShowString(0,0,"IDLE");
break;




case STATE_INPUT:
OLED_Clear();
OLED_ShowString(0,0,"INPUT");
while(key.key2==1){
    if(key.key1==0){
        number_of_circles++;
        if(number_of_circles>5){
            number_of_circles=0;
        }
    }
}OLED_ShowString(0,1,"Circles:");
OLED_ShowInt(1, 1, number_of_circles, 1);
break;








case STATE_RUNNING:
OLED_Clear();
OLED_ShowString(0,0,"RUNNING");
OLED_ShowString(0,1,"Circles:");
OLED_ShowInt(1, 1, true_number_of_circles, 1);
break;



case STATE_COLLIMATION:
OLED_Clear();
OLED_ShowString(0,0,"COLIMATION");
break;



case STATE_COLLIMATION_INPUT:
OLED_Clear();
OLED_ShowString(0,0,"COLIMATION_INPUT");
while(key.key2==1){
    if(key.key1==0){
        number_of_circles++;
        if(number_of_circles>5){
            number_of_circles=0;
        }
    }
}
OLED_ShowString(0,1,"Circles:");
OLED_ShowNum(1, 1, number_of_circles, 1);

break;



case STATE_RUNNING_COLLIMATION:
OLED_Clear();
OLED_ShowString(0,0,"RUNNING_COLLIMATION");
break;



case STATE_STOP:
number_of_circles=0;
true_number_of_circles=0;
break;
    }
}

void main()
{

    car_KEY key;
    while(1){

        key.key1 = DL_GPIO_readPins(GPIO_B,6);
        key.key2 = DL_GPIO_readPins(GPIO_B,7);
        move_to_next_state(key);
        station_action(key);
    }


}