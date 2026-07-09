#include <stdio.h>
#include <stdlib.h>
#include "MPU6050.h"
#include "OLED.h"

/*三段式状态机状态编码*/
typedef enum{
    STATE_IDLE,
    STATE_INPUT,
    STATE_RUNNING,
    STATE_COLLIMATION,
    STATE_COLLIMATION_INPUT,
    STATE_RUNNING_COLLIMATION,
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
    int number_of_circles = 0;//设定的圈数
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
while(key.key2==1){
    if(key.key1==0){
        number_of_circles++;
        if(number_of_circles>5){
            number_of_circles=0;
        }
    }
}
if(key.key2==0){
    next_state=STATE_RUNNING;
}
break;



case STATE_COLLIMATION_INPUT:
while(key.key2==1){
    if(key.key1==0){
        number_of_circles++;
        if(number_of_circles>5){
            number_of_circles=0;
        }
    }
}
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
return 0;
}



void station_action(){
switch(current_state){
case STATE_IDLE:
number_of_circles=0;
true_number_of_circles=0;
break;


case STATE_INPUT:




case STATE_RUNNING:




case STATE_COLLIMATION:




case STATE_COLLIMATION_INPUT:




case STATE_RUNNING_COLLIMATION:


case STATE_STOP:
number_of_circles=0;
true_number_of_circles=0;


    return 0;
}

void main(){



return 0;
}