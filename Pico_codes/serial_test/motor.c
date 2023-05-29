#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"


#define byte uint8_t
#define numChars 32
char receivedChars[numChars];

#define ENA 15
#define IN1 14
#define IN2 13

const uint SPEED_FACTOR = 1;

void command_motor(int speed)
{
    if(speed>0){
        gpio_put(IN1, 0);  //spin forward
        gpio_put(IN2, 1);
    }
    else if(speed<0){
        gpio_put(IN1, 1);  //spin forward
        gpio_put(IN2, 0);
    }
    else{
        gpio_put(IN1, 0);  //stop
        gpio_put(IN2, 0);
    }
    int pwm_level = (int)abs(speed)*SPEED_FACTOR;
    uint slice_num = pwm_gpio_to_slice_num(ENA);
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, pwm_level);
    pwm_set_enabled(slice_num, true);
}

int getMotorCommand()
{
    char rc;
    int motorCommand = 0;
    rc = getchar();
    char endMarker = '.';
    static byte ndx = 0;
    if (rc != endMarker) 
    {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
        gpio_put(25, 1);
    }
    else {
        ndx = 0;
        gpio_put(25, 0);
        motorCommand = atoi(receivedChars);
    }
    return motorCommand;
}

int main(){
    //Initialise I/O
    stdio_init_all(); 

    // Initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);

    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);

    gpio_set_function(ENA, GPIO_FUNC_PWM);

    //Main Loop 
    while(1){
        //Get User Input
        int motorCommandspeed;
        motorCommandspeed = getMotorCommand();
        command_motor(motorCommandspeed);
    }
}