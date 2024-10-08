#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "quadrature.pio.h"


#define byte uint8_t
#define numChars 32
char receivedChars[numChars];

// #define ENA 15
// #define IN1 14
// #define IN2 13
// #define QUADRATURE_A_PIN 16
// #define QUADRATURE_B_PIN 17

#define ENB 9
#define IN3 8
#define IN4 7
#define QUADRATURE2_A_PIN 18
#define QUADRATURE2_B_PIN 19

long encoder_count = 0;  
int pwm_level = 0;
PIO pio = pio0;
uint sm = 0;
int motorCommand = 0;

void counter()
{   
    // counter
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
    encoder_count = pio_sm_get_blocking(pio, sm);
}

void command_motor(int speed)
{
    int pwm_level;
    if(speed>0){
        gpio_put(IN3, 0);  //spin forward
        gpio_put(IN4, 1);
        pwm_level = (int)abs(speed);
    }
    else if(speed<0){
        gpio_put(IN3, 1);  //spin forward
        gpio_put(IN4, 0);
        pwm_level = (int)abs(speed);
    }
    else{
        gpio_put(IN3, 0);  //stop
        gpio_put(IN4, 0);
        pwm_level = 0;
    }
    
    uint slice_num = pwm_gpio_to_slice_num(ENB);
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, pwm_level);
    pwm_set_enabled(slice_num, true);
}

int getInput()
{
    char rc;

    rc = getchar();
    char endMarker = '.';
    static byte ndx = 0;
    if (rc == '\r')
    {
        counter();
        printf("%d\n",encoder_count);
    }
    else if (rc == endMarker) 
    {
        ndx = 0;
        gpio_put(25, 0);
        motorCommand = atoi(receivedChars);
    }
    else
    {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
    }
    
    return motorCommand;
}

int main(){
    //Initialise I/O
    stdio_init_all(); 

    gpio_init(IN3);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4);
    gpio_set_dir(IN4, GPIO_OUT);
    gpio_set_function(ENB, GPIO_FUNC_PWM);

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &quadrature_program);
    uint sm = pio_claim_unused_sm(pio, true);
    quadrature_program_init(pio, sm, offset, QUADRATURE2_A_PIN, QUADRATURE2_B_PIN);


    int motorCommandspeed;
    char ping;
    //Main Loop 
    while(1){

        //Get User Input
        motorCommandspeed = getInput();
        command_motor(motorCommandspeed);
        
    }
}