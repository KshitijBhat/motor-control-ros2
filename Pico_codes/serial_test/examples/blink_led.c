#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#define byte uint8_t
#define numChars 32
char receivedChars[numChars];


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

    //Main Loop 
    while(1){
        //Get User Input
        int motorCommand;
        motorCommand = getMotorCommand();
        // if(motorCommand == 1234)
        // {
        //     gpio_put(25, 0);
        //     sleep_ms(1000);
        //     gpio_put(25, 1);
        //     sleep_ms(1000);
        //     gpio_put(25, 0);
        //     sleep_ms(1);
        //     gpio_put(25, 1);
        //     sleep_ms(1000);
        //     gpio_put(25, 0);
        //     sleep_ms(1000);
        //     gpio_put(25, 1);
        //     sleep_ms(1000);
        //     gpio_put(25, 0);
        //     sleep_ms(1000);
        //     gpio_put(25, 1);
        //     sleep_ms(1000);
        // }
        if(motorCommand == 123)
        {
            gpio_put(25, 0);
            sleep_ms(1000);
            gpio_put(25, 1);
            sleep_ms(1000);
            gpio_put(25, 0);
            sleep_ms(1);
            gpio_put(25, 1);
            sleep_ms(1000);
            gpio_put(25, 0);
            sleep_ms(1000);
            gpio_put(25, 1);
            sleep_ms(1000);
            gpio_put(25, 0);
            sleep_ms(1000);
            gpio_put(25, 1);
            sleep_ms(1000);
        }
    }
}