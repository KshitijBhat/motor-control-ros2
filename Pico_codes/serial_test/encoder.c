#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "quadrature.pio.h"

#define QUADRATURE_A_PIN 16
#define QUADRATURE_B_PIN 17

long encoder_count = 0;  
int pwm_level = 0;
PIO pio = pio0;
uint sm = 0;

void counter()
{   
    // counter
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
    encoder_count = pio_sm_get_blocking(pio, sm);
}


int main(){
    //Initialise I/O
    stdio_init_all(); 

    // Initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    stdio_init_all();
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &quadrature_program);
    uint sm = pio_claim_unused_sm(pio, true);
    quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

    char ping;

    //Main Loop 
    while(1){
        // printf("Give 1 to get encoder value:\n");
        ping = getchar();
        if (ping == 'e')
        {
            counter();
            printf("%d\n",encoder_count);
        }
               
    }
}