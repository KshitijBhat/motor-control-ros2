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

#define ENA 15
#define IN1 14
#define IN2 13
#define QUADRATURE1_A_PIN 16
#define QUADRATURE1_B_PIN 17

#define ENB 9
#define IN3 8
#define IN4 7
#define QUADRATURE2_A_PIN 18
#define QUADRATURE2_B_PIN 19

#define Imax 128

long encoder_count_d = 0;  
long encoder_count_s = 0;  
PIO pio = pio0;
uint sm1 = 0;
uint sm2 = 1;

// PID Params
double Kp = 0.5;
double Ki = 0.0;
double Kd = 0.0;
long errorIntegral;
long prev_error;
bool setpid = false;


void counter(uint sm1, uint sm2)
{   
    // counter
    pio_sm_exec_wait_blocking(pio, sm1, pio_encode_in(pio_x, 32));
    encoder_count_s = pio_sm_get_blocking(pio, sm1);

    pio_sm_exec_wait_blocking(pio, sm2, pio_encode_in(pio_x, 32));
    encoder_count_d = pio_sm_get_blocking(pio, sm2);
}

void command_steering_motor(int speed)
{
    int pwm_level;
    if(speed>255){
        speed = 255;
    }
    else if(speed<-255){
        speed = -255;
    }
    if(speed>0){
        gpio_put(IN1, 0);  //spin forward
        gpio_put(IN2, 1);
        pwm_level = (int)abs(speed);
    }
    else if(speed<0){
        gpio_put(IN1, 1);  //spin forward
        gpio_put(IN2, 0);
        pwm_level = (int)abs(speed);
    }
    else{
        gpio_put(IN1, 0);  //stop
        gpio_put(IN2, 0);
        pwm_level = 0;
    }
    
    uint slice_num = pwm_gpio_to_slice_num(ENA);
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, pwm_level);
    pwm_set_enabled(slice_num, true);
}

void command_driving_motor(int speed)
{
    int pwm_level;
    if(speed>255){
        speed = 255;
    }
    else if(speed<-255){
        speed = -255;
    }
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

void getTargets(int *steerPosition, int *driveVelocity)
{
    char rc;
    rc = getchar();
    char endMarker = '.';
    static byte ndx = 0;
    static byte pidx = 0;
    if (rc == '\t')
    {
        counter(sm1, sm2);
        printf("%d\t%d\n",encoder_count_s, encoder_count_d);
    }
    if (rc == '\r')
    {
        setpid = true;
    }
    if (rc == endMarker && !setpid) 
    {
        ndx = 0;
        char delim[] = " ";
        char *ptr = strtok(receivedChars, delim);
        *steerPosition = atoi(ptr);
        ptr = strtok(NULL, delim);
        *driveVelocity = atoi(ptr);
    }
    if (rc != endMarker && !setpid)
    {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
    }
    if (setpid && rc != ';'){
        receivedChars[pidx] = rc;
        pidx++;
        if (pidx >= numChars) {
            pidx = numChars - 1;
        }
    }
    if (setpid && rc == ';'){
        pidx = 0;
        setpid = false;
        char delim[] = " ";
        char *ptr = strtok(receivedChars, delim);
        Kp = atof(ptr);
        ptr = strtok(NULL, delim);
        Ki = atof(ptr); 
        ptr = strtok(NULL, delim);
        Kd = atof(ptr);       
    }
}

void getInput(int *steerPosition, int *driveVelocity)
{
    char rc;
    rc = getchar();
    char endMarker = '.';
    static byte ndx = 0;
    if (rc == '\r')
    {
        counter(sm1, sm2);
        printf("%d\t%d\n",encoder_count_s, encoder_count_d);
    }
    else if (rc == endMarker) 
    {
        ndx = 0;
        char delim[] = " ";
        char *ptr = strtok(receivedChars, delim);
        *steerPosition = atoi(ptr);
        ptr = strtok(NULL, delim);
        *driveVelocity = atoi(ptr);
    }
    else
    {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
    }
}

void controlPosition(double targetPosition)
{
    //https://vanhunteradams.com/Pico/ReactionWheel/Tuning.html
    
    counter(sm1, sm2);
    // Compute the error encoder_s is the current position
    int error = targetPosition - encoder_count_s;

    // Integrate the error
    errorIntegral += error;

    // Approximate the rate of change of the error
    int errorDerivative = (error - prev_error);

    // Clamp the integrated error (start with Imax = max_duty_cycle/2)
    if (errorIntegral>Imax) errorIntegral=Imax;
    if (errorIntegral<(-Imax)) errorIntegral=-Imax;

    double Command;

    Command = Kp*error + Ki*errorIntegral + Kd*errorDerivative;

    // Update prev_error
    prev_error = error;

    command_steering_motor((int)Command);
}

int main(){
    //Initialise I/O
    stdio_init_all(); 

    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_function(ENA, GPIO_FUNC_PWM);

    gpio_init(IN3);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4);
    gpio_set_dir(IN4, GPIO_OUT);
    gpio_set_function(ENB, GPIO_FUNC_PWM);

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &quadrature_program);
    // uint sm1 = pio_claim_unused_sm(pio, true);
    // uint sm2 = pio_claim_unused_sm(pio, true);
    quadrature_program_init(pio, sm1, offset, QUADRATURE1_A_PIN, QUADRATURE1_B_PIN);
    quadrature_program_init(pio, sm2, offset, QUADRATURE2_A_PIN, QUADRATURE2_B_PIN);


    int steerPos = 0;
    int driveVel = 0;
    //Main Loop 
    while(1){

        // //Get User Input
        // getInput(&motorCommandspeed1, &motorCommandspeed2);

        //Get Targets
        getTargets(&steerPos, &driveVel);
        controlPosition(steerPos);


        // command_steering_motor(motorCommandspeed1);
        // command_driving_motor(motorCommandspeed2);

        
    }
}