#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pwm.h"


const uint LED_PIN = 25;
const uint ENA = 15;
const uint IN1 = 14;
const uint IN2 = 13;
const uint encoderA = 16;
const uint encoderB = 17;

const uint SPEED_FACTOR = 1;
int encoder_count = 0; 
int aState;
int aLastState;  
int pwm_level = 50;


rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 motor_msg;
std_msgs__msg__Int32 encoder_msg;


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

void counter()
{
    aState = gpio_get(encoderA); // Reads the "current" state of the encoderA
    // If the previous and the current state of the encoderA are different, that means a Pulse has occured
    if (aState != aLastState){     
        // If the encoderB state is different to the encoderA state, that means the encoder is rotating clockwise
        if (gpio_get(encoderB) != aState) { 
        encoder_count ++;
        } 
        else {
        encoder_count --;
        }
        // Serial.print("Position: ");
        // Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the encoderA with the current state
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &encoder_msg, NULL);
    counter();
    encoder_msg.data = encoder_count; ////////////////////////////////////////////// encoder count
}

void motor_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * motor_msg = (const std_msgs__msg__Int32 *)msgin;
    pwm_level = motor_msg->data;
    command_motor(pwm_level);
    gpio_put(LED_PIN, 1);
    sleep_ms(1000); 
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // Setup all the pins

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);

    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);

    gpio_set_function(ENA, GPIO_FUNC_PWM);
    
    gpio_init(encoderA);
    gpio_set_dir(encoderA, GPIO_IN);

    gpio_init(encoderB);
    gpio_set_dir(encoderB, GPIO_IN);

    aLastState = gpio_get(encoderA);
   

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    // create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node 
    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    // create publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "joint_state");

    // create subscriber
    rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"motor_command");

    // create timer
    rcl_timer_t timer;
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1),
        timer_callback);

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &motor_msg, &motor_callback, ON_NEW_DATA);

    encoder_msg.data = 0; //default

    rclc_executor_spin(&executor);

    rcl_subscription_fini(&subscriber, &node);
    rcl_publisher_fini(&publisher, &node);
	rcl_node_fini(&node);
    return 0;
}

//cd ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/build && cmake .. && make
//sudo snap connect micro-ros-agent:serial-port snapd:pico && sudo micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200