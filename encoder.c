#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;
const uint encoderA = 16;
const uint encoderB = 17;


int count = 0; 
int aState;
int aLastState;  


rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void counter()
{
    aState = gpio_get(encoderA); // Reads the "current" state of the encoderA
    // If the previous and the current state of the encoderA are different, that means a Pulse has occured
    if (aState != aLastState){     
        // If the encoderB state is different to the encoderA state, that means the encoder is rotating clockwise
        if (gpio_get(encoderB) != aState) { 
        count ++;
        } 
        else {
        count --;
        }
        // Serial.print("Position: ");
        // Serial.println(counter);
   } 
   aLastState = aState; // Updates the previous state of the encoderA with the current state
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    counter();
    msg.data = count; ////////////////////////////////////////////// encoder count
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

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(encoderA);
    gpio_set_dir(encoderA, GPIO_IN);
    gpio_init(encoderB);
    gpio_set_dir(encoderB, GPIO_IN);

    aLastState = gpio_get(encoderA);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "joint_state");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);
    

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

//cd ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk/build && cmake .. && make
//sudo snap connect micro-ros-agent:serial-port snapd:pico && sudo micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200