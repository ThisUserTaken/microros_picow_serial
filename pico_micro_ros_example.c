#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;
#define ADC_TEMPERATURE_SENSOR 4 
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    uint16_t raw=adc_read();
    const float conversion_factor = 3.3f / (1 << 12); // 12-bit ADC
    float voltage = raw * conversion_factor;
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;
    msg.data=temperature;
 //   const float conversion_factor = 3.3f / (1 << 12);
 //   uint16_t result = adc_read();
 //   msg.data=result;
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

    stdio_init_all();
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(ADC_TEMPERATURE_SENSOR);
 //   adc_init();
 //   adc_select_input(4);
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
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
