// Headless ESP32-WROOM spray valve controller.
//
// Talks to ROS 2 as a micro-ROS node over WiFi/UDP and drives a GPIO pin to
// open/close a solenoid valve for a commanded duration. Status is logged over
// USB serial; no display is required.
//
// Subscribes:
//   /status/can_bus_healthy   (std_msgs/Bool)     - status monitoring
//   /status/controller_active (std_msgs/Bool)     - status monitoring
//   /spray/valve_command      (std_msgs/Float32)  - duration in seconds to open the valve;
//                                                    <= 0 closes it immediately
// Publishes:
//   /spray/valve_state        (std_msgs/Bool)     - current valve open/closed state
//
#include <Arduino.h>
#include <cmath>
#include <stdint.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include "config.h"

#define RCCHECK(fn)                                                                             \
  {                                                                                              \
    rcl_ret_t rc = fn;                                                                          \
    if (rc != RCL_RET_OK) {                                                                     \
      error_loop();                                                                             \
    }                                                                                            \
  }
#define RCSOFTCHECK(fn)                                                                          \
  {                                                                                               \
    rcl_ret_t rc = fn;                                                                           \
    (void)rc;                                                                                    \
  }
#define EXECUTE_EVERY_N_MS(MS, X)                                                                 \
  do {                                                                                            \
    static unsigned long init = 0;                                                                \
    if (millis() - init > (MS)) {                                                                 \
      X;                                                                                          \
      init = millis();                                                                            \
    }                                                                                              \
  } while (0)

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState g_state = WAITING_AGENT;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t control_timer;

static rcl_subscription_t can_bus_sub;
static rcl_subscription_t controller_active_sub;
static rcl_subscription_t valve_cmd_sub;
static rcl_publisher_t valve_state_pub;

static std_msgs__msg__Bool can_bus_msg;
static std_msgs__msg__Bool controller_active_msg;
static std_msgs__msg__Float32 valve_cmd_msg;
static std_msgs__msg__Bool valve_state_msg;

static bool can_bus_healthy = false;
static bool controller_active = false;
static bool valve_open = false;
static uint32_t valve_opened_at_ms = 0;
static uint32_t valve_open_duration_ms = 0;

static void set_valve(bool open) {
  if (valve_open == open) {
    return;
  }

  valve_open = open;
  if (!open) {
    valve_open_duration_ms = 0;
  }

  const bool pin_high = VALVE_ACTIVE_HIGH ? open : !open;
  digitalWrite(VALVE_RELAY_PIN, pin_high ? HIGH : LOW);
  Serial.println(open ? "Valve command: OPEN" : "Valve command: CLOSED");
}

static void error_loop() {
  set_valve(false);
  while (true) {
    Serial.println("micro-ROS error - check agent connection/wiring");
    delay(1000);
  }
}

static void check_valve_timeout() {
  if (valve_open && valve_open_duration_ms != 0 &&
      static_cast<uint32_t>(millis() - valve_opened_at_ms) >= valve_open_duration_ms) {
    set_valve(false);
  }
}

static void can_bus_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  can_bus_healthy = msg->data;
  Serial.println(can_bus_healthy ? "CAN bus: healthy" : "CAN bus: fault");
}

static void controller_active_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  controller_active = msg->data;
  Serial.println(controller_active ? "Controller: active" : "Controller: inactive");
}

static void valve_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  const float duration_sec = msg->data;
  const double duration_ms = static_cast<double>(duration_sec) * 1000.0;
  if (std::isfinite(duration_sec) && duration_sec > 0.0f &&
      duration_ms <= static_cast<double>(UINT32_MAX)) {
    valve_open_duration_ms = static_cast<uint32_t>(duration_ms);
    if (valve_open_duration_ms == 0) {
      valve_open_duration_ms = 1;
    }
    valve_opened_at_ms = millis();
    set_valve(true);
  } else {
    if (!std::isfinite(duration_sec) || duration_sec > 0.0f) {
      Serial.println("Rejected invalid valve duration");
    }
    set_valve(false);
  }
}

static void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }

  valve_state_msg.data = valve_open;
  RCSOFTCHECK(rcl_publish(&valve_state_pub, &valve_state_msg, NULL));
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 5));

  RCCHECK(rclc_support_init_with_options(
      &support,
      0,
      NULL,
      &init_options,
      &allocator));

  RCCHECK(rcl_init_options_fini(&init_options));

  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NODE_NAMESPACE, &support));

  RCCHECK(rclc_subscription_init_default(&can_bus_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_CAN_BUS_HEALTHY));
  RCCHECK(rclc_subscription_init_default(&controller_active_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_CONTROLLER_ACTIVE));
  RCCHECK(rclc_subscription_init_default(&valve_cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TOPIC_VALVE_COMMAND));
  RCCHECK(rclc_publisher_init_default(&valve_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_VALVE_STATE));

  const unsigned int timer_timeout_ms = 100;
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(timer_timeout_ms),
      control_timer_callback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &can_bus_sub, &can_bus_msg,
      &can_bus_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &controller_active_sub,
      &controller_active_msg, &controller_active_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &valve_cmd_sub, &valve_cmd_msg,
      &valve_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&can_bus_sub, &node);
  rcl_subscription_fini(&controller_active_sub, &node);
  rcl_subscription_fini(&valve_cmd_sub, &node);
  rcl_publisher_fini(&valve_state_pub, &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);

  const bool safe_pin_high = VALVE_ACTIVE_HIGH ? false : true;
  digitalWrite(VALVE_RELAY_PIN, safe_pin_high ? HIGH : LOW);
  pinMode(VALVE_RELAY_PIN, OUTPUT);
  set_valve(false);

  Serial.println("ESP32 spray controller starting");

  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASSWORD, agent_ip, AGENT_PORT);

}

void loop() {
  switch (g_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, g_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                             ? AGENT_AVAILABLE
                                             : WAITING_AGENT);
      break;

    case AGENT_AVAILABLE:
      g_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (g_state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1000, g_state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1))
                                             ? AGENT_CONNECTED
                                             : AGENT_DISCONNECTED);
      if (g_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;

    case AGENT_DISCONNECTED:
      set_valve(false);
      destroy_entities();
      g_state = WAITING_AGENT;
      break;
  }

  check_valve_timeout();

  static AgentState last_logged_state = AGENT_DISCONNECTED;
  static bool have_logged_state = false;
  if (!have_logged_state || g_state != last_logged_state) {
    switch (g_state) {
      case WAITING_AGENT:
        Serial.println("micro-ROS agent: waiting");
        break;
      case AGENT_AVAILABLE:
        Serial.println("micro-ROS agent: available; creating entities");
        break;
      case AGENT_CONNECTED:
        Serial.println("micro-ROS agent: connected");
        break;
      case AGENT_DISCONNECTED:
        Serial.println("micro-ROS agent: disconnected");
        break;
    }
    last_logged_state = g_state;
    have_logged_state = true;
  }
}
