// CYD (ESP32-2432S028R) status display + spray valve controller.
//
// Talks to ROS 2 as a micro-ROS node over WiFi/UDP. Shows CAN bus health,
// controller-active state and valve state on the built-in TFT, and drives a
// GPIO pin to open/close a solenoid valve for a commanded duration.
//
// Subscribes:
//   /status/can_bus_healthy   (std_msgs/Bool)     - drives the CAN status tile
//   /status/controller_active (std_msgs/Bool)     - drives the controller status tile
//   /spray/valve_command      (std_msgs/Float32)  - duration in seconds to open the valve;
//                                                    <= 0 closes it immediately
// Publishes:
//   /spray/valve_state        (std_msgs/Bool)     - current valve open/closed state
//
// Touchscreen input is not wired up yet; reserved for a future manual-control mode.

#include <Arduino.h>
#include <TFT_eSPI.h>

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

static TFT_eSPI tft = TFT_eSPI();

static bool can_bus_healthy = false;
static bool controller_active = false;
static bool valve_open = false;
static unsigned long valve_close_at_ms = 0;
static bool display_dirty = true;

void error_loop() {
  while (true) {
    Serial.println("micro-ROS error - check agent connection/wiring");
    delay(1000);
  }
}

static void set_valve(bool open) {
  valve_open = open;
  const bool pin_high = VALVE_ACTIVE_HIGH ? open : !open;
  digitalWrite(VALVE_RELAY_PIN, pin_high ? HIGH : LOW);
  display_dirty = true;
}

static void draw_status_row(int y, const char *label, bool ok, const char *true_text,
                             const char *false_text) {
  tft.fillRect(0, y, 320, 40, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, y + 10);
  tft.print(label);
  uint16_t color = ok ? TFT_DARKGREEN : TFT_RED;
  tft.fillRoundRect(170, y + 4, 140, 30, 6, color);
  tft.setTextColor(TFT_WHITE, color);
  tft.setCursor(180, y + 12);
  tft.print(ok ? true_text : false_text);
}

static void draw_dashboard(bool full_redraw) {
  if (full_redraw) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(4, 4);
    tft.print("OBot Spray Controller");
    tft.drawFastHLine(0, 28, 320, TFT_DARKGREY);
  }

  draw_status_row(40, "CAN bus", can_bus_healthy, "HEALTHY", "FAULT");
  draw_status_row(84, "Controller", controller_active, "ACTIVE", "INACTIVE");

  tft.fillRect(0, 128, 320, 40, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 138);
  tft.print("Valve");
  uint16_t valve_color = valve_open ? TFT_ORANGE : TFT_DARKGREY;
  tft.fillRoundRect(170, 132, 140, 30, 6, valve_color);
  tft.setTextColor(TFT_BLACK, valve_color);
  tft.setCursor(180, 140);
  tft.print(valve_open ? "OPEN" : "CLOSED");
}

static void draw_connection_banner(AgentState state) {
  const char *text;
  uint16_t color;
  switch (state) {
    case AGENT_CONNECTED:
      text = "AGENT: CONNECTED";
      color = TFT_DARKGREEN;
      break;
    case AGENT_AVAILABLE:
      text = "AGENT: CONNECTING";
      color = TFT_ORANGE;
      break;
    default:
      text = "AGENT: DISCONNECTED";
      color = TFT_RED;
      break;
  }
  tft.fillRect(0, 204, 320, 24, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 206);
  tft.print(text);
}

static void can_bus_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  can_bus_healthy = msg->data;
  display_dirty = true;
}

static void controller_active_callback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  controller_active = msg->data;
  display_dirty = true;
}

static void valve_cmd_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float duration_sec = msg->data;
  if (duration_sec > 0.0f) {
    valve_close_at_ms = millis() + (unsigned long)(duration_sec * 1000.0f);
    set_valve(true);
  } else {
    valve_close_at_ms = 0;
    set_valve(false);
  }
}

static void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }

  if (valve_open && valve_close_at_ms != 0 && millis() >= valve_close_at_ms) {
    valve_close_at_ms = 0;
    set_valve(false);
  }

  valve_state_msg.data = valve_open;
  RCSOFTCHECK(rcl_publish(&valve_state_pub, &valve_state_msg, NULL));
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
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

  pinMode(VALVE_RELAY_PIN, OUTPUT);
  set_valve(false);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.print("Connecting to WiFi...");

  IPAddress agent_ip;
  agent_ip.fromString(AGENT_IP);
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASSWORD, agent_ip, AGENT_PORT);

  draw_dashboard(true);
  draw_connection_banner(g_state);
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
      EXECUTE_EVERY_N_MS(200, g_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                             ? AGENT_CONNECTED
                                             : AGENT_DISCONNECTED);
      if (g_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      g_state = WAITING_AGENT;
      break;
  }

  if (display_dirty) {
    draw_dashboard(false);
    display_dirty = false;
  }

  static unsigned long last_conn_draw = 0;
  if (millis() - last_conn_draw > 500) {
    draw_connection_banner(g_state);
    last_conn_draw = millis();
  }
}
