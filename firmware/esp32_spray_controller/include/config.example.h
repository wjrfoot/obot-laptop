#pragma once

// ---- WiFi / micro-ROS agent -----------------------------------------------
// Networks are tried in order until one connects. Each has the IP of the
// machine running micro_ros_agent on that network.
struct WifiNetwork { const char *ssid; const char *password; const char *agent_ip; };
static const WifiNetwork WIFI_NETWORKS[] = {
  {"YOUR_WIFI_SSID_1", "YOUR_WIFI_PASSWORD_1", "192.168.0.52"},
  {"YOUR_WIFI_SSID_2", "YOUR_WIFI_PASSWORD_2", "192.168.0.52"},
};
#define WIFI_CONNECT_TIMEOUT_MS 15000

#define AGENT_PORT      8888

#define ROS_NODE_NAME      "esp32_spray_controller"
#define ROS_NODE_NAMESPACE ""

// ---- GPIO ------------------------------------------------------------------
#define VALVE_RELAY_PIN   27
#define VALVE_ACTIVE_HIGH 1

// ---- Topics ----------------------------------------------------------------
#define TOPIC_CAN_BUS_HEALTHY    "/status/can_bus_healthy"
#define TOPIC_CONTROLLER_ACTIVE  "/status/controller_active"
#define TOPIC_VALVE_COMMAND      "/spray/valve_command"
#define TOPIC_VALVE_STATE        "/spray/valve_state"
