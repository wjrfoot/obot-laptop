#pragma once

// ---- WiFi / micro-ROS agent -----------------------------------------------
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

// IP address of the micro-ROS agent.
#define AGENT_IP        "192.168.1.100"
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
