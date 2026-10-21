#pragma once

// ---- WiFi / micro-ROS agent -----------------------------------------------
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

// IP address of the micro-ROS agent.
#define AGENT_IP        "192.168.1.100"
#define AGENT_PORT      8888

#define ROS_NODE_NAME      "cyd_spray_controller"
#define ROS_NODE_NAMESPACE ""

// ---- GPIO ------------------------------------------------------------------
#define VALVE_RELAY_PIN   22
#define VALVE_ACTIVE_HIGH 1

// ---- Topics ----------------------------------------------------------------
#define TOPIC_CAN_BUS_HEALTHY    "/status/can_bus_healthy"
#define TOPIC_CONTROLLER_ACTIVE  "/status/controller_active"
#define TOPIC_VALVE_COMMAND      "/spray/valve_command"
#define TOPIC_VALVE_STATE        "/spray/valve_state"
#define TOPIC_VBUS_LEFT_NOW      "/obot/bus_voltage/left/current"
#define TOPIC_VBUS_LEFT_MAX      "/obot/bus_voltage/left/max"
#define TOPIC_VBUS_RIGHT_NOW     "/obot/bus_voltage/right/current"
#define TOPIC_VBUS_RIGHT_MAX     "/obot/bus_voltage/right/max"
#define TOPIC_ERROR_LEFT         "/obot/odrive_error/left"
#define TOPIC_ERROR_RIGHT        "/obot/odrive_error/right"
