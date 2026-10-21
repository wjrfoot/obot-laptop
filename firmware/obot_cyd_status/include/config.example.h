#pragma once

// ---- WiFi / micro-ROS agent -----------------------------------------------
// Networks are tried in order until one connects.
struct WifiNetwork { const char *ssid; const char *password; const char *agent_ip; };
static const WifiNetwork WIFI_NETWORKS[] = {
  {"YOUR_WIFI_SSID_1", "YOUR_WIFI_PASSWORD_1", "192.168.1.100"},
  {"YOUR_WIFI_SSID_2", "YOUR_WIFI_PASSWORD_2", "192.168.1.100"},
};
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define AGENT_PORT              8888
#define ROS_DOMAIN_ID           5

#define ROS_NODE_NAME      "obot_cyd_status"
#define ROS_NODE_NAMESPACE ""

// ---- Topics (see doc/OBot_CYD_Interfaces.md) -------------------------------
#define TOPIC_STATUS         "/obot/status"
#define TOPIC_DIAGNOSTICS    "/obot/diagnostics"
#define TOPIC_CYD_COMMAND    "/obot/cyd/command"
#define TOPIC_VALVE_STATE    "/spray/valve_state"   // read-only; the CYD never actuates the valve

// Seconds without a new /obot/status before the screen shows LINK STALE.
#define STATUS_STALE_SEC 3

// No valve GPIO on purpose: the separate valve ESP32 owns it.
// Touch controller pins: unverified for this unit; confirm on hardware before use.
