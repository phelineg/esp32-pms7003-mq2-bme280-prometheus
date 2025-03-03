#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <IPAddress.h>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

extern const IPAddress local_desired_IP;
extern const IPAddress gateway_IP;
extern const IPAddress subnet;
extern const IPAddress dns_IP;

extern const char *ntpServer1;
extern const char *ntpServer2;
extern const long gmtOffset_sec;
extern const int daylightOffset_sec;

#define INSTANCE "esp32_air_quality"

#define LOKI_HOST "10.4.x.x"
#define LOKI_PORT 3100

#define PUSHGATEWAY_HOST "10.4.x.x"
#define PUSHGATEWAY_PORT 9090
#define PUSHGATEWAY_JOB "esp32_metrics"

extern const std::map<String, String> OBSERVABILITY_LABELS;

#define SYSTEM_METRIC true
#define WIFI_METRIC true
#define METRICS_REFRESH_RATE 15000
#define BLINK_RATE 60000

#endif
