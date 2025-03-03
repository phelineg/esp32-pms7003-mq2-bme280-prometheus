#include "Config.h"

const IPAddress local_desired_IP(10, 4, 117, 130);
const IPAddress gateway_IP(10, 4, 117, 102);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress dns_IP(1, 1, 1, 1); // Needed to resolve pool.ntp.org

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

const std::map<String, String> OBSERVABILITY_LABELS = {
    {"instance", INSTANCE},
    {"location", "office"}
};
