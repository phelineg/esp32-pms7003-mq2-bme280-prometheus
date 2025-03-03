#include <WiFi.h>
#include <HTTPClient.h>

#include "Config.h"

std::map<String, String> extendLabelss(const std::map<String, String>& baseLabels, const std::map<String, String>& additionalLabels) {
    std::map<String, String> result = baseLabels;
    for (const auto& label : additionalLabels) {
        result[label.first] = label.second;
    }
    return result;
}

long long getLokiTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Level: WARN - Message: No time available (yet)");
    return 0;
  }

  time_t now = mktime(&timeinfo);
  long long timestamp_ns = (long long)now * 1000000000;

  return timestamp_ns;
}

void log(const char* logLevel, const String message) {
  
  long long ts = getLokiTimestamp();
  
  if (ts == 0 || WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to retrieve timestamp. Logging locally.");
    Serial.print("Level: ");
    Serial.print(logLevel);
    Serial.print(" - Message: ");
    Serial.println(message);
    return;
  }

  String labelsJson = "";
  for (const auto& label : extendLabelss(OBSERVABILITY_LABELS, {{"level", String(logLevel)}})) {
    labelsJson += "\"" + label.first + "\": \"" + label.second + "\", ";
  }

  if (labelsJson.length() > 0) {
    labelsJson.remove(labelsJson.length() - 2, 2);
  }

  String payload = String(
    "{"
    "  \"streams\": [{"
    "    \"stream\": {" + labelsJson + "},"
    "    \"values\": [[ \"" + String(ts) + "\",\"" + String(message) + "\"]]"
    "  }]"
    "} "
  );

  String url = String("http://") + LOKI_HOST + ":" + String(LOKI_PORT)
                + "/loki/api/v1/push";

  HTTPClient httpClient;
  httpClient.begin(url);
  httpClient.addHeader("Content-Type", "application/json");
  int httpResponseCode = httpClient.POST(payload);

  if (httpResponseCode > 0) {
    Serial.println("Log sent successfully: " + String(httpResponseCode));
  } else {
    Serial.println("Error sending log: " + String(httpClient.errorToString(httpResponseCode).c_str()));
  }

  httpClient.end();
}
