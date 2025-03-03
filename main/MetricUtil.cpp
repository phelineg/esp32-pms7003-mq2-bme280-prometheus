#include <Arduino.h> 
#include <HTTPClient.h>

#include "MetricUtil.h"
#include "LogUtil.h"
#include "Config.h"

String metricTypeToString(MetricType type) {
    switch (type) {
        case GAUGE: return "gauge";
        case COUNTER: return "counter";
        case HISTOGRAM: return "histogram";
        case SUMMARY: return "summary";
        default: return "unknown";
    }
}

std::map<String, String> extendLabels(const std::map<String, String>& baseLabels, const std::map<String, String>& additionalLabels) {
    std::map<String, String> result = baseLabels;
    for (const auto& label : additionalLabels) {
        result[label.first] = label.second;
    }
    return result;
}

String createPrometheusMetricString(
    const String& name, 
    const String& description, 
    MetricType type, 
    const std::map<String, String>& labels, 
    const String& value
) {
    String result;
    result += "# HELP " + name + " " + description + "\n";
    result += "# TYPE " + name + " " + metricTypeToString(type) + "\n";

    String labelString;
    for (const auto& label : labels) {
        if (!labelString.isEmpty()) {
            labelString += ",";
        }
        labelString += label.first + "=\"" + label.second + "\"";
    }

    result += name + "{" + labelString + "} " + value + "\n";
    return result;
}

void pushMetrics(const String& metrics) {

  String url = String("http://") + PUSHGATEWAY_HOST + ":" + String(PUSHGATEWAY_PORT)
                + "/metrics/job/" + PUSHGATEWAY_JOB + "/instance/" + INSTANCE;
  
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "text/plain");

  int httpResponseCode = http.PUT(metrics);
  if (httpResponseCode > 0) {
    log("INFO", "Metrics pushed successfully. HTTP Response code: " + String(httpResponseCode));
  } else {
    log("ERROR", "Error pushing metrics:" + String(http.errorToString(httpResponseCode)));
  }
  
  http.end();
}