#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>

#include "MetricUtil.h"
#include "LogUtil.h"
#include "Config.h"

unsigned long lastSensorUpdate = 0;
unsigned long lastBlink = 0;

/***************************** MQ-2 *******************************************/
#include <MQUnifiedsensor.h>

#define         Board                   ("ESP-32")
#define         MQ_2_Pin                (32)
#define         MQ_2_Type               ("MQ-2")
#define         Voltage_Resolution      (3.3)
#define         ADC_Bit_Resolution      (12)
#define         RatioMQ2CleanAir        (9.83)

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ_2_Pin, MQ_2_Type);

float MQ_2_H2_ppm = NAN;
float MQ_2_LPG_ppm = NAN;
float MQ_2_CO_ppm = NAN;
float MQ_2_Alcohol_ppm = NAN;
float MQ_2_Propane_ppm = NAN;

/***************************** BME280 *******************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1020) // For ESSA/ARN

Adafruit_BME280 bme;

float temperature = NAN;
float humidity = NAN;
float pressure = NAN;
float altitude = NAN;

/***************************** PMS7003 *******************************************/
#define RXD2 16

struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

boolean pms7003reading = false;
struct pms7003data pms7003data;

/***************************** RGB LED *******************************************/
#define RED_PIN 13
#define GREEN_PIN 14
#define BLUE_PIN 27

const int fadeDelay = 20;
const int maxBrightness = 20;
const int minBrightness = 0;

void timeavailable(struct timeval *t) {
  Serial.println("Got time adjustment from NTP!");
}

boolean readPMSdata(Stream *s) {
  if (!s->available()) {
    log("ERROR", "PMS7003 Sensor reading failed: No data available.");
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    log("ERROR", "PMS7003 Sensor reading failed: Invalid start bytes.");
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    log("ERROR", "PMS7003 Sensor reading failed: Incomplete data frame.");
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&pms7003data, (void *)buffer_u16, 30);

  if (sum != pms7003data.checksum) {
    log("ERROR", "PMS7003 Sensor reading failed: Checksum mismatch.");
    return false;
  }
  // success!
  return true;
}

String generateMetricsString() {

  String metrics;

  metrics += createPrometheusMetricString("system_uptime_total","Uptime in milliseconds",COUNTER,OBSERVABILITY_LABELS,String(millis()));

  if(WIFI_METRIC){
    metrics += createPrometheusMetricString("wifi_rssi", "Received Signal Strength Indicator value measures the strength of the Wi-Fi signal received by your ESP32", GAUGE, OBSERVABILITY_LABELS, String(WiFi.RSSI()));
    metrics += createPrometheusMetricString("wifi_mac_address", "MAC address of the ESP32's Wi-Fi interface", GAUGE, extendLabels(OBSERVABILITY_LABELS, {{"mac_address", WiFi.macAddress()}}), "1");
    metrics += createPrometheusMetricString("wifi_local_ip", "Current IP address of the ESP32", GAUGE, extendLabels(OBSERVABILITY_LABELS, {{"local_ip", WiFi.localIP().toString()}}), "1");
  }

  if(SYSTEM_METRIC){
    metrics += createPrometheusMetricString("system_heap_size", "Total heap size", GAUGE, OBSERVABILITY_LABELS, String(ESP.getHeapSize()));
    metrics += createPrometheusMetricString("system_free_heap", "Available free heap size", GAUGE, OBSERVABILITY_LABELS, String(ESP.getFreeHeap()));
    metrics += createPrometheusMetricString("system_min_free_heap", "Minimum free heap size since boot", GAUGE, OBSERVABILITY_LABELS, String(ESP.getMinFreeHeap()));
    metrics += createPrometheusMetricString("system_max_alloc_heap", "Largest allocable block of heap", GAUGE, OBSERVABILITY_LABELS, String(ESP.getMaxAllocHeap()));
    metrics += createPrometheusMetricString("system_psram_size", "Total SPI RAM size", GAUGE, OBSERVABILITY_LABELS, String(ESP.getPsramSize()));
    metrics += createPrometheusMetricString("system_free_psram", "Available free SPI RAM", GAUGE, OBSERVABILITY_LABELS, String(ESP.getFreePsram()));
    metrics += createPrometheusMetricString("system_min_free_psram", "Minimum free SPI RAM size since boot", GAUGE, OBSERVABILITY_LABELS, String(ESP.getMinFreePsram()));
    metrics += createPrometheusMetricString("system_max_alloc_psram", "Largest allocable block of SPI RAM", GAUGE, OBSERVABILITY_LABELS, String(ESP.getMaxAllocPsram()));
    metrics += createPrometheusMetricString("system_chip_revision", "ESP chip revision", GAUGE, OBSERVABILITY_LABELS, String(ESP.getChipRevision()));
    metrics += createPrometheusMetricString("system_chip_model", "ESP chip model", GAUGE, extendLabels(OBSERVABILITY_LABELS, {{"chip_model", String(ESP.getChipModel())}}), "1");
    metrics += createPrometheusMetricString("system_chip_cores", "Number of CPU cores on the ESP chip", GAUGE, OBSERVABILITY_LABELS, String(ESP.getChipCores()));
    metrics += createPrometheusMetricString("system_cpu_freq", "CPU frequency in MHz", GAUGE, OBSERVABILITY_LABELS, String(ESP.getCpuFreqMHz()));
    metrics += createPrometheusMetricString("system_cycle_count_total", "Cycle count since boot", COUNTER, OBSERVABILITY_LABELS, String(ESP.getCycleCount()));
    metrics += createPrometheusMetricString("system_sdk_version", "ESP-IDF SDK version", GAUGE, extendLabels(OBSERVABILITY_LABELS, {{"sdk_version", String(ESP.getSdkVersion())}}), "1");
    metrics += createPrometheusMetricString("system_core_version", "ESP core version", GAUGE, extendLabels(OBSERVABILITY_LABELS, {{"sdk_version", String(ESP.getCoreVersion())}}), "1");
    metrics += createPrometheusMetricString("system_flash_chip_size", "Flash chip size", GAUGE, OBSERVABILITY_LABELS, String(ESP.getFlashChipSize()));
    metrics += createPrometheusMetricString("system_flash_chip_speed", "Flash chip speed", GAUGE, OBSERVABILITY_LABELS, String(ESP.getFlashChipSpeed()));
    metrics += createPrometheusMetricString("system_flash_chip_mode", "Flash chip mode", GAUGE, OBSERVABILITY_LABELS, String(ESP.getFlashChipMode()));
    metrics += createPrometheusMetricString("system_sketch_size", "Sketch size in bytes", GAUGE, OBSERVABILITY_LABELS, String(ESP.getSketchSize()));
    metrics += createPrometheusMetricString("system_sketch_md5", "Sketch MD5 checksum", GAUGE, extendLabels(OBSERVABILITY_LABELS, {{"sketch_md5", String(ESP.getSketchMD5())}}), "1");
    metrics += createPrometheusMetricString("system_free_sketch_space", "Free space in the sketch", GAUGE, OBSERVABILITY_LABELS, String(ESP.getFreeSketchSpace()));
    metrics += createPrometheusMetricString("system_mac_address", "MAC address of the device", GAUGE, OBSERVABILITY_LABELS, String(ESP.getEfuseMac()));
  }

  std::map<String, String> bme280Labels = extendLabels(OBSERVABILITY_LABELS, {{"sensor", "BME280"}});
  if (!isnan(temperature)) {
    metrics += createPrometheusMetricString("temperature", "Temperature in deg. C", GAUGE, bme280Labels, String(temperature));
  }
  if (!isnan(humidity)) {
    metrics += createPrometheusMetricString("humidity", "Humidity %", GAUGE, bme280Labels, String(humidity));
  }
  if (!isnan(pressure)) {
    metrics += createPrometheusMetricString("pressure", "Pressure in hPa", GAUGE, bme280Labels, String(pressure));
  }
  if (!isnan(altitude)) {
    metrics += createPrometheusMetricString("altitude", "Altitude in m", GAUGE, bme280Labels, String(altitude));
  }

  std::map<String, String> mq2Labels = extendLabels(OBSERVABILITY_LABELS, {{"sensor", "MQ-2"}});
  if (!isnan(MQ_2_H2_ppm)) {
    metrics += createPrometheusMetricString("H2", "Hydrogen gas concentration in ppm", GAUGE, mq2Labels, String(MQ_2_H2_ppm));
  }
  if (!isnan(MQ_2_LPG_ppm)) {
    metrics += createPrometheusMetricString("LPG", "Liquefied Petroleum Gas (LPG) concentration in ppm", GAUGE, mq2Labels, String(MQ_2_LPG_ppm));
  }
  if (!isnan(MQ_2_CO_ppm)) {
    metrics += createPrometheusMetricString("CO", "Carbon monoxide concentration in ppm", GAUGE, mq2Labels, String(MQ_2_CO_ppm));
  }
  if (!isnan(MQ_2_Alcohol_ppm)) {
    metrics += createPrometheusMetricString("alcohol", "Ethanol vapor concentration in ppm", GAUGE, mq2Labels, String(MQ_2_Alcohol_ppm));
  }
  if (!isnan(MQ_2_Propane_ppm)) {
    metrics += createPrometheusMetricString("propane", "Propane gas concentration in ppm", GAUGE, mq2Labels, String(MQ_2_Propane_ppm));
  }

  std::map<String, String> pms7003Labels = extendLabels(OBSERVABILITY_LABELS, {{"sensor", "PMS7003"}});
  if(pms7003reading){
    metrics += createPrometheusMetricString("PM_1_Standard", "PM 1.0 Concentration Units (standard)", GAUGE, pms7003Labels, String(pms7003data.pm10_standard));
    metrics += createPrometheusMetricString("PM_2_5_Standard", "PM 2.5 Concentration Units (standard)", GAUGE, pms7003Labels, String(pms7003data.pm25_standard));
    metrics += createPrometheusMetricString("PM_10_0_Standard", "PM 10 Concentration Units (standard)", GAUGE, pms7003Labels, String(pms7003data.pm100_standard));

    metrics += createPrometheusMetricString("PM_1_Env", "PM 1.0 Concentration Units (environmental)", GAUGE, pms7003Labels, String(pms7003data.pm10_env));
    metrics += createPrometheusMetricString("PM_2_5_Env", "PM 2.5 Concentration Units (environmental)", GAUGE, pms7003Labels, String(pms7003data.pm25_env));
    metrics += createPrometheusMetricString("PM_10_0_Env", "PM 10 Concentration Units (environmental)", GAUGE, pms7003Labels, String(pms7003data.pm100_env));

    metrics += createPrometheusMetricString("particles_03um", "Particles > 0.3um / 0.1L air", GAUGE, pms7003Labels, String(pms7003data.particles_03um));
    metrics += createPrometheusMetricString("particles_05um", "Particles > 0.5um / 0.1L air", GAUGE, pms7003Labels, String(pms7003data.particles_05um));
    metrics += createPrometheusMetricString("particles_10um", "Particles > 1.0um / 0.1L air", GAUGE, pms7003Labels, String(pms7003data.particles_10um));
    metrics += createPrometheusMetricString("particles_25um", "Particles > 2.5um / 0.1L air", GAUGE, pms7003Labels, String(pms7003data.particles_25um));
    metrics += createPrometheusMetricString("particles_50um", "Particles > 5.0um / 0.1L air", GAUGE, pms7003Labels, String(pms7003data.particles_50um));
    metrics += createPrometheusMetricString("particles_100um", "Particles > 10.0um / 0.1L air", GAUGE, pms7003Labels, String(pms7003data.particles_100um));
  }

  return metrics;
}

void updateSensorReadings() {

  log("INFO", "Collecting BME280 sensor information");

  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if(isnan(temperature) || temperature > 100){
    log("WARNING", "Could not read BME280 Temperature");
  }

  if(isnan(humidity) || humidity > 99){
    log("WARNING", "Could not read BME280 Humidity");
  }

  if(isnan(pressure) || pressure < 100){
    log("WARNING", "Could not read BME280 Pressure");
  }

  if(isnan(altitude)){
    log("WARNING", "Could not read BME280 altitude");
  }
 
  log("INFO", "Collecting MQ-2 sensor information");

  MQ2.update();

  MQ2.setA(987.99); MQ2.setB(-2.162); MQ_2_H2_ppm = MQ2.readSensor();
  MQ2.setA(574.25); MQ2.setB(-2.222); MQ_2_LPG_ppm = MQ2.readSensor();
  MQ2.setA(36974); MQ2.setB(-3.109); MQ_2_CO_ppm = MQ2.readSensor();
  MQ2.setA(3616.1); MQ2.setB(-2.675); MQ_2_Alcohol_ppm = MQ2.readSensor();
  MQ2.setA(658.71); MQ2.setB(-2.168); MQ_2_Propane_ppm = MQ2.readSensor();

  log("INFO", "Collecting PMS7003 sensor information");

  if (readPMSdata(&Serial1)) {
    pms7003reading = true;
  } else {
    log("ERROR","PMS7003 Sensor reading failed.");
    pms7003reading = false;
  }

  if(!pms7003reading || isnan(temperature) || isnan(MQ_2_H2_ppm)){
    redBlink();
  } else {
    greenBlink();
  }
}

void setup() {
  Serial.begin(115200);

  /**
  * Wifi
  */

  if (!WiFi.config(local_desired_IP, gateway_IP, subnet, dns_IP)) {
    log("ERROR", "Failed to configure Static IP");
  }

  log("INFO", "Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  log("INFO","Connected to Wi-Fi! IP Address: " + WiFi.localIP().toString());

  /**
  * NTP
  */

  sntp_set_time_sync_notification_cb(timeavailable);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  /**
  * BME280 
  */

  if (!bme.begin(0x76)) {
    log("ERROR","BME280 Sensor reading failed : please check your wiring and supply.");
  }

  /**
  * MQ-2 
  */
  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25); MQ2.setB(-2.222);
  MQ2.init(); 
 
  log("INFO","Calibrating MQ-2.");
  float mq_2_calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
    MQ2.update();
    mq_2_calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
  }
  MQ2.setR0(mq_2_calcR0/10);
  log("INFO","MQ-2 Calibration done. R0 : " + String(MQ2.getR0()) + " Rs : " + String(MQ2.getRS()) + " Rl : " + String(MQ2.getRL()) + " Rs/R0 : " + String(MQ2.getRS() / MQ2.getR0()));
  
  if(isinf(mq_2_calcR0)) {
    log("ERROR","MQ-2 Calibration failed : Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply.");
  }

  if(mq_2_calcR0 == 0){
    log("ERROR","MQ-2 Calibration failed : Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply.");
  }

  /**
  * PMS7003
  */
  Serial1.begin(9600, SERIAL_8N1, RXD2);

  /**
  * RGB LED
  */
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  analogWrite(RED_PIN, 0);
  analogWrite(BLUE_PIN, 0);
  analogWrite(GREEN_PIN, 0);
}

void redBlink() {
    for (int brightness = minBrightness; brightness <= maxBrightness; brightness++) {
        analogWrite(RED_PIN, brightness);
        delay(fadeDelay);
    }

    for (int brightness = maxBrightness; brightness >= minBrightness; brightness--) {
        analogWrite(RED_PIN, brightness);
        delay(fadeDelay);
    }
}

void greenBlink() {
    for (int brightness = minBrightness; brightness <= maxBrightness; brightness++) {
        analogWrite(GREEN_PIN, brightness);
        delay(fadeDelay);
    }

    for (int brightness = maxBrightness; brightness >= minBrightness; brightness--) {
        analogWrite(GREEN_PIN, brightness);
        delay(fadeDelay);
    }
}

void loop() {

  unsigned long now = millis();

  if (now - lastSensorUpdate >= METRICS_REFRESH_RATE) {

    unsigned long startTime = millis();
    lastSensorUpdate = now;
    
    updateSensorReadings();
  
    String metrics = generateMetricsString();
    pushMetrics(metrics);

    unsigned long elapsedTime = millis() - startTime;
    log("INFO", "Collecting Sensor information & pushing metrics took : " + String(elapsedTime) + " ms");
  }
}