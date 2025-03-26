#include "ExternalLedControl.h"
#include "PumpControl.h"
#include "Led_scheduler.h"

#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LIGHT_SENSOR_PIN 1 // A0-LUX
#define MOIS_SENSOR_PIN  3 // A2-MOISTURE

#include <WiFi.h>
#include <string.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>

constexpr char WIFI_SSID[] = "DCrab";
constexpr char WIFI_PASSWORD[] = "zzzzzzzz";

constexpr char TOKEN[] = "6e2vlzqcf3ci6cd2p8n3";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr char MQTT_CLIENT_ID[] = "device1";
constexpr char MQTT_USER[] = "device1";
constexpr char MQTT_PASSWORD[] = "device1";

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

extern const std::array<RPC_Callback, 1U> callbacks;
extern const std::array<RPC_Callback, 1U> exLed_callbacks;
extern const std::array<RPC_Callback, 1U> pump_callbacks;

// constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";
constexpr char FW_TAG[] = "fw_tag";
constexpr char FW_VERSION[] = "fw_version";
constexpr char FW_CHECKSUM[] = "fw_checksum";
constexpr char FW_TITLE[] = "fw_title";
constexpr char FW_SIZE[] = "fw_size";
constexpr char FW_CHECKSUM_ALGORITHM[] = "fw_checksum_algorithm";
constexpr char FW_URL[] = "fw_url";

constexpr char SCHED_BLINK_LED[] = "sched_blink_led";
constexpr char SCHED_BLINK_LED_INTERVAL[] = "sched_blink_led_interval";
constexpr char SCHED_BLINK_LED_DURATION[] = "sched_blink_led_duration";

volatile bool attributesChanged = false;

volatile int ledMode = 0;
volatile bool ledState = false;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, EPC_1> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  SCHED_BLINK_LED,
  SCHED_BLINK_LED_INTERVAL,
  SCHED_BLINK_LED_DURATION,
  FW_TAG,
  FW_VERSION,
  FW_CHECKSUM,
  FW_TITLE,
  FW_SIZE,
  FW_CHECKSUM_ALGORITHM,
  FW_URL
  // BLINKING_INTERVAL_ATTR
};

String fwTag;
String fwVersion;
String fwChecksum;
String fwTitle;
int fwSize;
String fwChecksumAlgorithm;
String fwUrl;

String currentFwTag;
String currentFwVersion;
String currentFwChecksum;
String currentFwTitle;
int currentFwSize;
String currentFwChecksumAlgorithm;
String currentFwUrl;

bool sched_blink_led = false;
int sched_blink_led_interval = 5; // 5s
int sched_blink_led_duration = 60; // 60s

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
//############RPC############//
RPC_Response setLedSwitchState(const RPC_Data &data) {
  try {
    if (data.isNull()) {
      Serial.println("Received empty RPC data");
      return RPC_Response("error", "Empty input");
    }
    
    bool newState = data;
    Serial.print("Switch state change: ");
    Serial.println(newState);

    digitalWrite(LED_PIN, newState);
    tb.sendTelemetryData("in_LED_Status", newState ? "ON" : "OFF");
    
    return RPC_Response("setLedSwitchValue", newState);
  } catch (...) {
    Serial.println("Exception in RPC callback");
    return RPC_Response("error", "Processing failed");
  }
}
//##########################//
const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setLedSwitchValue", setLedSwitchState }
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {

    if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
    if (strcmp(it->key().c_str(), SCHED_BLINK_LED) == 0) {
      sched_blink_led = it->value().as<bool>();
      Serial.print("SCHED_BLINK_LED: ");
      Serial.println(sched_blink_led);
    }
    if (strcmp(it->key().c_str(), SCHED_BLINK_LED_INTERVAL) == 0) {
      sched_blink_led_interval = 1000 * (it->value().as<int>());
      Serial.print("SCHED_BLINK_LED_INTERVAL: ");
      Serial.println(it->value().as<int>());
    }
    if (strcmp(it->key().c_str(), SCHED_BLINK_LED_DURATION) == 0) {
      sched_blink_led_duration = 1000 * (it->value().as<int>());
      Serial.print("SCHED_BLINK_LED_DURATION: ");
      Serial.println(it->value().as<int>());
    }
    if (strcmp(it->key().c_str(), FW_TAG) == 0) {
      Serial.print("FW Tag: ");
      fwTag = it->value().as<String>();
      Serial.println(it->value().as<String>());
    }
    if (strcmp(it->key().c_str(), FW_VERSION) == 0) {
      Serial.print("FW Version: ");
      fwVersion = it->value().as<String>();
      Serial.println(it->value().as<String>());
    }
    if (strcmp(it->key().c_str(), FW_CHECKSUM) == 0) {
      Serial.print("FW Checksum: ");
      fwChecksum = it->value().as<String>();
      Serial.println(it->value().as<String>());
    }
    if (strcmp(it->key().c_str(), FW_TITLE) == 0) {
      Serial.print("FW Title: ");
      fwTitle = it->value().as<String>();
      Serial.println(it->value().as<String>());
    }
    if (strcmp(it->key().c_str(), FW_SIZE) == 0) {
      Serial.print("FW Size: ");
      fwSize = it->value().as<int>();
      Serial.println(it->value().as<String>());
    }
    if (strcmp(it->key().c_str(), FW_CHECKSUM_ALGORITHM) == 0) {
      Serial.print("FW Checksum Algorithm: ");
      fwChecksumAlgorithm = it->value().as<String>();
      Serial.println(it->value().as<String>());
    }
    if (strcmp(it->key().c_str(), FW_URL) == 0) {
      Serial.print("FW URL: ");
      fwUrl = it->value().as<String>();
      Serial.println(it->value().as<String>());
    }
  }
  Serial.println("-----------------------------------");
  attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to WiFi ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(">");
  }
  Serial.println("Connected to WiFi");
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

//############RTOS task############//

void updateFirmwareTask(void *pvParameters) {
  WiFiClientSecure client;
  HTTPClient http;
  
  client.setInsecure();
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  while (true) {
    if (attributesChanged) {
      attributesChanged = false;
      
      if (fwUrl != currentFwUrl || fwVersion != currentFwVersion) {
        Serial.println("\nFirmware update available");
        Serial.printf("Current: %s\nNew: %s\n", currentFwVersion.c_str(), fwVersion.c_str());
        
        currentFwUrl = fwUrl;
        currentFwVersion = fwVersion;
        currentFwChecksum = fwChecksum;

        if (fwUrl.length() > 0) {
          Serial.printf("Downloading from: %s\n", fwUrl.c_str());
          
          http.begin(client, fwUrl);
          int httpCode = http.GET();
          
          if (httpCode == HTTP_CODE_OK) {
            int contentLength = http.getSize();
            Serial.printf("File size: %d bytes\n", contentLength);
            
            if (Update.begin(contentLength)) {
              size_t written = Update.writeStream(*http.getStreamPtr());
              
              if (written == contentLength) {
                Serial.println("Firmware fully written");
              } else {
                Serial.printf("Partial write: %d/%d bytes\n", written, contentLength);
              }

              if (Update.end()) {
                if (Update.isFinished()) {
                  Serial.println("Update successful, rebooting...");
                  tb.sendTelemetryData("fw_state", "UPDATED");
                  delay(1000);
                  ESP.restart();
                } else {
                  Serial.println("Update incomplete");
                  tb.sendTelemetryData("fw_state", "FAILED");
                }
              } else {
                Serial.printf("Update error: %d\n", Update.getError());
                tb.sendTelemetryData("fw_state", "UPDATE_FAILED");
                tb.sendTelemetryData("fw_error", String(Update.getError()));
              }
            } else {
              Serial.println("Not enough space for update");
              tb.sendTelemetryData("fw_state", "INSUFFICIENT_SPACE");
            }
          } else {
            Serial.printf("HTTP error: %d\n", httpCode);
            tb.sendTelemetryData("fw_state", "DOWNLOAD_FAILED");
            tb.sendTelemetryData("http_code", httpCode);
          }
          http.end();
        }
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void connectToWiFi(void * parameter) {
  while (true) {
    if (!reconnect()) {
      return;
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS); //10s delay
  }
}

void coreIoTConnectTask(void *pvParameters) {
  const uint32_t RECONNECT_DELAY_MS = 5000;
  const uint32_t SUBSCRIPTION_DELAY_MS = 100;
  
  while (true) {
    // Kiểm tra kết nối WiFi trước
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Mất kết nối, đợi kết nối lại...");
      vTaskDelay(RECONNECT_DELAY_MS / portTICK_PERIOD_MS);
      continue;
    }

    // Kết nối ThingsBoard nếu chưa kết nối
    if (!tb.connected()) {
      Serial.println("[TB] Đang kết nối tới ThingsBoard...");
      
      // Đảm bảo ngắt kết nối cũ (nếu có)
      if (mqttClient.connected()) {
        mqttClient.disconnect();
        Serial.println("[TB] Đã đóng kết nối cũ");
      }

      // Thử kết nối với timeout
      bool connected = tb.connect(THINGSBOARD_SERVER, MQTT_USER, THINGSBOARD_PORT, 
                                MQTT_CLIENT_ID, MQTT_PASSWORD);
      
      if (!connected) {
        Serial.println("[TB] Kết nối thất bại, thử lại sau 5s");
        vTaskDelay(RECONNECT_DELAY_MS / portTICK_PERIOD_MS);
        continue;
      }

      Serial.println("[TB] Đã kết nối, đăng ký RPC...");

      // 1. Đăng ký main RPC callbacks
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("[TB] Lỗi đăng ký main RPC");
        tb.disconnect();
        continue;
      }
      vTaskDelay(SUBSCRIPTION_DELAY_MS / portTICK_PERIOD_MS);

      // 2. Đăng ký external LED RPC
      if (!tb.RPC_Subscribe(exLed_callbacks.cbegin(), exLed_callbacks.cend())) {
        Serial.println("[TB] Lỗi đăng ký external LED RPC");
        tb.disconnect();
        continue;
      }
      vTaskDelay(SUBSCRIPTION_DELAY_MS / portTICK_PERIOD_MS);

      // 3. Đăng ký pump RPC
      if (!tb.RPC_Subscribe(pump_callbacks.cbegin(), pump_callbacks.cend())) {
        Serial.println("[TB] Lỗi đăng ký pump RPC");
        tb.disconnect();
        continue;
      }
      vTaskDelay(SUBSCRIPTION_DELAY_MS / portTICK_PERIOD_MS);

      // Đăng ký shared attributes
      if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
        Serial.println("[TB] Lỗi đăng ký shared attributes");
        tb.disconnect();
        continue;
      }

      // Gửi thông tin thiết bị
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
      tb.sendAttributeData("fw_version", currentFwVersion.c_str());
      
      Serial.println("[TB] Đã đăng ký tất cả RPC và attributes");
    }

    // Xử lý message loop với try-catch
    try {
      if (tb.connected()) {
        tb.loop();
      }
    } catch (const std::exception& e) {
      Serial.printf("[TB] Lỗi trong loop: %s\n", e.what());
      tb.disconnect();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void sendAtributesTask(void *pvParameters) {
  while (true) {
    if (attributesChanged) {
      attributesChanged = false;
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }

    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());

    vTaskDelay(1000 / portTICK_PERIOD_MS); //1s delay
  }
}


void sendTelemetryTask(void *pvParameters) {
  while (true) {
      dht20.read();
      
      float temperature = dht20.getTemperature();
      float humidity = dht20.getHumidity();
      
      if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT20 sensor!");
      } else {
        // Serial.print("Temperature: ");
        // Serial.print(temperature);
        // Serial.print(" °C, Humidity: ");
        // Serial.print(humidity);
        // Serial.println(" %");

        tb.sendTelemetryData("temperature", temperature);
        tb.sendTelemetryData("humidity", humidity);
      }
      vTaskDelay(5000 / portTICK_PERIOD_MS); //2s delay
    }
}

void lightSensorTask(void *pvParameters) {
  while (true) {
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    Serial.print("Light Sensor Value: ");
    Serial.println(lightValue);
    tb.sendTelemetryData("light", lightValue);
    vTaskDelay(2000 / portTICK_PERIOD_MS); //2s delay
  }
}

void moisSensorTask(void *pvParameters) {
  while (true) {
    int rawMoistureValue = analogRead(MOIS_SENSOR_PIN);
    float moistureValue = (rawMoistureValue * 1.0 / 4095.0) * 100;
    Serial.print("Moisture Sensor Value: ");
    Serial.print(moistureValue);
    Serial.println("%");
    tb.sendTelemetryData("moisture", moistureValue);
    vTaskDelay(2000 / portTICK_PERIOD_MS); //2s delay
  }
}

void tbLoopTask(void *pvParameters) {
  while (true) {
    tb.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS); //10ms delay
  }
}
//#################################//
void setup() {
  // float longtitude = 107.321990;
  // float latitude = 10.694964;
  // tb.sendTelemetryData("longtitude", longtitude);
  // tb.sendTelemetryData("latitude", latitude);
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
  xTaskCreate(connectToWiFi,      "connectToWiFi",      4096, NULL, 1, NULL);
  xTaskCreate(coreIoTConnectTask, "coreIoTConnectTask", 12288, NULL, 1, NULL);
  xTaskCreate(sendAtributesTask,  "sendAtributesTask",  4096, NULL, 2, NULL);
  xTaskCreate(sendTelemetryTask,  "sendTelemetryTask",  4096, NULL, 2, NULL);
  xTaskCreate(tbLoopTask,         "tbLoopTask",         8192, NULL, 1, NULL);
  xTaskCreate(neoPixelTask,       "neoPixelTask",       2048, NULL, 2, NULL);
  xTaskCreate(updateFirmwareTask, "updateFirmwareTask", 12288, NULL, 2, NULL);
  xTaskCreate(ledSchedulerTask,   "ledSchedulerTask",   2048, NULL, 2, NULL);
  // xTaskCreate(lightSensorTask,    "lightSensorTask",    2048, NULL, 2, NULL);
  // xTaskCreate(moisSensorTask,     "moisSensorTask",     2048, NULL, 2, NULL);
  // xTaskCreate(pumpTask,           "pumpTask",           2048, NULL, 2, NULL);
}

void loop() {
  
}
