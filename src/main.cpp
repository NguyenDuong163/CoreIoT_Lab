#include "ExternalLedControl.h"
#include "PumpControl.h"

#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LIGHT_SENSOR_PIN 1 // A0-LUX
#define MOIS_SENSOR_PIN  3 // A2-MOISTURE

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

constexpr char WIFI_SSID[] = "DCrab";
constexpr char WIFI_PASSWORD[] = "zzzzzzzz";

constexpr char TOKEN[] = "6e2vlzqcf3ci6cd2p8n3";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr char MQTT_CLIENT_ID[] = "pk953x4l3gocac91q792";
constexpr char MQTT_USER[] = "u7o0ijppct005wrr2r22";
constexpr char MQTT_PASSWORD[] = "ug6bx7oenuthcceh9xzs";

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;

volatile int ledMode = 0;
volatile bool ledState = false;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, EPC_1> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  // BLINKING_INTERVAL_ATTR
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
//############RPC############//
RPC_Response setLedSwitchState(const RPC_Data &data) {
    Serial.println("Received Switch state");
    bool newState = data;
    Serial.print("Switch state change: ");
    Serial.println(newState);

    digitalWrite(LED_PIN, newState);

    if (newState) {
      tb.sendTelemetryData("in_LED_Status", "ON");
    } else {
      tb.sendTelemetryData("in_LED_Status", "OFF");
    }
    // neoPixelState = newState;

    attributesChanged = true;
    return RPC_Response("setLedSwitchValue", newState);
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
  }
  // attributesChanged = true;
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
void connectToWiFi(void * parameter) {
  while (true) {
    if (!reconnect()) {
      return;
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS); //10s delay
  }
}


void coreIoTConnectTask(void *pvParameters) {
  while (true) {
    if (!tb.connected()) {
      // Serial.print("Connecting to: ");
      // Serial.print(THINGSBOARD_SERVER);
      // Serial.print(" with token ");
      // Serial.println(TOKEN);
      //Use TOKEN (for CP)
      //Use MQTT (for IoT)
      // if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      //   Serial.println("Failed to connect");
      //   // vTaskDelay(10 / portTICK_PERIOD_MS);
      //   continue;
      // }

      //    USE MQTT
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with MQTT_USER: ");
      Serial.print(MQTT_USER);
      Serial.print(" and MQTT_PASSWORD: ");
      Serial.println(MQTT_PASSWORD);
      if (!tb.connect(THINGSBOARD_SERVER, MQTT_USER, THINGSBOARD_PORT, MQTT_CLIENT_ID, MQTT_PASSWORD)) {
        Serial.println("Failed to connect");
        continue;
      }

      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

      Serial.println("Subscribing for RPC...");
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        continue;
      }
      //
      //
      //
      // Đăng ký callback cho setExternLed //@
      if (!tb.RPC_Subscribe(exLed_callbacks.cbegin(), exLed_callbacks.cend())) {
        Serial.println("Failed to subscribe for external LED RPC");
        continue;
      }

      // Đăng ký callback cho setPump //@
      if (!tb.RPC_Subscribe(pump_callbacks.cbegin(), pump_callbacks.cend())) {
        Serial.println("Failed to subscribe for pump RPC");
        continue;
      }
      //
      //
      //
      //
      if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
        Serial.println("Failed to subscribe for shared attribute updates");
        continue;
      }

      Serial.println("Subscribe done");

      if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
        Serial.println("Failed to request for shared attributes");
        continue;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); //1s delay
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
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");

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
  float longtitude = 107.321990;
  float latitude = 10.694964;
  tb.sendTelemetryData("longtitude", longtitude);
  tb.sendTelemetryData("latitude", latitude);
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
  xTaskCreate(connectToWiFi,      "connectToWiFi",      4096, NULL, 1, NULL);
  xTaskCreate(coreIoTConnectTask, "coreIoTConnectTask", 4096, NULL, 1, NULL);
  xTaskCreate(sendAtributesTask,  "sendAtributesTask",  4096, NULL, 2, NULL);
  xTaskCreate(sendTelemetryTask,  "sendTelemetryTask",  4096, NULL, 2, NULL);
  xTaskCreate(tbLoopTask,         "tbLoopTask",         4096, NULL, 1, NULL);
  xTaskCreate(neoPixelTask,       "neoPixelTask",       2048, NULL, 2, NULL);
  // xTaskCreate(lightSensorTask,    "lightSensorTask",    2048, NULL, 2, NULL);
  // xTaskCreate(moisSensorTask,     "moisSensorTask",     2048, NULL, 2, NULL);
  xTaskCreate(pumpTask,           "pumpTask",           2048, NULL, 2, NULL);
}

void loop() {
  
}
