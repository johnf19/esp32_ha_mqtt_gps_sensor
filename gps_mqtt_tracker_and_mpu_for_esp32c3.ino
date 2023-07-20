#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*
github/johnf19
this project is based on FREERTOS
*/

static const uint32_t GPSBaud = 9600;
const char* ssid = "CAMPUS WIFI SSID"; //edit wifi ssid
const char* password = "USUALLY NO PASSWD"; //edit wifi password
const char* mqtt_server = "YOUR MQTT SERVER"; //edit mqtt server

WiFiClient espClient;
PubSubClient client(espClient);

// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
HardwareSerial ss(1);

typedef struct {
  int time; 
  int date;
  int satellite;
  float lat;
  float lng;
  float alt;
} GPS_DATA;

GPS_DATA gps_data;

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

typedef struct {
  float temp;
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  double x;
  double y;
  double z;
} MPU6050;

MPU6050 sensor_data;

SemaphoreHandle_t xMutexGPS = NULL;
SemaphoreHandle_t xMutexMPU = NULL;
TickType_t timeOut = 1000;

void conTask(void* ptParam) {
  Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(100);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.println(WiFi.localIP());

  client.setServer("192.168.19.101", 1883);
  while (1) {
    if (!client.connected()) {
      reconnect();
    }
    //Client.setCallback(callback);
    if (xSemaphoreTake(xMutexGPS, timeOut) == pdPASS) {
      Serial.println("GPS Publishing...");
      StaticJsonDocument<200> attr;
      client.publish("location/piaggio1", "leave");

      char lat_mqtt[20];
      char lng_mqtt[20];
      char alt_mqtt[10];
      char sat_mqtt[10];
      char x_mqtt[5];
      char y_mqtt[5];
      char z_mqtt[5];


      attr["latitude"] = dtostrf(gps_data.lat, 5, 14, lat_mqtt);
      attr["longitude"] = dtostrf(gps_data.lng, 5, 14, lng_mqtt);
      attr["altitude"] = dtostrf(gps_data.alt, 7, 2, alt_mqtt);
      attr["satellite"] = itoa(gps_data.satellite, sat_mqtt, 10);
      attr["x"] = dtostrf(sensor_data.x, 3, 1, alt_mqtt);
      attr["y"] = dtostrf(sensor_data.y, 3, 1, alt_mqtt);
      attr["z"] = dtostrf(sensor_data.z, 3, 1, alt_mqtt);
      String attributes;
      serializeJson(attr, attributes);
      int len = attributes.length() + 1;
      char msg[len];
      attributes.toCharArray(msg, len);
      Serial.println(msg);

      if (gps_data.lat - 0 != 0 or gps_data.lng - 0 != 0) {
        client.publish("piaggio1/attributes", msg);
        Serial.println(msg);
        Serial.println("GPS Published...");
      } else {
        Serial.println("Waiting for GPS...");
      }
      xSemaphoreGive(xMutexGPS);
      vTaskDelay(2000);
    } else {
      Serial.println("GPS Mutex Error, retring");
      vTaskDelay(10);
    }

    if (xSemaphoreTake(xMutexMPU, timeOut) == pdPASS) {
      Serial.println("MPU Publishing...");
      StaticJsonDocument<200> attr;
      char temp_mqtt[7];
      client.publish("vehicle/tracker/temperature", dtostrf(sensor_data.temp, 4, 2, temp_mqtt)); //edit your tracker name here
      client.publish("vehicle/tracker/movement", "STAY");
      Serial.println("MPU Published...");
      xSemaphoreGive(xMutexMPU);
      vTaskDelay(1000);
    } else {
      Serial.println("MPU Mutex Error, retring");
      vTaskDelay(10);
    }
    vTaskDelay(600000);
  }
  
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("RECONNECTING MQTT");
    if (client.connect("ESP32Client", "user", "testuserpmaekxe5")) {
      Serial.println("MQTT CONNECTED");
      //client.subscribe("esp32/output");
    } else {
      vTaskDelay(10);
    }
  }
}

void mpuTask(void* pvParam) {
  mpu.begin();
  mpu_temp = mpu.getTemperatureSensor();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  while (1) {

    if (xSemaphoreTake(xMutexMPU, timeOut) == pdPASS) {

      mpu_temp->getEvent(&temp);
      mpu_accel->getEvent(&accel);
      mpu_gyro->getEvent(&gyro);

      sensor_data.temp = temp.temperature;
      sensor_data.accX = accel.acceleration.x;
      sensor_data.accY = accel.acceleration.y;
      sensor_data.accZ = accel.acceleration.z;
      sensor_data.gyroX = gyro.gyro.x;
      sensor_data.gyroY = gyro.gyro.y;
      sensor_data.gyroZ = gyro.gyro.z;

      sensor_data.x = RAD_TO_DEG * (atan2(-sensor_data.gyroY, -sensor_data.gyroZ) + PI);
      sensor_data.y = RAD_TO_DEG * (atan2(-sensor_data.gyroX, -sensor_data.gyroZ) + PI);
      sensor_data.z = RAD_TO_DEG * (atan2(-sensor_data.gyroY, -sensor_data.gyroX) + PI);

      xSemaphoreGive(xMutexMPU);
    } else {
    }

    vTaskDelay(10000);
  }
}

  void gpsTask(void* ptParam) {
  ss.begin(GPSBaud, SERIAL_8N1, 4, 5, false);
  Serial.println("Getting GPS");
  while (1) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        Serial.println("searching");
        if (xSemaphoreTake(xMutexGPS, timeOut) == pdPASS) {
          gps_data.time = gps.time.value();
          gps_data.date = gps.date.value();
          gps_data.satellite = gps.satellites.value();
          gps_data.lat = gps.location.lat();
          gps_data.lng = gps.location.lng();
          gps_data.alt = gps.altitude.meters();
          xSemaphoreGive(xMutexGPS);
          vTaskDelay(1);
          if (gps_data.lat - 0 != 0 or gps_data.lng - 0 != 0) {
             vTaskDelay(600000);
          } else {
            vTaskDelay(1);
          }
        } else {
          vTaskDelay(1);
        }
      } else {
        vTaskDelay(1);
      }
    }
  }
}


void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("TASK SETUP");
  xMutexGPS = xSemaphoreCreateMutex();
  xMutexMPU = xSemaphoreCreateMutex();
  xTaskCreate(gpsTask, "", 1024 * 3, NULL, 1, NULL);
  xTaskCreate(mpuTask, "", 1024 * 3, NULL, 2, NULL);
  vTaskDelay(1000);
  xTaskCreate(conTask, "", 1024 * 3, NULL, 3, NULL);
  Serial.println("over");
}
void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
}
