#include<Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <ESP32Servo.h>

// ------------------- WiFi -------------------
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define WIFI_CHANNEL 6

// ------------------- Pins -------------------
#define ONE_WIRE_BUS 4
#define DHT_PIN 5
#define DHT_TYPE DHT22
#define SERVO_PIN 13
#define STEP_PIN 12
#define DIR_PIN 14
#define TRIG1 15
#define ECHO1 16
#define TRIG2 17
#define ECHO2 18
#define GAS_PIN 33
#define GAS_THRESHOLD 2000

// ------------------- Globals -------------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);
DHT dht(DHT_PIN, DHT_TYPE);
Servo servo;
WebServer server(80);

int stepSpeed = 30;
unsigned long lastReadTime = 0, lastStepTime = 0, lastServoTime = 0;
#define READ_INTERVAL 1000
#define SERVO_INTERVAL 15

float dsTemp = 0.0, dhtTemp = 0.0, dhtHum = 0.0;
int distance1 = 0, distance2 = 0, gasValue = 0;
bool servoActive = false;
bool stepperActive = false;

// ------------------- Sensor Reading -------------------
void readSensors() {
  ds.requestTemperatures();
  dsTemp = ds.getTempCByIndex(0);
  if (dsTemp == -127.0) dsTemp = 0;

  dhtTemp = dht.readTemperature();
  dhtHum = dht.readHumidity();
  if (isnan(dhtTemp)) dhtTemp = 0;
  if (isnan(dhtHum)) dhtHum = 0;

  digitalWrite(TRIG1, LOW); delayMicroseconds(2);
  digitalWrite(TRIG1, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  long duration1 = pulseIn(ECHO1, HIGH, 30000);
  distance1 = duration1 / 58.2;

  digitalWrite(TRIG2, LOW); delayMicroseconds(2);
  digitalWrite(TRIG2, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  long duration2 = pulseIn(ECHO2, HIGH, 30000);
  distance2 = duration2 / 58.2;

  gasValue = analogRead(GAS_PIN);
}

// ------------------- Actuator Logic -------------------
void controlActuators() {
  unsigned long now = micros();
  unsigned long nowMs = millis();

  stepperActive = dsTemp > 100;
  if (stepperActive && now - lastStepTime >= (1000 * (50 - stepSpeed))) {
    lastStepTime = now;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
  }

  if (distance1 < 20) {
    servoActive = true;
    static int pos = 0, dir = 1;
    if (nowMs - lastServoTime >= SERVO_INTERVAL) {
      lastServoTime = nowMs;
      pos += dir;
      if (pos >= 180) dir = -1;
      else if (pos <= 0) dir = 1;
      servo.write(pos);
    }
  } else {
    servoActive = false;
    servo.write(90);
  }
}

// ------------------- Web Dashboard -------------------
void handleRoot() {
  String html = R"(
  <!DOCTYPE html><html>
  <head>
    <title>Fermentation Dashboard</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
      body { font-family: Arial; background:#f2f2f2; text-align:center; padding:10px;}
      .card { background:white; border-radius:12px; padding:20px; margin:10px auto; width:300px;
              box-shadow:0 2px 6px rgba(0,0,0,0.2);}
      .on {color:green; font-weight:bold;}
      .off {color:red; font-weight:bold;}
    </style>
    <script>
      async function updateData(){
        let res = await fetch('/sensor');
        let d = await res.json();
        document.getElementById('ds').innerText = d.ds18b20;
        document.getElementById('dt').innerText = d.dht_temp;
        document.getElementById('dh').innerText = d.dht_hum;
        document.getElementById('s1').innerText = d.sonar1;
        document.getElementById('s2').innerText = d.sonar2;
        document.getElementById('mq').innerText = d.mq;
        document.getElementById('pump').innerText = d.pump ? "ON" : "OFF";
        document.getElementById('fan').innerText = d.fan ? "ON" : "OFF";
      }
      setInterval(updateData, 1000);
      window.onload = updateData;
    </script>
  </head>
  <body>
    <h1>Fermentation Monitoring</h1>
    <div class='card'>
      <h3>Sensor Readings</h3>
      DS18B20: <span id='ds'></span>°C<br>
      DHT22 Temp: <span id='dt'></span>°C<br>
      Humidity: <span id='dh'></span>%<br>
      Sonar1: <span id='s1'></span> cm<br>
      Sonar2: <span id='s2'></span> cm<br>
      MQ: <span id='mq'></span><br>
    </div>
    <div class='card'>
      <h3>Actuators</h3>
      Pump: <span id='pump'></span><br>
      Fan: <span id='fan'></span>
    </div>
  </body>
  </html>
  )";
  server.send(200, "text/html", html);
}

// ------------------- JSON API -------------------
void handleSensor() {
  String json = "{";
  json += "\"ds18b20\":" + String(dsTemp, 2) + ",";
  json += "\"dht_temp\":" + String(dhtTemp, 2) + ",";
  json += "\"dht_hum\":" + String(dhtHum, 2) + ",";
  json += "\"sonar1\":" + String(distance1) + ",";
  json += "\"sonar2\":" + String(distance2) + ",";
  json += "\"mq\":" + String(gasValue) + ",";
  json += "\"pump\":" + String(servoActive ? 1 : 0) + ",";
  json += "\"fan\":" + String(stepperActive ? 1 : 0);
  json += "}";
  server.send(200, "application/json", json);
}

void handleControl() {
  if (!server.hasArg("target") || !server.hasArg("state")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  String target = server.arg("target");
  String state = server.arg("state");

  if (target == "fan") {
    stepperActive = (state == "on");
  } else if (target == "pump") {
    servoActive = (state == "on");
    servo.write(servoActive ? 180 : 90);
  } else {
    server.send(404, "text/plain", "Unknown target");
    return;
  }
  server.send(200, "text/plain", "OK");
}

// ------------------- Setup & Loop -------------------
void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.println("\nConnected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  ds.begin();
  dht.begin();
  servo.attach(SERVO_PIN);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  digitalWrite(DIR_PIN, HIGH);

  server.on("/", handleRoot);
  server.on("/sensor", handleSensor);
  server.on("/control", handleControl);
  server.begin();
}

void loop() {
  unsigned long now = millis();
  if (now - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now;
    readSensors();
  }
  controlActuators();
  server.handleClient();
}
