#include <Arduino.h>
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

// ------------------- Debug / LED -------------------
#define DEBUG_LED 2  // Built-in LED (ESP32 GPIO2)

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

int stepSpeed = 30; // 0..100 (default 30)
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

  // Sonar 1
  digitalWrite(TRIG1, LOW); delayMicroseconds(2);
  digitalWrite(TRIG1, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  long duration1 = pulseIn(ECHO1, HIGH, 30000);
  distance1 = (duration1 == 0) ? 999 : (duration1 / 58.2);

  // Sonar 2
  digitalWrite(TRIG2, LOW); delayMicroseconds(2);
  digitalWrite(TRIG2, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  long duration2 = pulseIn(ECHO2, HIGH, 30000);
  distance2 = (duration2 == 0) ? 999 : (duration2 / 58.2);

  gasValue = analogRead(GAS_PIN);
}

// ------------------- Actuator Logic -------------------
void controlActuators() {
  unsigned long now_us = micros();
  unsigned long now_ms = millis();

  // Example condition to run stepper: dsTemp > 100 (keeps original logic)
  stepperActive = dsTemp > 100;

  // map stepSpeed 0..100 to pulse interval in microseconds
  // Higher stepSpeed -> smaller interval -> faster pulses
  unsigned long interval = map(stepSpeed, 0, 100, 2000, 200);  // microseconds

  if (stepperActive && (now_us - lastStepTime >= interval)) {
    lastStepTime = now_us;

    // STEP pulse for A4988
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2); // minimum HIGH pulse
    digitalWrite(STEP_PIN, LOW);

    // Toggle debug LED to visualize pulses
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }

  // Debug Serial: print stepper state and interval every 500ms
  static unsigned long lastDebug = 0;
  if (now_ms - lastDebug > 500) {
    lastDebug = now_ms;
    if (stepperActive) {
      Serial.print("[Stepper] Speed=");
      Serial.print(stepSpeed);
      Serial.print(" | Interval=");
      Serial.print(interval);
      Serial.println(" us");
    } else {
      Serial.println("[Stepper] OFF");
    }
  }

  // Servo control based on sonar distance1 (existing logic)
  if (distance1 < 20) {
    servoActive = true;
    static int pos = 0, dir = 1;
    if (now_ms - lastServoTime >= SERVO_INTERVAL) {
      lastServoTime = now_ms;
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
  String html = R"rawliteral(
  <!DOCTYPE html><html>
  <head>
    <title>Fermentation Dashboard</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
      body { font-family: Arial; background:#f2f2f2; text-align:center; padding:10px;}
      .card { background:white; border-radius:12px; padding:20px; margin:10px auto; width:320px;
              box-shadow:0 2px 6px rgba(0,0,0,0.2);}
      .on {color:green; font-weight:bold;}
      .off {color:red; font-weight:bold;}
      input[type=range] { width: 90%; }
    </style>
  </head>
  <body>
    <h1>Fermentation Monitoring</h1>
    <div class='card'>
      <h3>Sensor Readings</h3>
      DS18B20: <span id='ds'>0</span>°C<br>
      DHT22 Temp: <span id='dt'>0</span>°C<br>
      Humidity: <span id='dh'>0</span>%<br>
      Sonar1: <span id='s1'>0</span> cm<br>
      Sonar2: <span id='s2'>0</span> cm<br>
      MQ: <span id='mq'>0</span><br>
    </div>

    <div class='card'>
      <h3>Actuators</h3>
      Pump: <span id='pump'>OFF</span><br>
      Fan (Stepper): <span id='fan'>OFF</span><br><br>

      <h4>Stepper Speed Control</h4>
      <input type="range" id="spd" min="0" max="100" value="30" oninput="setSpeed(this.value)">
      <div>Speed: <span id="spdVal">30</span></div>
    </div>

    <script>
      async function setSpeed(v){
        document.getElementById("spdVal").innerText = v;
        // call control endpoint to set speed and also ensure fan ON
        try {
          await fetch(`/control?target=fan&state=on&speed=${v}`);
        } catch(e) {
          console.log("Error setting speed", e);
        }
      }

      async function updateData(){
        try {
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

          // update slider with backend speed value (keeps UI in sync)
          document.getElementById("spdVal").innerText = d.speed;
          document.getElementById("spd").value = d.speed;
        } catch(e) {
          console.log("updateData error", e);
        }
      }

      setInterval(updateData, 1000);
      window.onload = updateData;
    </script>
  </body>
  </html>
  )rawliteral";
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
  json += "\"fan\":" + String(stepperActive ? 1 : 0) + ",";
  json += "\"speed\":" + String(stepSpeed);
  json += "}";
  server.send(200, "application/json", json);
}

// ------------------- Control API -------------------
void handleControl() {
  if (!server.hasArg("target") || !server.hasArg("state")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  String target = server.arg("target");
  String state = server.arg("state");

  if (target == "fan") {
    // Optional speed parameter
    if (server.hasArg("speed")) {
      int sp = server.arg("speed").toInt();
      if (sp < 0) sp = 0;
      if (sp > 100) sp = 100;
      stepSpeed = sp;
      Serial.print("[HTTP] Speed set to: ");
      Serial.println(stepSpeed);
    }
    // state on/off controls stepperActive flag (but actuator logic also checks dsTemp)
    stepperActive = (state == "on");
    server.send(200, "text/plain", "OK");
    return;
  } else if (target == "pump") {
    servoActive = (state == "on");
    servo.write(servoActive ? 180 : 90);
    server.send(200, "text/plain", "OK");
    return;
  } else {
    server.send(404, "text/plain", "Unknown target");
    return;
  }
}

// ------------------- Setup & Loop -------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting");
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    // optional timeout to avoid infinite block (10s)
    if (millis() - startAttempt > 10000) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi not connected (continuing in AP-less mode)");
  }

  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);

  ds.begin();
  dht.begin();
  servo.attach(SERVO_PIN);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(GAS_PIN, INPUT);

  // set default direction for A4988
  digitalWrite(DIR_PIN, HIGH);

  server.on("/", handleRoot);
  server.on("/sensor", handleSensor);
  server.on("/control", handleControl);
  server.begin();

  Serial.println("HTTP server started");
}

void loop() {
  unsigned long now_ms = millis();
  if (now_ms - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now_ms;
    readSensors();
  }

  controlActuators();
  server.handleClient();
}
