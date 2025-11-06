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
float slope = 3.8333;  // calibration slope (m)
float intercept = -9.5; // calibration intercept (c)
String co2;
String sug;
String res;

const float RL_VALUE = 5.0;    // load resistor value in kΩ
const float RO = 10.0;         // fixed Ro in kΩ (simulation)
const int ADC_MAX = 4095;      // ADC resolution max (ESP32 = 4095). Set to 1023 for Arduino UNO.
// ---------------------------

// curves: { log10(ppm_point), log10(Rs/Ro_at_point), slope }
float LPGCurve[3]     = {2.3, 0.21, -0.47};
float COCurve[3]      = {2.3, 0.72, -0.34};
float SmokeCurve[3]   = {2.3, 0.53, -0.44};
float AlcoholCurve[3] = {2.3, 0.28, -0.47};

// ---------- helpers ----------
float MQResistanceCalculation(int raw_adc) {
  if (raw_adc <= 0) return -1.0; // invalid
  if (raw_adc >= ADC_MAX) return 1e9; // extremely small sensor voltage -> huge resistance
  // Rs = RL * (ADC_MAX - raw) / raw
  return ( RL_VALUE * ( (float)(ADC_MAX - raw_adc) / (float)raw_adc ) );
}

float MQRead(int mq_pin) {
  float rs = 0.0;
  const int samples = 5;
  for (int i = 0; i < samples; ++i) {
    int raw = analogRead(mq_pin);
    float r = MQResistanceCalculation(raw);
    if (r < 0) return -1.0;
    rs += r;
    delay(50);
  }
  return rs / (float)samples;
}

bool isFinitePositive(float v) {
  return isfinite(v) && (v > 0.0);
}

float MQGetPercentageFloat(float rs_ro_ratio, float *pcurve) {
  // returns ppm as float. Uses log10 consistently with pcurve entries
  if (!isFinitePositive(rs_ro_ratio)) return NAN;
  float log_ratio = log10(rs_ro_ratio);
  float x = ( (log_ratio - pcurve[1]) / pcurve[2] ) + pcurve[0];
  // result = 10^x
  return pow(10.0, x);
}










// Function to measure distance from sonar (in cm)
float getDistance() {
  digitalWrite(TRIG2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);

  long duration = pulseIn(ECHO2, HIGH);
  // Convert time to distance (sound speed = 0.034 cm/μs)
  float distance = duration * 0.034 / 2.0;
  return distance;
}

// Convert distance (cm) to °Brix
float distanceToBrix(float distance) {
  float brix = slope * distance + intercept;
  if (brix < 0) brix = 0;  // clamp for sanity
  return brix;
}


#define READ_INTERVAL 1000
#define SERVO_INTERVAL 15

float dsTemp = 0.0, dhtTemp = 0.0, dhtHum = 0.0,brix=0.0;
int distance1 = 0, distance2 = 0, gasValue = 0;
double ppm =0;
bool servoActive = false;
bool stepperActive = false;
double a = 6.18898729177493e-16;
double b = 4.856191982;
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
  // digitalWrite(TRIG2, LOW); delayMicroseconds(2);
  // digitalWrite(TRIG2, HIGH); delayMicroseconds(10);
  // digitalWrite(TRIG2, LOW);
  // long duration2 = pulseIn(ECHO2, HIGH, 30000);
  // distance2 = (duration2 == 0) ? 999 : (duration2 / 58.2);
    float distance = getDistance();        // cm
     brix = distanceToBrix(distance); // °Brix
     if(brix<10){
      sug = "FEND";
     }
     else{
      sug = "FON";
     }

  gasValue = MQRead(GAS_PIN);


 float rs = gasValue;
  float ratio = NAN;
  if (isFinitePositive(rs) && RO > 0.0) 
      ratio = rs / RO;
  if (!isFinitePositive(rs)) {
    Serial.println(F("Rs: invalid"));
    Serial.println(F("Rs/Ro: --"));
  } else {
    Serial.print(F("Rs (kΩ): "));
    Serial.println(rs, 3);
    Serial.print(F("Rs/Ro Ratio: "));
    Serial.println(ratio, 4);
  }

  // compute ppm for each gas (float)
   ppm = MQGetPercentageFloat(ratio, AlcoholCurve);

  if(ppm<1000){
      co2 = "COFF";
  }
  else{
    co2 = "CON";
  }



  // if(co2 == "CON" && sug == "FEND")
  // {
  //   res = "Fermentation Complete";
  // }
  // else if(co2 == "COFF" && sug == "FEND")
  // {
  //   res = "Farmentation off, Check CO2, Sugar level ok";
  // }
  // else if(co2 == "CON" && sug == "FON")
  // {
  //     res = "Farmentation Ongoing, High Sugar";
  // }
  // else
  // {
  //   res = "Farmentation OFF";
  // }












 
 

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
      Sonar2: <span id='s2'>0</span> °brix<br>
      MQ: <span id='mq'>0</span><br>
      Fermentation: <span id='sug'> </span><br>
      CO2: <span id='co2'> </span><br>
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
          document.getElementById('sug').innerText = d.sug;
          document.getElementById('co2').innerText = d.co2;

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
  json += "\"sonar2\":" + String(brix) + ",";
  json += "\"mq\":" + String(ppm) + ",";
  json += "\"pump\":" + String(servoActive ? 1 : 0) + ",";
  json += "\"fan\":" + String(stepperActive ? 1 : 0) + ",";
  json += "\"speed\":" + String(stepSpeed);
  json += ",\"sug\":\"" + sug + "\"";
  json += ",\"co2\":\"" + co2 + "\"";

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
