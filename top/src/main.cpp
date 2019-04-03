
#include "Adafruit_BME680.h"
#include "Adafruit_CCS811.h"
#include </home/user/auth.h> // see auth.h.sample
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>

#define PIN_MOTION 12
#define PIN_RESET_CCS811 0

#define V_OUT_TEMP 1
#define V_OUT_CO2 2
#define V_OUT_TVOC 3
#define V_OUT_MOTION 4

#define V_OUT_TEMPERATURE 6
#define V_OUT_PRESSURE 7
#define V_OUT_HUMIDITY 8
#define V_OUT_GAS 9

#define V_BRIDGE_OUT_MOVE 30

#define MOVEMENT_WAIT_MS 90000

Adafruit_CCS811 ccs;
BlynkTimer timer;
WidgetBridge lightBridgeMove(V_BRIDGE_OUT_MOVE);

Adafruit_BME680 bme;

unsigned volatile long lastMove = 0;
boolean isMove = false;
unsigned long lastRead = 0;

BLYNK_CONNECTED() {
  lightBridgeMove.setAuthToken(BLYNK_TOKEN_LIGHT);
  Blynk.syncAll();
}

void tryLogMove() {
  long diff = (millis() - lastMove);
  bool newIsMove = lastMove > 0 && diff < MOVEMENT_WAIT_MS;

  if (newIsMove != isMove) {
    isMove = newIsMove;
    int val = isMove ? 1024 : 0;
    Blynk.virtualWrite(V_OUT_MOTION, val);
    lightBridgeMove.virtualWrite(V_BRIDGE_OUT_MOVE, val);
  }
}

void onMove() { lastMove = millis(); }

void doSubmitSensorReadings() {
  if (!ccs.available()) {
    return;
  }

  if (bme.performReading()) {
    Blynk.virtualWrite(V_OUT_TEMPERATURE, bme.temperature);
    Blynk.virtualWrite(V_OUT_PRESSURE, 0.7500616827 * bme.pressure);
    Blynk.virtualWrite(V_OUT_HUMIDITY, bme.humidity);
    Blynk.virtualWrite(V_OUT_GAS, -bme.gas_resistance / 1000.0);

    ccs.setEnvironmentalData(bme.humidity, bme.temperature);

    if (!ccs.readData()) {
      lastRead = millis();
      double co2 = ccs.geteCO2();
      double tvoc = ccs.getTVOC();
      Blynk.virtualWrite(V_OUT_CO2, co2);
      Blynk.virtualWrite(V_OUT_TVOC, tvoc);
    }
  }
}

void initCcs811() {
  Serial.println("CCS811 test");

  while (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    delay(1000);
  }
  ccs.enableInterrupt();
  ccs.setDriveMode(CCS811_DRIVE_MODE_10SEC);
  Serial.println("CCS811 OK!");
}

void initBme680() {
  if (bme.begin()) {
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  } else {
    Serial.println("Failed to start BME680");
  }
}

void doCheckCcs811() {
  // CCS811 appears to be unstable, need to reset if hands
  if (lastRead > 0 && millis() - lastRead > 60000) {
    lastRead = millis();
    Serial.println("RESET");
    // ccs.SWReset();
    digitalWrite(PIN_RESET_CCS811, LOW);
    Blynk.virtualWrite(21, 1024);
    delay(1500);
    digitalWrite(PIN_RESET_CCS811, HIGH);
    delay(10);

    initCcs811();
  } else {
    Blynk.virtualWrite(21, 0);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_MOTION, INPUT);
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  initBme680();
  initCcs811();

  attachInterrupt(digitalPinToInterrupt(PIN_MOTION), onMove, RISING);

  Blynk.begin(BLYNK_TOKEN_TOP, WIFI_SSID, WIFI_PASSWORD);
  timer.setInterval(1000, doSubmitSensorReadings);
  timer.setInterval(10000, doCheckCcs811);
  timer.setInterval(50, tryLogMove);

  ESP.wdtDisable();
}

void loop() {
  timer.run();
  Blynk.run();
  if (Blynk.connected()) {
    ESP.wdtFeed();
  }
}
