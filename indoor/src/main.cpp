#include "Adafruit_BME680.h"
#include </home/user/auth.h> // see auth.h.sample
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
#include <adc.h>

#define BLYNK_PRINT Serial
#define I_RATIO (2000.0 / 22.0) * ((ADC_VCC) / ADC_MAX);
#define PULSE_EVENTS_DURATION_MS 46000

#define ADC_MQ135_PIN 5
#define ADC_LIGHT_PIN 6
#define ADC_POWER_PIN 7

#define BELL_PIN 10
#define DOOR_OPEN_PIN 2
#define ACTIVITY_PIN 16

#define V_OUT_POWER 1
#define V_OUT_LIGHT 2
#define V_OUT_MQ135 3
#define V_OUT_DOOR 4
#define V_OUT_BELL 5
#define V_OUT_TEMPERATURE 6
#define V_OUT_PRESSURE 7
#define V_OUT_HUMIDITY 8
#define V_OUT_GAS 9

double offsetI = 2048.;
BlynkTimer timer;
Adafruit_BME680 bme; // I2C

unsigned volatile long lastBell = 0;
unsigned volatile long lastDoorOpen = 0;

double calcRms(int samples) {
  double sumI = 0;
  for (unsigned int n = 0; n < samples; n++) {
    double sampleI = readSpiAdc(ADC_POWER_PIN);
    offsetI = (offsetI + (sampleI - offsetI) / 3000.);
    double filteredI = sampleI - offsetI;
    double sqI = filteredI * filteredI;
    sumI += sqI;
    delay(1);
  }
  double Irms = sqrt(sumI / (double)samples) * I_RATIO;
  return Irms;
}

void doSubmitPower() {
  double rms = calcRms(3000);
  double power = rms * 220.0;
  Blynk.virtualWrite(V_OUT_POWER, power);
}

void activityBegin() { digitalWrite(ACTIVITY_PIN, HIGH); }

void activityEnd() { digitalWrite(ACTIVITY_PIN, LOW); }

void doSubmitAdcSensors() {
  activityBegin();
  int lightIn = readSpiAdc(ADC_LIGHT_PIN);
  Blynk.virtualWrite(V_OUT_LIGHT, lightIn);

  int mq135 = readSpiAdc(ADC_MQ135_PIN);
  Blynk.virtualWrite(V_OUT_MQ135, mq135);
  activityEnd();
}

int getPulseEventVal(unsigned long last) {
  unsigned long now = millis();
  if (last > 0 && now - last < PULSE_EVENTS_DURATION_MS) {
    return 1024;
  } else {
    return 0;
  }
}

void doSubmitPulseEvents() {
  activityBegin();
  unsigned long now = millis();
  if (digitalRead(DOOR_OPEN_PIN) == LOW) {
    lastDoorOpen = now;
  }

  if (digitalRead(BELL_PIN) == HIGH) {
    lastBell = now;
  }

  Blynk.virtualWrite(V_OUT_DOOR, getPulseEventVal(lastDoorOpen));
  Blynk.virtualWrite(V_OUT_BELL, getPulseEventVal(lastBell));
  activityEnd();
}

void doSubmitBme680() {
  activityBegin();
  if (bme.performReading()) {
    Blynk.virtualWrite(V_OUT_TEMPERATURE, bme.temperature);
    Blynk.virtualWrite(V_OUT_PRESSURE, 0.7500616827 * bme.pressure);
    Blynk.virtualWrite(V_OUT_HUMIDITY, bme.humidity);
    Blynk.virtualWrite(V_OUT_GAS, -bme.gas_resistance / 1000.0);
  }
  activityEnd();
}

void onDoorOpen() { lastDoorOpen = millis(); }

void onBell() { lastBell = millis(); }

void setup() {
  initSpiAdc();
  pinMode(DOOR_OPEN_PIN, INPUT);
  pinMode(BELL_PIN, INPUT);
  pinMode(ACTIVITY_PIN, OUTPUT);
  Serial.begin(9600);

  delay(1500);
  attachInterrupt(digitalPinToInterrupt(DOOR_OPEN_PIN), onDoorOpen, FALLING);
  attachInterrupt(digitalPinToInterrupt(BELL_PIN), onBell, RISING);

  Blynk.begin(BLYNK_TOKEN_INDOOR, WIFI_SSID, WIFI_PASSWORD);

  timer.setInterval(5741, doSubmitPower);
  timer.setInterval(5743, doSubmitAdcSensors);
  timer.setInterval(5749, doSubmitPulseEvents);

  if (bme.begin()) {
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    timer.setInterval(5783, doSubmitBme680);
  } else {
    Serial.println("Failed to start BME680");
  }
}

void loop() {
  timer.run();
  Blynk.run();
}
