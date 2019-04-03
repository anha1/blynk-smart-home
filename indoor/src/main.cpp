#include </home/user/auth.h> // see auth.h.sample
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

double offset = 2048.;
double offsetPrev = 2048.;
BlynkTimer timer;

unsigned volatile long lastBell = 0;
unsigned volatile long lastDoorOpen = 0;
unsigned int readingsPhase = 0;

double calcRms(int cycles) {
  double sum = 0;
  unsigned int samplesCount = 0;
  unsigned long stop = millis() + cycles * 20; // 20ms per cycle for 50Hz
  while (millis() < stop) {
    double sample = readSpiAdc(ADC_POWER_PIN);
    offset = (offset + (sample - offset) / 15000.);
    double filteredI = sample - offset;
    double sq = filteredI * filteredI;
    sum += sq;
    samplesCount++;
  }
  double Irms = sqrt(sum / (double)samplesCount) * I_RATIO;
  return Irms;
}

void doSubmitPower() {
  double rms = calcRms(35);
  double offsetDiff = abs(offset - offsetPrev);
  offsetPrev = offset;

  if (offsetDiff < 0.3) {
    double power = rms * 220.0;
    Blynk.virtualWrite(V_OUT_POWER, power);
  }
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

void onDoorOpen() { lastDoorOpen = millis(); }

void onBell() { lastBell = millis(); }

void doSubmitSensorReadings() {
  switch ((readingsPhase++) % 3) {
  case 0:
    doSubmitPulseEvents();
    break;
  case 1:
    doSubmitAdcSensors();
    break;
  case 2:
    doSubmitPower();
    break;
  }
}

void setup() {
  initSpiAdc();
  pinMode(DOOR_OPEN_PIN, INPUT);
  pinMode(BELL_PIN, INPUT);
  pinMode(ACTIVITY_PIN, OUTPUT);
  Serial.begin(9600);

  delay(10);
  attachInterrupt(digitalPinToInterrupt(DOOR_OPEN_PIN), onDoorOpen, FALLING);
  attachInterrupt(digitalPinToInterrupt(BELL_PIN), onBell, RISING);

  Blynk.begin(BLYNK_TOKEN_INDOOR, WIFI_SSID, WIFI_PASSWORD);
  timer.setInterval(5000, doSubmitSensorReadings);

  ESP.wdtDisable();
}

void loop() {
  timer.run();
  Blynk.run();

  if (Blynk.connected()) {
    ESP.wdtFeed();
  }
}
