#include "Adafruit_SHT31.h"
#include </home/user/auth.h> #see auth.h.sample
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>

#define BLYNK_PRINT Serial

#define TUBE_PARAM 151.0

#define GEIGER_COUNTER_PIN 12

#define V_OUT_TEMPERATURE 1
#define V_OUT_HUMIDITY 2
#define V_OUT_GEIGER_COUNTER_USV 3

volatile unsigned long counts;
unsigned long previousMillis;
double usv;

BlynkTimer timer;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void onPulse() {
  Serial.print("#");
  counts++;
}

void doSubmitSht31() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  Blynk.virtualWrite(V_OUT_TEMPERATURE, t);
  Blynk.virtualWrite(V_OUT_HUMIDITY, h);
}

void doSubmitGeigerCounter() {
  unsigned long currentMillis = millis();

  unsigned long diff = currentMillis - previousMillis;
  double multiplier = 60000. / ((double)diff);
  double cpm = ((double)counts) * multiplier;

  usv = (double)cpm / TUBE_PARAM;

  Blynk.virtualWrite(V_OUT_GEIGER_COUNTER_USV, usv);

  previousMillis = currentMillis;
  counts = 0;
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(BLYNK_TOKEN_OUTDOOR, WIFI_SSID, WIFI_PASSWORD);

  if (sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
    timer.setInterval(5347, doSubmitSht31);
  } else {
    Serial.println("Couldn't find SHT31");
  }

  attachInterrupt(digitalPinToInterrupt(GEIGER_COUNTER_PIN), onPulse, RISING);
  previousMillis = millis();
  timer.setInterval(26183, doSubmitGeigerCounter);
}

void loop() {
  timer.run();
  Blynk.run();
}
