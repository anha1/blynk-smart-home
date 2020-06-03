#include </home/user/auth.h> // see auth.h.sample
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#define BLYNK_PRINT Serial

#define VPIN_LEVEL 10
#define VPIN_WATERING 11
#define VPIN_ERROR 12
#define VPIN_SINGE_WATERING_MINUTES 13
#define VPIN_SINGE_WATERING_HOURS 14
#define VPIN_SINGE_WATERING_HOURS_MINUTES 15

#define PIN_SENSOR_POWER 32
#define PIN_PUMP 33

BlynkTimer timer;

int wateringThreshold = 500;
int wateringDurationSeconds = 3;
int cooldownMinutes = 10;
boolean isEnabled = false;
boolean isInitialized = false;

unsigned long lastWatering = 0;

int level = 600;

boolean isInvalidReading = false;
int invalidReadingCount = 0;
boolean isWatering = false;

hw_timer_t *watchdogTimer = NULL;

BLYNK_CONNECTED() { Blynk.syncAll(); }

void pump(boolean isEnable) {
  digitalWrite(PIN_PUMP, isEnable ? LOW : HIGH); // need to be disabled if no relay input
}

void reboot() {
  Serial.print("Rebooting .. \n\n");
  esp_restart();
}

void report () {
  if (isWatering) {
    Serial.println("no report: isWatering");
    return;
  }
  
  digitalWrite(PIN_SENSOR_POWER, HIGH);
  delay(300);  
  long readVal = analogRead(A0);
  digitalWrite(PIN_SENSOR_POWER, LOW);
  if (readVal == 0 || readVal == 4095) {
    pump(false);
    Serial.print("INVALID READING: ");
    Serial.println(readVal);
    invalidReadingCount ++;
    if (!isInvalidReading && invalidReadingCount > 3) {
      Serial.println("INVALID READING STATE REACHED");
      isInvalidReading = true;
      Blynk.virtualWrite(VPIN_ERROR, 1023);
    }
    if (invalidReadingCount > 100) {
      Serial.println("INVALID READING: REBOOT");
      reboot();
    }
    return;
  } else {
    Blynk.virtualWrite(VPIN_ERROR, 0);
    isInvalidReading = false;
    invalidReadingCount = 0;
  }
  

  Serial.println("report");
  level = 1000.0*(4095.0 - (float) readVal)/4095.0;
  
  Serial.print("level: ");
  Serial.println(level);
  Blynk.virtualWrite(VPIN_LEVEL, level);
}

void act() {
  if (isWatering) {
    Serial.println("no action: isWatering");
    return;
  }

  unsigned long now = millis();

  unsigned long sinceLastWateringMinutes = (now - lastWatering) / 60000;
  Serial.print("sinceLastWateringMinutes: ");
  Serial.println(sinceLastWateringMinutes);
  Blynk.virtualWrite(VPIN_SINGE_WATERING_MINUTES, sinceLastWateringMinutes);
  //to display hh:mm
  int h = sinceLastWateringMinutes / 60;
  int m = sinceLastWateringMinutes - h * 60;
  Blynk.virtualWrite(VPIN_SINGE_WATERING_HOURS, h);
  Blynk.virtualWrite(VPIN_SINGE_WATERING_HOURS_MINUTES, m);
  Blynk.virtualWrite(VPIN_WATERING, 0);

  if (sinceLastWateringMinutes < cooldownMinutes) {
    Serial.println("no action: cooldownMinutes");
    return;
  }

  if (!isInitialized) {
    Serial.println("no action: isInitialized = false");
    return;
  }  

  if (!isEnabled) {
    Serial.println("no action: isEnabled = false");
    return;
  }

  if (isInvalidReading) {
    Serial.println("no action: invalid reading");
    return;
  }

  if (level > wateringThreshold) {
    Serial.println("no action: wateringThreshold");
    return;
  } 

  Serial.println("WATERING START!");
  isWatering = true;
  Blynk.virtualWrite(VPIN_WATERING, 1023);
  lastWatering = millis();
  pump(true);
}

void control () {
    if (!isWatering) {
      return;
    }
    unsigned long now = millis();
    unsigned long sinceLastWateringSeconds = (now - lastWatering) / 1000;
    
    if (sinceLastWateringSeconds > wateringDurationSeconds) {
      Serial.println("WATERING END!");
      pump(false);      
      isWatering = false;
    }
}

BLYNK_WRITE(V0) {
  wateringThreshold = min(1000, (max(0, param[0].asInt())));
  Serial.print("wateringThreshold: ");
  Serial.println(wateringThreshold);
  report();
}

BLYNK_WRITE(V1) {
  wateringDurationSeconds = min(30, (max(1, param[0].asInt())));
  Serial.print("wateringDurationSeconds: ");
  Serial.println(wateringThreshold);
  report();
}

BLYNK_WRITE(V2) {
  cooldownMinutes = min(60, (max(1, param[0].asInt())));
  Serial.print("cooldownMinutes: ");
  Serial.println(cooldownMinutes);
  report();
}

BLYNK_WRITE(V3) {
  isEnabled = param[0].asInt() == 1;
  Serial.print("isEnabled: ");
  Serial.println(isEnabled);
  report();
}

void setup() {
  pinMode(PIN_SENSOR_POWER, OUTPUT); 
  digitalWrite(PIN_SENSOR_POWER, LOW);

  pinMode(PIN_PUMP, OUTPUT); 
  pump(false);

  Serial.begin(9600);
  Serial.println("setup");

  watchdogTimer = timerBegin(0, 80, true); //timer 0 divisor 80
  timerAlarmWrite(watchdogTimer, 60000000, false); // set time in uS must be fed within this time or reboot
  timerAttachInterrupt(watchdogTimer, & reboot, true);
  timerAlarmEnable(watchdogTimer);  // enable interrupt
  
  Blynk.begin(BLYNK_TOKEN_WATERING, WIFI_SSID, WIFI_PASSWORD);
  timer.setInterval(5003, report);
  timer.setInterval(60000, act);
  timer.setInterval(1000, control);
  Serial.println("setup done");
  isInitialized = true;
  report();
}

void loop() {
   Blynk.run();  
   if (Blynk.connected()) {
    timerWrite(watchdogTimer, 0); 
    timer.run();
   } else {
    Serial.println("CONNECTION LOST!");
    pump(false);
   }   
}
