#define BLYNK_PRINT Serial

#include </home/user/auth.h> // see auth.h.sample
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <BlynkSimpleEthernet.h>
#include <Ethernet.h>
#include <SPI.h>

#define NUMPIXELS 138
#define PIN_NEOPIXEL 9
#define PIN_IN_MOVEMENT 7

#define MOVEMENT_WAIT_MS 90000
#define MOVEMENT_FILTER_MS 15000

#define SWITCH_OFF 1
#define SWITCH_ON 2
#define SWITCH_AUTO 3

#define MODE_WHITE 1
#define MODE_RGB 2
#define MODE_RAINBOW 3
#define MODE_STROBE 4
#define MODE_WARM_WHITE 5
#define MODE_STRIPES 6

#define MOTION_OUT_VPIN 21

uint32_t black;
uint32_t white;
uint32_t warmWhite;
BlynkTimer timer;

Adafruit_NeoPixel strip =
    Adafruit_NeoPixel(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
int timerId;

// RGB
int r = 0;
int g = 150;
int b = 255;

int p = 0;

int animationDelayMs = 10;
int br = 255;           // brightness
int mod = MODE_RAINBOW; // mode
int swit = SWITCH_OFF;  // switch
float step = 100;
float speed = 100;
int strobeMicros = 1; // strobe duration
int numRunners = 7;

unsigned volatile long lastMove = 0;
boolean isMove = false;

void logMove() { Blynk.virtualWrite(MOTION_OUT_VPIN, isMove ? 1024 : 0); }

void onMove() {
  unsigned long now = millis();
  if (abs(now - (lastMove + MOVEMENT_WAIT_MS)) > MOVEMENT_FILTER_MS) {
    lastMove = now;
  }
}

boolean calcIsLight() {
  if (digitalRead(PIN_IN_MOVEMENT) == HIGH) {
    onMove();
  }
  long diff = (millis() - lastMove);
  isMove = lastMove > 0 && diff < MOVEMENT_WAIT_MS;
  boolean isLight = Blynk.connected() &&
                    ((swit == SWITCH_ON) || ((swit == SWITCH_AUTO) && isMove));
  return isLight;
}

uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

BLYNK_CONNECTED() { Blynk.syncAll(); }

BLYNK_WRITE(V0) {
  r = param[0].asInt();
  g = param[1].asInt();
  b = param[2].asInt();
}

BLYNK_WRITE(V2) { br = param.asInt(); }

BLYNK_WRITE(V3) {
  mod = param.asInt();
  calcIsLight();
}

BLYNK_WRITE(V4) { step = param.asInt(); }

BLYNK_WRITE(V5) { strobeMicros = param.asInt(); }

BLYNK_WRITE(V6) { speed = param.asInt(); }

BLYNK_WRITE(V7) { numRunners = param.asInt(); }

BLYNK_WRITE(V8) {
  swit = param.asInt();
  calcIsLight();
}

void draw() {
  if (!calcIsLight()) {
    strip.setBrightness(0);
    strip.show();
    return;
  }

  p++;

  strip.setBrightness(br);

  uint32_t rgb = strip.Color(r, g, b);

  int partitions = (NUMPIXELS / numRunners);

  float stepMod = ((float)step) * 0.0777 / ((float)NUMPIXELS);
  float speedMod = ((float)speed) * 0.0111 * ((float)animationDelayMs) / 100.;

  switch (mod) {
  case MODE_WHITE:
    for (int i = 0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, white);
    }
    break;
  case MODE_RGB:
    for (int i = 0; i < NUMPIXELS; i++) {
      if ((i + p) % partitions == 0) {
        strip.setPixelColor(i, rgb);
      }
    }
    break;
  case MODE_RAINBOW:
    for (int i = 0; i < NUMPIXELS; i++) {
      float phase = i * stepMod + p * speedMod;
      strip.setPixelColor(i, Wheel((int)phase & 255));
    }
    break;
  case MODE_STROBE:
    for (int i = 0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, white);
    }
    strip.show();
    delayMicroseconds(strobeMicros);
    strip.setBrightness(0);
    break;
  case MODE_WARM_WHITE:
    for (int i = 0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, warmWhite);
    }
    break;
  case MODE_STRIPES:
    int shiftPerPartition = floor(255 / numRunners);
    for (int i = 0; i < NUMPIXELS; i++) {
      int partition = i / partitions;
      float phase = shiftPerPartition * partition + p * speedMod;
      strip.setPixelColor(i, Wheel((int)phase & 255));
    }
    break;
  }
  strip.show();
}

void setup() {

  pinMode(PIN_IN_MOVEMENT, INPUT);

  Serial.begin(9600);

  calcIsLight();

  strip.begin();
  draw();
  attachInterrupt(digitalPinToInterrupt(PIN_IN_MOVEMENT), onMove, RISING);

  black = strip.Color(0, 0, 0);
  white = strip.Color(255, 255, 255);
  warmWhite = strip.Color(255, 103, 23);

  timer.setInterval(16, draw); // 60fps

  timer.setInterval(9000, logMove);

  delay(10);

  Blynk.begin(BLYNK_TOKEN_LIGHT);
}

void loop() {
  Blynk.run();
  timer.run();
}
