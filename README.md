# Blynk-based smart home.
This repository contains several custom smart home modules based on [Blynk](http://www.blynk.cc/), NodeMCU (ESP8266) and Arduino.

### Indoor module
- Electric power (using [SCT-013](https://openenergymonitor.org/emon/buildingblocks/ct-sensors-interface), Rb=22);
- Light level (using a [photocell](https://learn.adafruit.com/photocells/using-a-photocell));
- Temperature, atmospheric pressure, relative humidity, air quality (using [BME680](https://www.adafruit.com/product/3660) I2C interface);
- Air quality using MQ135 sensor;
- Door open/closed notification (using a [magnetic contat switch](https://www.adafruit.com/product/375));
- Door bell notification.

### Outdoor module
- [NGMC-V1 Geiger counter module](https://www.aliexpress.com/item/Free-shipping-Newer-Upgrade-Assembled-DIY-Geige-Geiger-Counter-Kit-Nuclear-Radiation-Detector-GM-Tube-connector/32336592385.html?spm=2114.13010608.0.0.yJGWUw);
- Temperature, relative humidity (using [SHT31](https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout/overview) I2C interface).

### Light module
- Motion detection (using [HC-SR501](http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/arduino-hc-sr501-motion-sensor-tutorial/));
- [NeoPixel](https://www.adafruit.com/category/168) controller.
