# TOC-FairyDust-LandingPad
TrainOperationCenter Landing Pad for minature fairy dust


## Hardware

- external Button
- ESP8266 WeMos D1 mini Pro
- WS2812 LED Ring with additional LED-strip
- 3850mAh LiIon Battery of former Kindle
- IRLU8743PbF MosFET for power saving. Low-Side disables WS2812 power.
- (planned)  I2C illumination sensor

## ESP8266 Pins

- GPIO0 : Button to switch animation or to OFF
- GPIO3 (RX) : WS2812 Data
- GPIO4 : onboard LED
- GPIO5 : MosFET
- A0 : onboard voltage divider connected to battery

## Software

- hacked FastLED with ESP8266 DMA Transfer from https://github.com/coryking/FastLED merged into latest FastLED
- animations.h shared with WS2812AudioFFT_music_ducks
- compile with arduino or platformIO
