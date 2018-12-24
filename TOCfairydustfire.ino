//MIT license, except where code from other projects was borrowed and other licenses might apply

// SK6812 support using https://github.com/coryking/FastLED 
#undef FASTLED_RGBW
// DMA on Pin 3 / RX of ESP8266/ESP32 using https://github.com/coryking/FastLED
#define FASTLED_ESP8266_DMA
//disable interrupts during LED.show() so wdt does not kill us
#define FASTLED_ALLOW_INTERRUPTS 0
#undef DEBUG_ESP_PORT
#include <FastLEDESP32.h>
#include <math.h>
#include <EEPROM.h>
#include <vector>
#include <ESP8266WiFi.h>


#define EEPROMBEGIN EEPROM.begin(16);

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

///// Requirements:
/// * newest FastLED from GitHub (master branch > 3.1.6)
/// * PJRC Audio (included in Teensyduino)
/// * Teensy 3.2
/// * https://github.com/PaulStoffregen/WS2812Serial
///

// Data pin that led data will be written out over
#define WS2812_PIN 3 //D1
// #define LED_PIN 2  //D4
#define LED_PIN 4 // RX?
#define BUTTON_PIN 0 //D3 Button
#define MOSFET_PIN 5 //switches WS2812 strip power low-side


#define BATTERY_TEST_AIN A0

#define NUM_LEDS (93+27)
#define BUTTON_DEBOUNCE  500
#define LIGHT_THRESHOLD (500*3300/4096)  //500mV
#define LIGHT_DEBOUNCE 50000

#define EEPROM_CURRENT_VERSION 0
#define EEPROM_ADDR_VERS 0
#define EEPROM_ADDR_CURANIM 1

#define FFT_SIZE 256

CRGB leds_[NUM_LEDS];

bool is_dark_=true;
int32_t dark_count_=0;
uint16_t light_level=0;

#include "animations.h"

AnimationBlackSleepESP8266 anim_sleep_off(BUTTON_PIN);
AnimationConfetti anim_confetti;
AnimationTOCFairyDustFire anim_toc;
// AnimationTOCFairyDustFire2 anim_toc2;
AnimationTOCFairyDustLandingRing anim_landing;
AnimationRainbowGlitter anim_rainbow;
AnimationFireRing anim_firering;
AnimationBatteryIndicator anim_battery_indicator;

std::vector<BaseAnimation*> fairy_dust_list =
  {
    &anim_landing,
    &anim_toc,
    &anim_firering,
  };
AutoSwitchAnimationCollection fairy_dust_auto_switch(10*60*1000, fairy_dust_list);

std::vector<BaseAnimation*> animations_list_=
  {
    &fairy_dust_auto_switch,
    &anim_landing,
    &anim_toc,
    &anim_firering,
    &anim_confetti,
    &anim_rainbow,
    &anim_sleep_off,
    &anim_battery_indicator,
  };

#define NUM_ANIM animations_list_.size()
uint8_t animation_current_= NUM_ANIM-1; //start with battery indicator



// This function sets up the leds and tells the controller about them
void setup()
{
#ifdef USE_PJRC_AUDIO
  AudioMemory(12);
  delay(2000);
#endif

  //Serial.begin(115200);
  Serial.end();

  //wifi / BLE off
  WiFi.mode(WIFI_OFF);

  FastLED.addLeds<WS2812,WS2812_PIN, GRB>(leds_,NUM_LEDS);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BATTERY_TEST_AIN, INPUT);
#ifdef USE_PJRC_AUDIO
  pinMode(MICROPHONE_AIN, INPUT);
#endif

  //init animation
  EEPROMBEGIN
  load_from_EEPROM();
  animations_list_[animation_current_]->init();
}



void save_to_EEPROM()
{
  if (EEPROM.read(EEPROM_ADDR_VERS) != EEPROM_CURRENT_VERSION)
    EEPROM.write(EEPROM_ADDR_VERS, EEPROM_CURRENT_VERSION);
  if (animation_current_ != EEPROM.read(EEPROM_ADDR_CURANIM))
    EEPROM.write(EEPROM_ADDR_CURANIM, animation_current_);
}

void load_from_EEPROM()
{
  if (EEPROM.read(EEPROM_ADDR_VERS) != EEPROM_CURRENT_VERSION)
    return;

  animation_current_ = EEPROM.read(EEPROM_ADDR_CURANIM) % NUM_ANIM;
}

void task_check_battery()
{
  static millis_t next_check=0;

  //ignore overflow for now, as the only thing that happens that we check all the time until 20s have passed.
  if (millis() < next_check)
    return;

  next_check=millis()+1000*20;

  //full charge with 4.2V at about 930 (out of 1024)
  //empty with 2.85V at about 650 (out of 1024)
  //battery charge thus ranges from 0 .. 255 to indicate charge
  uint16_t const batt_empty = 650;
  uint16_t const esp8266_minimum_operating_voltage = 680; //3.0V
  uint16_t const batt_full = 930;
  uint16_t adc_reading = analogRead(BATTERY_TEST_AIN);
  uint16_t batt_charge_byte = min(0xff,(max(batt_empty,adc_reading)-batt_empty) * 0xff / (batt_full-batt_empty));
  anim_battery_indicator.setBatteryChargeLevel0to255(batt_charge_byte);

  if (adc_reading < batt_empty/2)
  {
    //no battery connected
    return;
  }

  if (adc_reading <= esp8266_minimum_operating_voltage)
  {
    //switch to power saving
    animation_switch_to_off_if_in_list();
    return;
  }
}

void task_check_lightlevel()
{
  light_level = 0; //TODO: read via SPI
  if (light_level > LIGHT_THRESHOLD)
  {
    //assume daylight
    if (dark_count_ < LIGHT_DEBOUNCE) {
      dark_count_++;
    }
  } else {
    //assume darkness
    if (dark_count_ > -LIGHT_DEBOUNCE) {
      dark_count_--;
    }
  }
  if (dark_count_ <= -LIGHT_DEBOUNCE)
  {
    is_dark_ = true;
  } else if (dark_count_ >= LIGHT_DEBOUNCE)
  {
    is_dark_ = false;
  }
}

inline void task_sample_mic()
{
#ifdef USE_PJRC_AUDIO
  //done by PJRC Audio
#else
  //TODO
#endif
}

void task_animate_leds()
{
  static millis_t next_run=0;

  //ignore overflow for now, as the only thing that happens that we continue immediately once.
  if (millis() < next_run)
    return;

  //run current animation
  millis_t delay_ms = animations_list_[animation_current_]->run();

  //enable or disable LED-strip power
  digitalWrite(MOSFET_PIN, (areAllPixelsBlack())? LOW : HIGH);

  // Show the leds
  FastLED.show();
  next_run=millis()+delay_ms;
}

void animation_switch_next()
{
  animation_current_++;
  animation_current_%=NUM_ANIM;
  save_to_EEPROM();
  animations_list_[animation_current_]->init();
}

void animation_switch_to_off_if_in_list()
{
  for (unsigned int c=0; c < NUM_ANIM; c++)
  {
    if (animations_list_[c] == &anim_sleep_off)
    {
      animation_current_ = c;
      save_to_EEPROM();
      animations_list_[animation_current_]->init();
      break;
    }
  }
}

void task_check_button()
{
  static uint16_t btn_count_=0;
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    if (btn_count_ < BUTTON_DEBOUNCE+1)
    {
      btn_count_++;
      digitalWrite(LED_PIN,HIGH);
    } else {
      digitalWrite(LED_PIN,LOW);
    }
  } else {
    btn_count_=0;
  }

  if (btn_count_ == BUTTON_DEBOUNCE)
  {
    animation_switch_next();
  }
}

void task_heartbeat()
{
  static uint16_t hbled=0;
  digitalWrite(LED_PIN,(hbled < 0xF00)?HIGH:LOW);
  hbled++;
  hbled %= 0x1000;
}

void loop() {
  task_heartbeat();
  task_check_button();
  task_check_battery();
  //task_check_lightlevel();
  //task_sample_mic();
  task_animate_leds();
}
