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


#define PHOTORESISTOR_AIN A0
#define PHOTORESISTOR_PIN 17
#define PHOTORESISTOR_USE_ADC

#define NUM_LEDS 93
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

AnimationConfetti anim_confetti;
AnimationTOCFairyDustFire anim_toc;
AnimationRainbowGlitter anim_rainbow;

std::vector<BaseAnimation*> animations_list_=
	{&anim_toc, &anim_rainbow, &anim_confetti };

uint8_t animation_current_= 0;
#define NUM_ANIM animations_list_.size()



// This function sets up the ledsand tells the controller about them
void setup() {
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
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	pinMode(PHOTORESISTOR_AIN, INPUT);
	pinMode(PHOTORESISTOR_PIN, INPUT);
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



void task_check_lightlevel()
{
#ifdef PHOTORESISTOR_USE_ADC
#ifdef USE_PJRC_AUDIO
  if (!photoPeak.available())
    return;
  light_level = photoPeak.read()*4095;
#else
  light_level = analogRead(PHOTORESISTOR_AIN);
#endif
  if (light_level > LIGHT_THRESHOLD)
#else
  if (digitalRead(PHOTORESISTOR_PIN) == LOW)
#endif
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

  // Show the leds (only one of which is set to white, from above)
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
   //task_check_lightlevel();
   //task_sample_mic();
   task_animate_leds();
}
