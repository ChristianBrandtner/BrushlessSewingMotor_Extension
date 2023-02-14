#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2023-01-20 08:47:57

#include "Arduino.h"
#define _SMC_OLED_V14_
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <gfxfont.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "ClickEncoder.h"
#include "MCP4725.h"
#include "TimedBlink.h"
#include "SMC_Oled_V15.h"

void wdt_init(void) ;
void reset_mcusr( void ) ;
ISR(TIMER2_COMPA_vect) ;
void init_Timer2() ;
uint8_t DebouncePin(uint8_t pin)  	 									 ;
void setupTimer1() ;
ISR(TIMER1_OVF_vect,ISR_NOBLOCK)			 ;
ISR(TIMER1_CAPT_vect)						 ;
uint16_t GetCalcedRPM()  ;
void setup() ;
void loop() ;
void stateMachine() ;
void checkTemperatur() ;
void checkForBtnEvent() ;
void checkEncoderEvent() ;
void checkForDP4Event() ;
void displayMenuType0( const __FlashStringHelper *title,                        const __FlashStringHelper *line1,                        const __FlashStringHelper *line2,                        uint8_t SelectedItem ) ;
void displayMenuType1( const __FlashStringHelper *title,                        const __FlashStringHelper *line1,                        const __FlashStringHelper *line2,                        const __FlashStringHelper *line3,                        uint8_t SelectedItem ) ;
void displayMenuType2( const __FlashStringHelper *title,                        const char *value,                        const __FlashStringHelper *units ) ;
void setTextProp( uint8_t size, uint8_t xpos, uint8_t ypos, uint16_t color, boolean invert ) ;
void drawStatusLine() ;
void displayMainScreen(bool signaled) ;
void SetDAC_Volt (uint16_t Volt) ;
int16_t SetDrehzahl(int16_t Drehzahl) ;
void displayHighTemperature() ;
void displaySecurityStop() ;
void displayEEPROM_Reset();
void message( const __FlashStringHelper *line1,               const __FlashStringHelper *line2,               const __FlashStringHelper *line3,               uint8_t displayTime ) ;
void splash() ;
char *valStr( char *str, uint16_t val_u, vf_Type fType ) ;
void resetEeprom( boolean full ) ;
bool loadEeprom() ;
void updateEeprom(bool force) ;
uint8_t get_key_press( uint8_t key_mask ) ;
uint8_t get_key_release(uint8_t key_mask) ;
uint8_t get_key_state(uint8_t key_mask)  ;
int getFreeRAM() ;


#include "SMC_Oled_V15.ino"

#endif
