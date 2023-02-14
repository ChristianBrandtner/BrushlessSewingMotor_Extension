
#define _SMC_OLED_V14_

/*****************************************************************************
 *   Copyright (C) 2023 Christian "Cebra" Brandtner, brandtner@brandtner.net*
 *                                                                          *
 *   This program is free software; you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation; either version 2 of the License.         *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   You should have received a copy of the GNU General Public License      *
 *   along with this program; if not, write to the                          *
 *   Free Software Foundation, Inc.,                                        *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.              *
 *                                                                          *
 *                                                                          *
 *   Credits to:                                                            *
 *		Marc Schönfleisch and his Arduino_Spot_Welder						*
 *		(https://github.com/KaeptnBalu/Arduino_Spot_Welder_V4)				*
 *																			*
 *	Sourcecode location:													*
 *																			*
 *		https://github.com/ChristianBrandtner		 						*
 *																			*
 ****************************************************************************/




/************************************************************************************************
 *
 * my development environment:
 *
 * Eclipse IDE for C/C++ Developers, Version: 2022-12 (4.26.0)
 * Sloeber, the Eclipse Arduino IDE (plugin version) 4.4.1
 *
 *
 *
 * !!!!!! Important   USE Libraries in utils folder!!!!!!!
 *  there are changes compared to the original Arduino Libraries
 *
 *
 *
 ***********************************************************************************************/



/************************************************************************************************
 * Flashing program to Arduino
 *
 * Linux in terminal window:
 *
 * avrdude -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:SMC_Oled_V15_3.hex:i
 *
 * Windows in commandline window (CMD eg. DOS window):
 *
 * avrdude -patmega328p -carduino -PCOM1 -b115200 -D -Uflash:w:SMC_Oled_V15_3.hex:i
 *
 ***********************************************************************************************/



/************************************************************************************************
 * It is only necessary if the bootloader was overwritten
 * Bootloader flash, Important use optiboot bootloader!!, we need all memory until 32256
 * avrdude -pm328P -cstk500v2 -P/dev/ttyUSB0 -b115200 -u
 * -Uflash:w:optiboot_atmega328.hex:a -Ulfuse:w:0xFF:m -Uhfuse:w:0xDE:m -Uefuse:w:0xFD:m -Ulock:w:0x3F:m
 ************************************************************************************************/







//############################################################################
//# HISTORY  SMC_Oled_V15_2.ino
//#
//# 17.01.2023 cebra
//# - add:
//# - chg: MIN_ENCSLOTS = 4	(für den Hallsensoe werden min. 4 Magnete benötigt)
//# - fix: save display orientation in bootmenu
//# - del:
//############################################################################


/***********************************************************************************************//**
 *              Menu and display Screen Descriptions.
 *              The main screen is accessed by a normal device boot.
 *              The system menu is acessed by helding the encoder button;
 *
 +----------------+  MAIN SCREEN
 |   Status Line  |  Temperature and Status information
 |----------------|
 | SPEED          |  Rotation speed
 | rotation       |  and direction of rotation (L or R)
 |   calc. Speed  |  calculated speed high shiftes gear, low shifted gear,  motor
 +----------------+

 +----------------+  SYSTEM MENU
 |   Systemmenu   |  menu label
 |----------------|
 |   gear / ENC   |  raise the gear / encoder setting screen
 |   temperatur   |  raise the temperature setting screen
 |   exit		  |  exit the System menu
 +----------------+

 +----------------+  GEAR/ENCODER SETTINGS
 | gear/encoder se|  Menu label
 |----------------|
 |    gears       |  raise the gears setting screen
 |    # slots     |  raise the number of encoderslots setting screen
 |     Exit       |  Exit to the System Menu screen
 +----------------+


 +----------------+  GEARRATIO SETTINGS
 | gear ratio set |  Menu label
 |----------------|
 |    driveRatio  |  ratio of the motor belt drive
 |    speed  Hi   |  ratio of the high speed gear shifter
 |    speed  Lo   |  ratio of the low speed gear shifter
 +----------------+

 +----------------+  ENCODER SETTINGS
 | encoder slots  |  Menu label
 |----------------|
 |      		  |  Depends of used encoder solution, optical or hallsensor
 | Number of slots|  number of slots or magnets
 |                |
 +----------------+

 +----------------+  TEMPERATURE SETTINGS
 | Temperature Set|  Menu label
 |----------------|
 | Temp.Alarm     |	 temperature threshold	for overheating of the SMC Controller, default 65 centigrade
 | celsius		  |  Unit of measurement, celsius/fahrenheit
 | Exit           |
 +----------------+




    The system screen is accessed by booting the device with the encoder push button pressed.
    The screen allows changing of system settings. It is arranged as follows;

 +----------------+  SYSTEM MENU
 |    Display     |  toggle the Display orientation
 |    EEP Reset   |  !! EEPROM Reset to factory values !!
 |     Exit       |  Reboot
 +----------------+


 *//***********************************************************************************************/


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

//************************************************************************************
// Integrate watchdog and switch off, is required for bootloader
//  !!must stay here on top!!

//--------------------------------------------------------------
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init1")));

//--------------------------------------------------------------
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    return;
}

//**********************************!!do not move this code!!**************************************************

// Machine states
enum states {

	ST_MAIN_SCREEN,    /**< Machine state: display main screen */
	ST_MAIN_SCREEN_CNT, /**< Machine state: display statistics */
	ST_MENU_SCREEN,    /**< Machine state: display menu screen */
	ST_CHANGE_DIR,    /**< Machine state: Motor change direction*/
	ST_WAIT_CHANGE_DIR, /**< Machine state: Wait change LED direction*/
	ST_DP4ON,			/**< Machine state: DP4 LED on*/
	ST_DP4OFF,			/**< Machine state: DP4 LED off*/
    ST_TEMP_HIGH,      /**< Machine state: high temperature */
	ST_SECURITYSTOP,	/**< Machine state: Security Switch Stop */
	ST_RPM_SCREEN,
	ST_GEARENC_MENU,
	ST_ENCDISC,			/**< Machine state: Encoderdisc slots */
	ST_TEMP_MENU,
	ST_GEAR_MENU,
	ST_RED_RATIO,
	ST_SET_C_F,
	ST_SET_HIGHTEMP,
	ST_SYSTEM_SCREEN,  /**< Machine state: display system screen */
	ST_SYSTEM_MENU,    /**< Machine state: display system menu */
	ST_SETTINGS_MENU,  /**< Machine state: display settings menu */
	ST_REBOOT_MENU,    /**< Machine state: display reboot menu */
	ST_INVERT_SCREEN,  /**< Machine state: display invert setting screen */

};

enum event {
	// Private machine events
	EV_NONE,           /**< Machine event: no pending event */
	EV_BTNCL,          /**< Machine event: button down */
	EV_BTNUP,          /**< Machine event: button released */
	EV_BTNDC,		   /**< Machine event: button double clicked */
	EV_BTNHD,		   /**< Machine event: button held*/
	EV_BTNOP,		   /**< Machine event: button open*/
	EV_DP4ON,		   /**< Machine event: DP4 Led an*/
	EV_DP4OFF,		   /**< Machine event: DP4 Led an*/

	EV_ENCUP,          /**< Machine event: encoder rotate right */
	EV_ENCDN,          /**< Machine event: encoder rotate left */



	// Public machine events
	EV_BOOTDN,         /**< Machine event: button pressed on boot */
	EV_TEMP_HIGH,      /**< Machine event: maximum temperature reached */
	EV_BTNSP,		   /**< Machine event: button stop*/
	EV_BTNST,		   /**< Machine event: button start*/
	EV_BTNDI,		   /**< Machine event: button direction*/
	EV_STOP				/**< Security Stop*/
};


enum error{
	ER_DISPLAY,			/**< Error event: Display I2C Error*/
	ER_DAC				/**< Error event: DAC I2C Error*/

};


/***************************************************************************************************
* Global program variables and objects                                                             *
***************************************************************************************************/

// Structures and objects
progData pData;                             /**< Program operating data */
Adafruit_SSD1306 display ( SCREEN_WIDTH , SCREEN_HEIGHT, &Wire, OLED_RESET, 800000L ); /**< OLED display object */

// Static variables
uint8_t mState         = ST_MAIN_SCREEN;    /**< Current machine state */
uint8_t TCelsius;                          /**< System temperature in celsius */
boolean sysMenu = false;             /**< In the system menu structure */
int16_t oldEncPos, encPos;
int16_t encDelta=0;
bool val_edited = false;				//	Flag für veränderte EEPROM Werte, danach save EEPROM
int16_t editdata;
float DAC_VAL;


int16_t MotorRotational_speed=100;    	/**< Motordrehzahl */
int16_t SpindleRotational_speed;   /**< Spindle Rotation Speed */

bool StartButton = false;
bool StopButton = false;
bool DirButton = false;
bool Runing = false;					// Motor läuft
bool Security_Switch_Closed = false;	// Securityswitch geschlossen
bool SMC_Dir_Led = false;				// Direction from SMC Display on/off (DP4)
bool BUT_Dir_Led = false;				// Direction Button LED  on/off
bool SYS_ERROR = false;			   //
bool DisplayError = false;
bool DAC_Error = false;
bool EEPROM_Error = false;
bool SMC_Error = false;					// Keine kommunkation vom SMC (DP4-Signal)
bool TempSensor_Error = false;
bool ChangeDir_Button = false;			// Flag wenn Dir Button in Bearbeitung ist
volatile boolean thisDP4State;


volatile unsigned long lastDP4 = 0;

// Volatile variables - will be changed by the ISR
volatile unsigned long lastActiveTime;
volatile uint8_t mEvent;          /**< Current pending machine event */


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

MCP4725 dac(DAC_ADRESS);

ClickEncoder Encoder = ClickEncoder(ENCODER_PINA,ENCODER_PINB,ENCODER_BTN,ENCODER_STEPS_PER_NOTCH);


TimedBlink DirectionLED(DIR_LED_PIN,true); 		// LED active low
TimedBlink StopLED(STOP_LED_PIN,true);
TimedBlink StartLED(START_LED_PIN,true);



int  resolution = 9;
unsigned long lastTempRequest = 0;
float SensorTemperature = 0.0;

int  idle = 0;


/***************************************************************************************************
* Debouncing in Timer ISR                                                                                    *
***************************************************************************************************/
volatile uint8_t key_state = 0;                      // debounced and inverted key state:
                                            		 // bit = 1: key pressed
volatile uint8_t DP4 = HIGH;						// Zustand der DP4 Dezimalpunktes der SMC Anzeige
volatile uint8_t DP4State = HIGH;					//
volatile uint8_t old_DP4State = HIGH;
volatile uint16_t DIR_Timer =0;						// Timerout für Richtungsumschaltung

volatile bool count330ms = false;					// neue Zählung fertig;
volatile uint16_t CountArray[3];

volatile uint8_t work_key_state = 0;
volatile uint8_t key_press = 0;                      // key press detect
volatile uint8_t key_release =0;						// key release detect
volatile uint8_t key_long  = 0;                      // key long press
volatile uint8_t key_rpt   = 0;                      // key repeat
volatile uint8_t key_lrpt  = 0;                      // key long press and repeat
volatile uint8_t key_rpts  = 0;                      // key long press and speed repeat
volatile uint8_t repeat_speed = 0;

volatile uint16_t debounceinterval = 0;

volatile uint8_t debouncestate = 0;

//**** RPM
#define BUF_SIZE	16
#define F_CPU2		16000000UL

volatile float 	measuredSpeed;				//Gemessene Geschwindigkeit
volatile uint32_t 	cycles = 0;				//Anzahl der vollständigen Zyklen seit der letzten Zählung
volatile uint8_t 	rpm_mux = 60;			//Geschwindigkeitsmultiplikator
volatile float 	measureBuff[BUF_SIZE];		//Puffer mit Messungen.
volatile uint8_t	measuredIndex=0;		//Pufferindex.




/***************************************************************************************************
* Timer ISR   encoder + debouncing keys                                                            *
***************************************************************************************************/

#define reload 		125		// 1ms


ISR(TIMER2_COMPA_vect)
{


	static uint16_t dispTicks = 0;

	static uint8_t ct0 = 0;
	static uint8_t ct1 = 0;
	static uint8_t k_time_l = 0;
	static uint8_t k_time_r = 0;
	uint8_t i;
	static uint8_t DP4_Low_Timer = 0;

	if (dispTicks>1000){
			double calcSpeed = 0;
			int8_t i = 0;
			if (measuredIndex){
				i=measuredIndex-1;
			}else{
				i=BUF_SIZE-1;
			}
			if (measureBuff[i]>2){
				for (uint8_t j = 0; j<BUF_SIZE;j++){
					calcSpeed+=(measureBuff[j]);
				}
				calcSpeed/=BUF_SIZE;
			}else{
				calcSpeed=measureBuff[i];
			}

			measuredSpeed = calcSpeed*rpm_mux;
			dispTicks = 0;
		}else{
			dispTicks++;
		}




	debounceinterval++;

	if (debounceinterval % 10 == 0)                               // alle 10ms debounce

	{	// Debouncing from Peter Dannegger

		i = key_state ^ ~KEY_PIN;                                       // key changed ?
		ct0 = ~(ct0 & i);                                               // reset or count ct0
		ct1 = ct0 ^ (ct1 & i);                                          // reset or count ct1
		i &= (ct0 & ct1);                                               // count until roll over ?
		key_state ^= i;                                                 // then toggle debounced state
		key_press |= (key_state & i);                                   // 0->1: key press detect
		key_release |= ~key_state & i;									// 1->0: key release detect

		//--------------------------------
		// Tasten
		//--------------------------------
		if ((key_state & LONG_MASK) == 0)                               // check long key function
		k_time_l = REPEAT_START;                                    // start delay

		if (--k_time_l == 0)                                            // long countdown
		key_long |= (key_state & LONG_MASK);

		//------
		if ((key_state & REPEAT_MASK) == 0)                             // check repeat function
		k_time_r = 1;                                               // kein delay

		if (--k_time_r == 0)
		{
			k_time_r = REPEAT_NEXT;                                     // repeat delay
			key_rpt |= (key_state & REPEAT_MASK);
		}



	}

	work_key_state = key_state;

	if (!(work_key_state &= 1 << SECURITY_SW_PIN))     // read security Switch
	{
		if (Security_Switch_Closed)
		{
			Security_Switch_Closed = false;
			mEvent = EV_STOP;
		};
	};




//************************** Detect Decimalpoint 4 *************************************************

	if (DebouncePin(SMC_DP4_PIN) == LOW)		// every 24ms there is a low when DP4 is on
	{
		DP4 = LOW;								// DP4 is on
		DP4_Low_Timer = _DP4_Low_Timeout_;
	}
	if (DP4_Low_Timer == 0) DP4 = HIGH;			// if there is no low signal from the SMC after 30ms, the DP4 is off

	if (DP4_Low_Timer > 0) DP4_Low_Timer--;		// Timer count down

	if (DIR_Timer > 0) DIR_Timer--;				// Wait timer for DP4 LED toggling

//	**********************End Detect DP4 *************************************************



Encoder.service();


OCR2A = reload;									// Reload für 1ms Time

}


void init_Timer2()
{
	OCR2A = reload;							// 1ms
	TCCR2A = 1<<WGM21;
	TCCR2B = (1<<CS22) | (1<<CS20);			//Prescaler 128
	TIMSK2 = (1<<OCIE2A);
}




// Idea courtesy Ganssle Group. Called from a 5ms timer,
// the debounced state only ever changes when the pin
// has been stable for 40ms. Initialize debounced_state
// to whatever is  "inactive" for the system (HIGH or LOW)
//
uint8_t DebouncePin(uint8_t pin)  	// used here with a 1ms timer to detect the 6ms decimalpoint 4 signal
									// from the SMC and hide the existing 3pc. 100uS low glitches
{
	static uint8_t debounced_state = HIGH;
	static uint8_t candidate_state = 0;
	candidate_state = candidate_state << 1 | digitalRead(pin);
	if (candidate_state == 0xff) debounced_state = HIGH;
	else

		if ((candidate_state) == 0b00000111) 	// Impuls ist nur 6 ms lang,
		{
		debounced_state = LOW;
		debouncestate = candidate_state;
		}
	return debounced_state;
}


/***************************************************************************************************
* Tacho Implementation                                                                                   *
***************************************************************************************************/

volatile uint16_t revTick;      // Ticks per revolution
volatile uint16_t revCtr;           // Total elapsed revolutions
uint16_t RPM;    // Revolutions per minute
volatile unsigned long RPMCounts=0;


void setupTimer1()
{           // setup timer1
   TCCR1A = 0;      // normal mode
   TCCR1B = _BV(ICNC1)|_BV(ICES1)|_BV(CS10); 		//Noise Canceler,falling edge trigger, Timer = CPU Clock/1 -or- 16 000 000 -or- 16 Mhz
   TCCR1C = 0;      // normal mode
   TIMSK1 = _BV(ICIE1)|_BV(TOIE1);	// (00100001) Input capture and overflow interupts enabled
   TCNT1 = 0;       // start from 0
}

ISR(TIMER1_OVF_vect,ISR_NOBLOCK)			//Timer-Überlauf-Interrupt-Vektor #1
{
	cycles++;
}

ISR(TIMER1_CAPT_vect)						//Timer-Capture-Interrupt-Vektor Nr. 1
{
	volatile uint32_t delta=0;
	TCNT1=0;
	delta = ICR1;
	delta+= ((cycles)*0xffff);
	cycles = 0;
	measureBuff[measuredIndex++] = ((float)F_CPU2/ (float)delta);
	if (measuredIndex==BUF_SIZE)	measuredIndex = 0;
}


uint16_t GetCalcedRPM()

{

	static unsigned long drehzahl;
	noInterrupts();
	drehzahl = measuredSpeed/pData.EncDiscSlots;
	interrupts();
	return (uint16_t) drehzahl;


}


/***************************************************************************************************
* Program setup                                                                                    *
***************************************************************************************************/
/**
 *  \brief    Program setup.
 *  \remarks  Performed once only at powerup.
 */






void setup()
{

	Wire.begin();


	// Setup the pin states and directions.
	pinMode( START_LED_PIN,           	OUTPUT );
	pinMode( STOP_LED_PIN,            	OUTPUT );
    pinMode( DIR_LED_PIN,             	OUTPUT );
    pinMode( BOARD_LED_PIN,             OUTPUT );
    pinMode( SMC_SW_PIN,             	OUTPUT );
	pinMode( START_BUTTON_PIN,     	INPUT_PULLUP );
	pinMode( STOP_BUTTON_PIN,     	INPUT_PULLUP );
	pinMode( DIR_BUTTON_PIN,     	INPUT_PULLUP);
	pinMode( SECURITY_SW_PIN,     	INPUT_PULLUP);
	pinMode( RPM_SENSOR_PIN,     	INPUT_PULLUP);
	pinMode( SMC_DP4_PIN,	    	INPUT_PULLUP);



	digitalWrite( START_LED_PIN, 		LOW );
	digitalWrite( STOP_LED_PIN, 		LOW );
	digitalWrite( DIR_LED_PIN, 			LOW );

	 digitalWrite( SMC_SW_PIN, 	HIGH );

// Setup the OLED display and clear it.
	if (!display.begin( SSD1306_SWITCHCAPVCC, 0x3C)) DisplayError = true;
	else
	{
	display.clearDisplay(); // clear remnants when rebooting
	display.display();
	// Invert the display if required and draw the splash screen. This can only be done after
	// loading the program data (which contains the screen invert switch).

	}
	if (!loadEeprom())
	{
		displayEEPROM_Reset();
		resetEeprom(true);				// Delete Eeprom
    	dac.writeDAC(_DAC_INITVAL, true); // Set DAC EEprom to _DAC_INITVAL e.g. Stopvolt Motor

		while (btnState() != B_DN);
	}
	display.setRotation( pData.oledInvert ? OLED_INVERT : 0 );




	setupTimer1();		//ICP for RPM

	init_Timer2();

	Encoder.setButtonHeldEnabled(true);
	Encoder.setDoubleClickEnabled(true);
	Encoder.setAccelerationEnabled(true);
	oldEncPos = -1;
	// This is used by the standbby timer to identify the period of inactivity.
	lastActiveTime = millis();

	// Test if the pushbutton is pressed at boot time. If so then ensure entry to the system
	// menu by the issue of a boot button down event.
#ifdef _BOOTSYS_
		mEvent = EV_BOOTDN;
#else
		mEvent = btnState() == B_DN ? EV_BOOTDN : EV_NONE;

#endif /* _BOOTSYS_ */




	sensors.begin();		//Tempsensor
	if (sensors.getAddress(insideThermometer, 0))

	{
		sensors.setResolution(insideThermometer, resolution);
		sensors.setWaitForConversion(false);
		sensors.requestTemperatures();
		SensorTemperature= sensors.getTempCByIndex(0);
		TCelsius = uint8_t(SensorTemperature+0.5);
		lastTempRequest = 0;
	}
	else
	{
		TempSensor_Error = true; //Serial.println(F("Unable to find address for Device 0"));
	}


	if (mEvent == EV_NONE)
	{
		if (!DisplayError) 	splash();

		if (!get_key_state(1 << SECURITY_SW_PIN))
		{
			mEvent = EV_STOP;
			Security_Switch_Closed = false;

		}
	}


	if ( dac.begin())
	{
		SetDrehzahl(STOP);
		DAC_Error = false;
	}
	else
	//	Serial.println(F("MCP4725 ERROR"));
		DAC_Error = true;

	digitalWrite( BOARD_LED_PIN, 0);
	digitalWrite( DIR_LED_PIN, 1);
	digitalWrite( START_LED_PIN, 1);
	digitalWrite( STOP_LED_PIN, 1);

//	I2C_Scanner();

	MotorRotational_speed = pData.Last_RPM;

	checkTemperatur();
}

/***************************************************************************************************
* Main program loop                                                                                *
***************************************************************************************************/
/**
 *  \brief    Main program loop.
 *  \remarks  Manages the state machine and eeprom updates.
 */
void loop()
{

	// The state machine is run every loop cycle - as fast as possible.

	stateMachine();

	DirectionLED.blink();
	StopLED.blink();
	StartLED.blink();

	updateEeprom(false);



}




/***************************************************************************************************
* State Machine                                                                                    *
***************************************************************************************************/
/**
 *  \brief  Implementation of state machine.
 */

// This entire state machine is legacy code that has been much modified and added to. It is
// somewhat of a "dog's breakfast" and really is in need of a complete rewrite. Even I have
// difficulty following it and modifications that work are a pleasant surprise - JFF.

void stateMachine() {
	char str[5];
	static uint8_t selectedMenu       = 0;
	static uint8_t selectedMainMenu   = 0;
	static uint8_t selectedSubMenu    = 0;


	// Scan for events - the event queue length is one.

	// Process any public boot events. These events are the highest priority
	// and must be processed before any of the private events.
	if ( mEvent == EV_BOOTDN ) {			// sofort nach dem Booten
		mState = ST_SYSTEM_SCREEN;
		mEvent = EV_NONE;

	}
	else
	{

		// Search for and process any private events. Pulse Activated

		checkEncoderEvent();
		checkForBtnEvent();
		checkForDP4Event();
		checkTemperatur();

		switch (mEvent) {

		case EV_BTNST:					// Start Button gedrückt

			if (!sysMenu)
			{
				if (Security_Switch_Closed & !Runing & !DAC_Error)
				{
					StartLED.Off();
					StopLED.On();

					Runing = true;
					encDelta = 0;
					SetDrehzahl(MotorRotational_speed);

				}
			}
			mEvent = EV_NONE;
			break;

		case EV_BTNSP:					// Stop Button gedrückt
			if (!sysMenu)
			{
				if (Security_Switch_Closed && Runing)
				{
					SetDrehzahl(HALT);
					StartLED.On();
					StopLED.Off();
					Runing = false;
				}
			}

			mEvent = EV_NONE;
			break;

		case EV_BTNDI:					// Direction Button gedrückt

			if (!sysMenu) if (!Runing) mState = ST_CHANGE_DIR;

			mEvent = EV_NONE;
			break;


		case EV_DP4ON:					// DP4 vom SMC auf Low gesetzt, DP4 leuchtet

			if (!sysMenu) mState = ST_DP4ON;
			mEvent = EV_NONE;
			break;

		case EV_DP4OFF:					// DP4 vom SMC auf High gesetzt, DP4 ist aus

			if (!sysMenu) mState = ST_DP4OFF;
			mEvent = EV_NONE;
			break;

		case EV_TEMP_HIGH:

			if (!sysMenu) mState = ST_TEMP_HIGH;

			mEvent = EV_NONE;
			break;

		case EV_STOP:
			if (!sysMenu)
			{
				Security_Switch_Closed = false;
				Runing = false;
				SetDrehzahl(STOP);				// sofort Motor abschalten
				StopLED.blink(200,200);
				StartLED.Off();

				if (!sysMenu) mState = ST_SECURITYSTOP;
			}
			mEvent = EV_NONE;
			break;

		case EV_BTNUP:

			if (mState != ST_REBOOT_MENU) mEvent = EV_NONE;

			break;



		}
	}

	// Machine states
	switch ( mState ) {


  // Over temperature state. Entered by receiving a high temperature event,
  // this state can be exited by clicking the rotary encoder
  case ST_TEMP_HIGH:

    if ( mEvent == EV_BTNCL ) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
    displayHighTemperature();
    mEvent = EV_NONE;
    break;    

    // Over temperature state. Entered by receiving a high temperature event,
    // this state can be exited by clicking the rotary encoder
    case ST_SECURITYSTOP:

      if ( mEvent == EV_BTNCL ) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;
      displaySecurityStop();
      if (Security_Switch_Closed)
    	  {
    	  mState = ST_MAIN_SCREEN_CNT;
    	  displayMainScreen( false );
    	  StopLED.blinkOff();


    	  }

      mEvent = EV_NONE;
      break;

	case ST_DP4ON:

		DirectionLED.On();

		mEvent = EV_NONE;
		if (ChangeDir_Button) mState = ST_WAIT_CHANGE_DIR;
		else
		{
			mState = ST_MAIN_SCREEN_CNT;
			displayMainScreen(false);
		}

		break;

	case ST_DP4OFF:

		DirectionLED.Off();

		mEvent = EV_NONE;
		if (ChangeDir_Button) mState = ST_WAIT_CHANGE_DIR;
		else
		{
			mState = ST_MAIN_SCREEN_CNT;
			displayMainScreen(false);
		}

		break;

	case ST_WAIT_CHANGE_DIR:

		if (old_DP4State != DP4)
		{
			digitalWrite( SMC_SW_PIN, HIGH);  		//"Taste loslassen"
			mState = ST_MAIN_SCREEN_CNT;
			ChangeDir_Button = false;
			displayMainScreen(false);
		}
		else
			if (DIR_Timer == 0)						//Timeout als Backup, damit SMC nicht blockiert falls die Umschaltung nicht klappt
			{
				digitalWrite( SMC_SW_PIN, HIGH); 	//"Taste loslassen"
				ChangeDir_Button = false;

				mState = ST_MAIN_SCREEN_CNT;
				displayMainScreen(false);
			}
		break;

    case ST_CHANGE_DIR:
    	ChangeDir_Button = true;
    	DirectionLED.blink(200,200);
		old_DP4State = DP4;
		digitalWrite( SMC_SW_PIN, LOW);		// "Taste drücken"
		DIR_Timer = 1200;					// Wartezeit setzen
		mState = ST_WAIT_CHANGE_DIR;

 	break;

	// Main screen state displays the main screen without responding to events. This is
	// effectively the home location for an idle loop that cycles through this and the
	// next state where it looks for events. Any state can return to this state to
	// enable an idle loop.
	case ST_MAIN_SCREEN:


		mState = ST_MAIN_SCREEN_CNT;
		sysMenu = false;
		selectedMenu = 0;
		displayMainScreen(false);
		break;

	// Main screen state that responds to events. This state is an entry point for
	// the menu structure and allows setting and display of operational variables.


	case ST_MAIN_SCREEN_CNT:
		// A button held event will enter the System Menu
		if (mEvent == EV_BTNHD)
		{
			mState = ST_MENU_SCREEN;
			selectedMenu = 0;
			mEvent = EV_NONE;
			displayMenuType1( FPSTR(LS_SYSMENU), FPSTR( LS_GEARENC), FPSTR( LS_TEMPERATUR ), FPSTR( LS_EXIT ), 0 );
		}
		else

			{
			if (Security_Switch_Closed)
			{
			SpindleRotational_speed = (uint16_t) GetCalcedRPM();

				// Rotary encoder events will change the Spindel RPM.
				if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
				{
					if (mEvent == EV_ENCUP) MotorRotational_speed =
							MotorRotational_speed < MAX_ROTATIONALSPEED ? MotorRotational_speed + encDelta : MAX_ROTATIONALSPEED;
					else
						MotorRotational_speed = MotorRotational_speed > MIN_ROTATIONALSPEED	 ? MotorRotational_speed - encDelta : MIN_ROTATIONALSPEED	;


					MotorRotational_speed = MotorRotational_speed < MIN_ROTATIONALSPEED	 ? MIN_ROTATIONALSPEED	 :
										MotorRotational_speed > MAX_ROTATIONALSPEED ? MAX_ROTATIONALSPEED : MotorRotational_speed;


					SpindleRotational_speed = (uint16_t) GetCalcedRPM();

					mEvent = EV_NONE;

					if (Runing)
						{
						 SetDrehzahl(MotorRotational_speed);
						 pData.Last_RPM = MotorRotational_speed;
						}

				}
			}

				displayMainScreen(false);
			}

		break;

	// The first menu in the menu structure. This is accessed from the main screen
	// by a button press event.
	case ST_MENU_SCREEN:
		sysMenu = true;
		StartLED.Off();
		// A button press event will enter the next level of the menu structure. There are
		// three items in the menu to decide which new menu is entered.
		if ( mEvent == EV_BTNCL ) {
			mEvent = EV_NONE;
			selectedMainMenu = selectedMenu;

			// Enter a sub-menu to select more options.
			if ( selectedMainMenu == 0 )
			{

				mState = ST_GEARENC_MENU;
				selectedMenu = 0;

				displayMenuType1(FPSTR(LS_DACMENU), FPSTR(LS_GEAR), FPSTR(LS_ENCSLOTS), FPSTR(LS_EXIT), 0);

			}
			// Allow settingge.
			else if (  selectedMainMenu == 1 )
			{
				mState = ST_TEMP_MENU;
				selectedMenu = 0;
				displayMenuType1(FPSTR(LS_TEMPALM), FPSTR(LS_HIGHT), pData.TCelsius ? FPSTR(  LS_CEL ) : FPSTR( LS_FAH ), FPSTR(LS_EXIT),0);
			}
			// Exit
			else if ( selectedMainMenu == 2 )
				{
				mState = ST_MAIN_SCREEN;
				StartLED.On();
				selectedMenu = 0;
				sysMenu = false;
				}

			// Rotary encoder events will allow scrolling through the menu options.
		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( mEvent == EV_ENCDN ) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
			else selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;
			mEvent = EV_NONE;
			displayMenuType1( FPSTR(LS_SYSMENU), FPSTR( LS_GEARENC), FPSTR( LS_TEMPERATUR ), FPSTR( LS_EXIT ), selectedMenu );
		}


		break;




	case ST_GEARENC_MENU:

			// A button press event will enter the next level of the menu structure. There are
			// three items in the menu to decide which new menu is entered.
			if (mEvent == EV_BTNCL)
			{
				mEvent = EV_NONE;

				selectedSubMenu = selectedMenu;
				// DAC Settings base Volt value
				if (selectedSubMenu == 0)
				{
					displayMenuType1(FPSTR(LS_GEARMENU), FPSTR(LS_DRIVERATIO), FPSTR(LS_HIRATIO), FPSTR(LS_LORATIO), 0);
					selectedMenu = 0;
					mState = ST_GEAR_MENU;

				}
				else
					if (selectedSubMenu == 1)
					{
						displayMenuType2(FPSTR(LS_ENCSLOTS_S), valStr(str, pData.EncDiscSlots, VF_RPM), FPSTR(LS_ENCSLOTS));
						selectedMenu = 0;
						mState = ST_ENCDISC;

					}

					// Exit
					else
						if (selectedSubMenu == 2)
						{
							displayMenuType1( FPSTR(LS_SYSMENU), FPSTR( LS_GEARENC), FPSTR( LS_TEMPERATUR ), FPSTR( LS_EXIT ), 0 );
							selectedMenu = 0;
							mEvent = EV_NONE;
							mState = ST_MENU_SCREEN;

						}

				// Rotary encoder events will allow scrolling through the menu options.
			}
			else
				if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
				{

					if (mEvent == EV_ENCDN) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
					else
						selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;
					mEvent = EV_NONE;
					displayMenuType1(FPSTR(LS_DACMENU), FPSTR(LS_GEAR), FPSTR(LS_ENCSLOTS), FPSTR(LS_EXIT),selectedMenu);
				}

			break;




	case ST_ENCDISC:

			if (mEvent == EV_BTNCL)
					{
						mEvent = EV_NONE;

						if (val_edited )
						{
							updateEeprom(false);
							val_edited = false;
							message(FPSTR(LS_EEPROMDASI), FPSTR(LS_EEPROMUPD), FPSTR(LS_BLANK), 2);

						}
						displayMenuType1(FPSTR(LS_DACMENU), FPSTR(LS_GEAR), FPSTR(LS_ENCSLOTS), FPSTR(LS_EXIT), 0);
						mState = ST_GEARENC_MENU;
						selectedMenu = 0;

					}
					else
						if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
						{
							if (mEvent == EV_ENCDN)
								pData.EncDiscSlots = pData.EncDiscSlots > MIN_ENCSLOTS ? pData.EncDiscSlots - 1 : MIN_ENCSLOTS;
							else
								pData.EncDiscSlots = pData.EncDiscSlots < MAX_ENCSLOTS ? pData.EncDiscSlots + 1 : MAX_ENCSLOTS;

							pData.EncDiscSlots = pData.EncDiscSlots < MIN_ENCSLOTS ? MIN_ENCSLOTS :
												pData.EncDiscSlots > MAX_ENCSLOTS ? MAX_ENCSLOTS : pData.EncDiscSlots;
							displayMenuType2(FPSTR(LS_ENCSLOTS_S), valStr(str, pData.EncDiscSlots, VF_RPM), FPSTR(LS_ENCSLOTS));
							val_edited = true;
							mEvent = EV_NONE;
						}



			break;


	case ST_GEAR_MENU:

		displayMenuType1(FPSTR(LS_GEARMENU), FPSTR(LS_DRIVERATIO), FPSTR(LS_HIRATIO), FPSTR(LS_LORATIO), selectedMenu);

		// A button press event will enter the next level of the menu structure. There are
		// three items in the menu to decide which new menu is entered.
		if (mEvent == EV_BTNCL)
		{
			mEvent = EV_NONE;
			selectedSubMenu = selectedMenu;

			//
			if (selectedSubMenu == 0)
			{

				editdata = pData.REDRATIO;
				displayMenuType2(FPSTR(LS_DRIVERATIO_S), valStr(str, pData.REDRATIO, VF_RATIO), FPSTR(LS_REDRATIO));
				selectedMenu = 0;
				mState = ST_RED_RATIO;
				mEvent = EV_NONE;

			}
			// Allow setting of reduction ration
			else
				if (selectedSubMenu == 1)
				{
					editdata = pData.HIGHRATIO;
					displayMenuType2(FPSTR(LS_HIRATIO_S), valStr(str, pData.HIGHRATIO, VF_RATIO), FPSTR(LS_REDRATIO));
					selectedMenu = 0;
					mState = ST_RED_RATIO;
					mEvent = EV_NONE;


				}
				else
				{
					if (selectedSubMenu == 2)
					{

						editdata = pData.LOWRATIO;
						displayMenuType2(FPSTR(LS_LORATIO_S), valStr(str, pData.LOWRATIO, VF_RATIO), FPSTR(LS_REDRATIO));
						selectedMenu = 0;
						mState = ST_RED_RATIO;
						mEvent = EV_NONE;


					}
				}

			selectedMenu = 0;
			// Rotary encoder events will allow scrolling through the menu options.
		}
		else
			if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
			{

				if (mEvent == EV_ENCDN) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
				else
					selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;
				mEvent = EV_NONE;

				displayMenuType1(FPSTR(LS_GEARMENU), FPSTR(LS_DRIVERATIO), FPSTR(LS_HIRATIO), FPSTR(LS_LORATIO), selectedMenu);
			}

		break;




	case ST_RED_RATIO:

		if (mEvent == EV_BTNCL)					// Fertig editiert
		{
			mEvent = EV_NONE;

			switch (selectedSubMenu) {

			case 0:
				pData.REDRATIO = editdata;
				break;

			case 1:
				pData.HIGHRATIO = editdata;
				break;

			case 2:
				pData.LOWRATIO = editdata;
				break;

			}

			if (val_edited)
			{
				updateEeprom(false);
				val_edited = false;
				message(FPSTR(LS_EEPROMDASI), FPSTR(LS_EEPROMUPD), FPSTR(LS_BLANK), 2);

			}

			mState = ST_GEARENC_MENU;
			displayMenuType1(FPSTR(LS_DACMENU), FPSTR(LS_GEAR), FPSTR(LS_ENCSLOTS), FPSTR(LS_EXIT), selectedMenu);
			selectedMenu = 0;

		}
		else
		{
			switch (selectedSubMenu) {

			case 0:

				displayMenuType2(FPSTR(LS_DRIVERATIO_S), valStr(str, editdata, VF_RATIO), FPSTR(LS_DRIVERATIO));

				break;

			case 1:
				displayMenuType2(FPSTR(LS_HIRATIO_S), valStr(str, editdata, VF_RATIO), FPSTR(LS_HIRATIO));
				break;

			case 2:
				displayMenuType2(FPSTR(LS_LORATIO_S), valStr(str, editdata, VF_RATIO), FPSTR(LS_LORATIO));

				break;
			}

			if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
			{

				if (mEvent == EV_ENCDN) editdata = editdata > MIN_GEARRATIO ? editdata - 1 : MIN_GEARRATIO;
				else
				{
					editdata = editdata < MAX_GEARRATIO ? editdata + 1 : MAX_GEARRATIO;

					editdata = editdata < MIN_GEARRATIO ? MIN_GEARRATIO : editdata > MAX_GEARRATIO ? MAX_GEARRATIO : editdata;

				}

				if (selectedSubMenu == 0) displayMenuType2(FPSTR(LS_DRIVERATIO_S), valStr(str, editdata, VF_RATIO),	FPSTR(LS_DRIVERATIO));
				 else
				if (selectedSubMenu == 1) displayMenuType2(FPSTR(LS_HIRATIO_S), valStr(str, editdata, VF_RATIO),FPSTR(LS_HIRATIO));
				 else
				if (selectedSubMenu == 2) displayMenuType2(FPSTR(LS_LORATIO_S), valStr(str, editdata, VF_RATIO),FPSTR(LS_LORATIO));

				val_edited = true;
			}

		}

		mEvent = EV_NONE;

		break;

//#endif

	case ST_TEMP_MENU:

		// A button press event will enter the next level of the menu structure. There are
		// three items in the menu to decide which new menu is entered.
		if (mEvent == EV_BTNCL)
		{
			mEvent = EV_NONE;
			selectedMainMenu = selectedMenu;

			//
			if (selectedMainMenu == 0)

			{
				displayMenuType2(FPSTR(LS_TEMPALM),
						valStr(str,
								pData.TCelsius ?
										pData.maxTemperatur : DallasTemperature::toFahrenheit(pData.maxTemperatur),
										pData.TCelsius ? VF_TEMPC : VF_TEMPF), FPSTR(LS_HIGHT));
				mState = ST_SET_HIGHTEMP;
				mEvent = EV_NONE;

			}

			else
				if (selectedMainMenu == 1)
				{
					displayMenuType2(FPSTR(LS_TEMPALM), NULL, pData.TCelsius ? FPSTR(LS_CEL) : FPSTR(LS_FAH));

					mState = ST_SET_C_F;
					mEvent = EV_NONE;

				}

				// Exit
				else
					if (selectedMainMenu == 2)
					{
						displayMenuType1( FPSTR(LS_SYSMENU), FPSTR( LS_GEARENC), FPSTR( LS_TEMPERATUR ), FPSTR( LS_EXIT ), 0 );
						mState = ST_MENU_SCREEN;
						mEvent = EV_NONE;

					}

			selectedMenu = 0;

			// Rotary encoder events will allow scrolling through the menu options.
		}
		else
			if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
			{

				if (mEvent == EV_ENCDN) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
				else
					selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;
				mEvent = EV_NONE;
				displayMenuType1(FPSTR(LS_TEMPALM), FPSTR(LS_HIGHT), pData.TCelsius ? FPSTR(LS_CEL) : FPSTR(LS_FAH),
						FPSTR(LS_EXIT), selectedMenu);

			}
		mEvent = EV_NONE;
		break;



	case ST_SET_C_F:

					if (mEvent == EV_BTNCL)
					{

						if (val_edited )
						{
							updateEeprom(false);
							val_edited = false;
							message(FPSTR(LS_EEPROMDASI), FPSTR(LS_EEPROMUPD), FPSTR(LS_BLANK), 2);

						}
						displayMenuType1(FPSTR(LS_TEMPALM), FPSTR(LS_HIGHT), pData.TCelsius ? FPSTR(  LS_CEL ) : FPSTR( LS_FAH ), FPSTR(LS_EXIT), selectedMenu);
						mState = ST_TEMP_MENU;
						selectedMenu = 0;

					}
					else
						if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
						{
							pData.TCelsius = !pData.TCelsius;
							displayMenuType2( FPSTR( LS_TEMPALM), NULL,pData.TCelsius ? FPSTR(  LS_CEL ) : FPSTR( LS_FAH ) );
							val_edited = true;

						}

					mEvent = EV_NONE;

					break;

		case ST_SET_HIGHTEMP:

		if (mEvent == EV_BTNCL)
		{

			if (val_edited )
			{
				updateEeprom(false);
				val_edited = false;
				message(FPSTR(LS_EEPROMDASI), FPSTR(LS_EEPROMUPD), FPSTR(LS_BLANK), 2);

			}
			displayMenuType1(FPSTR(LS_TEMPALM), FPSTR(LS_HIGHT), pData.TCelsius ? FPSTR(LS_CEL) : FPSTR(LS_FAH),
					FPSTR(LS_EXIT), selectedMenu);

			mState = ST_TEMP_MENU;
			selectedMenu = 0;

		}
		else
			if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
			{
				if (mEvent == EV_ENCDN) pData.maxTemperatur =
						pData.maxTemperatur > MIN_TEMP ? pData.maxTemperatur - 1 : MIN_TEMP;
				else
					pData.maxTemperatur = pData.maxTemperatur < MAX_TEMP ? pData.maxTemperatur + 1 : MAX_TEMP;

				pData.maxTemperatur = pData.maxTemperatur < MIN_TEMP ? MIN_TEMP :
										pData.maxTemperatur > MAX_TEMP ? MAX_TEMP : pData.maxTemperatur;

				displayMenuType2(FPSTR(LS_TEMPALM),
						valStr(str,
								pData.TCelsius ?
										pData.maxTemperatur : DallasTemperature::toFahrenheit(pData.maxTemperatur),
										pData.TCelsius ? VF_TEMPC : VF_TEMPF), FPSTR(LS_HIGHT));

				val_edited = true;

			}

		mEvent = EV_NONE;

		break;





	// System screen state displays the system screen without responding to events.
	// This is the home state of another idle loop, this one for the system menu.
	case ST_SYSTEM_SCREEN:

		mState = ST_SYSTEM_MENU;
		val_edited = false;
		sysMenu = true;
		selectedMenu = 0;


		displayMenuType1( FPSTR( LS_SYSMENU ), FPSTR( LS_DISPLAY ), FPSTR(LS_EEPROM),FPSTR( LS_EXIT), 0 );
		display.drawRect( 0, 0, SSD1306_LCDWIDTH-1,SSD1306_LCDHEIGHT-1, WHITE );
		display.display();
		break;

	// The first menu in the system menu structure. This is accessed by rebooting
	//  while holding the encoder button down.
	case ST_SYSTEM_MENU:



		if (mEvent == EV_BTNCL)
		{


			mEvent = EV_NONE;

			if (selectedMenu == 0)
			{

				selectedSubMenu = 0;
				mState = ST_INVERT_SCREEN;

				displayMenuType2(FPSTR(LS_INVERTMENU), NULL, pData.oledInvert ? FPSTR(LS_SCRINV) : FPSTR(LS_SCRNORM));


			}
			else
				if (selectedMenu == 1)
				{
					resetEeprom(true);
					wdt_enable( WDTO_250MS);   // start bootloader with Reset

				}
				else
					if (selectedMenu == 2)
					{

						wdt_enable( WDTO_250MS);   // start bootloader with Reset

					}

			selectedMenu = 0;

		}
		else
			if (mEvent == EV_ENCUP || mEvent == EV_ENCDN)
			{
				if (mEvent == EV_ENCDN) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
				else
					selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;

				mEvent = EV_NONE;

				displayMenuType1(FPSTR(LS_SYSMENU), FPSTR(LS_DISPLAY), FPSTR(LS_EEPROM), FPSTR(LS_EXIT), selectedMenu);
				display.drawRect(0, 0, SSD1306_LCDWIDTH - 1, SSD1306_LCDHEIGHT - 1, WHITE);
				display.display();

			}

		break;


	case ST_INVERT_SCREEN:

		if ( mEvent == EV_BTNCL ) {
			mState = ST_SYSTEM_SCREEN;
			mEvent = EV_NONE;
			selectedMenu = 0;


			if (val_edited)
			{
				updateEeprom(true);
				val_edited = false;
				message(FPSTR(LS_EEPROMDASI), FPSTR(LS_EEPROMUPD), FPSTR(LS_BLANK), 2);
			}

		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			pData.oledInvert = !pData.oledInvert;
			display.setRotation( pData.oledInvert ? 2 : 0 );
			displayMenuType2( FPSTR( LS_INVERTMENU ), NULL, pData.oledInvert ?
			                  FPSTR( LS_SCRINV ) : FPSTR( LS_SCRNORM ) );
			val_edited = true;
		}

		mEvent = EV_NONE;
		break;

	default: break;
	}
}

/***************************************************************************************************
* Hardware management routines                                                                     *
***************************************************************************************************/


void checkTemperatur()
{
		if (millis() - lastTempRequest >= TEMPCHECKTIME) // waited long enough??
	{
			SensorTemperature = sensors.getTempCByIndex(0);
			if (SensorTemperature==DEVICE_DISCONNECTED_C) TempSensor_Error = true;
			else sensors.setResolution(insideThermometer, resolution);

			if (!TempSensor_Error)
			{
				sensors.requestTemperatures();
				lastTempRequest = millis();
				TCelsius = uint8_t(SensorTemperature + 0.5);

				if (TCelsius > pData.maxTemperatur) mEvent = EV_TEMP_HIGH;
			}
		else
		{
			SensorTemperature = sensors.getTempCByIndex(0);
			if (SensorTemperature!=DEVICE_DISCONNECTED_C) TempSensor_Error = false;
			lastTempRequest = millis();
		}

	  }
}






void checkForBtnEvent()
{
	static uint8_t oldbuttonState=0;
	uint8_t buttonState = Encoder.getButton();

	if (oldbuttonState != buttonState)
	{

		oldbuttonState = buttonState;
		lastActiveTime = millis();

		switch (buttonState) {

		case ClickEncoder::Held:
			mEvent = EV_BTNHD;
			break;
		case ClickEncoder::Released:
			mEvent = EV_BTNUP;
			break;
		case ClickEncoder::Clicked:
			mEvent = EV_BTNCL;
			break;
		case ClickEncoder::DoubleClicked:
			mEvent = EV_BTNDC;
			break;
		case ClickEncoder::Open:
			mEvent = EV_NONE;
			break;
		}
	}

	if (get_key_state(1 << SECURITY_SW_PIN))
	{

		if (!Security_Switch_Closed)

		{
			StartLED.On();
			StopLED.Off();
			Security_Switch_Closed = true;
		}

	}
	else
		Security_Switch_Closed = false;


	if (get_key_press (1 << START_BUTTON_PIN)) mEvent = EV_BTNST;

	if (get_key_press (1 << STOP_BUTTON_PIN))  mEvent = EV_BTNSP;

	if (get_key_press(1 << DIR_BUTTON_PIN)) mEvent = EV_BTNDI;


}







void checkEncoderEvent()
{
	encPos += Encoder.getValue();

	if (encPos != oldEncPos)
	{
		lastActiveTime = millis();

		if (encPos > oldEncPos)
		{
			encDelta = (encPos - oldEncPos) * 2;
			mEvent = EV_ENCUP;
			oldEncPos = encPos;
		}

		else

		{
			encDelta = (oldEncPos - encPos) * 2;
			mEvent = EV_ENCDN;
			oldEncPos = encPos;
		}
	}

}


void checkForDP4Event()
{

	static unsigned long oldDP4 = 0xff;
	lastDP4 = DP4;

	if (oldDP4 != lastDP4)
	{
		if (DP4 == LOW) mEvent = EV_DP4ON;
		else
			mEvent = EV_DP4OFF;
		oldDP4 = lastDP4;
	}
}



/***************************************************************************************************
* Menu management routines                                                                         *
***************************************************************************************************/
/**
 *  \brief      Displays a menu type 0 on the LCD.
 *  \remarks    This is a three line selection menu. If the title pointer is NULL then a standard
 *              status line is drawn otherwise the title is drawn on the screen.
 *  \param [in] const __FlashStringHelper *title       Menu title.
 *  \param [in] const __FlashStringHelper *line1       Line 1 text.
 *  \param [in] const __FlashStringHelper *line2       Line 2 text.
 *  \param [in] uint8_t                   SelectedItem Selected menu item.
 */
void displayMenuType0( const __FlashStringHelper *title,
                       const __FlashStringHelper *line1,
                       const __FlashStringHelper *line2,
                       uint8_t SelectedItem ) {

	display.clearDisplay();

	if ( title == NULL )
		drawStatusLine();
	else {
		setTextProp( 1, 1, 1, WHITE );
		display.print( title );
		display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );
	}

	setTextProp( 2, 2, 16, WHITE, SelectedItem == 0 );
	display.print( line1 );
	setTextProp( 2, 2, 16 + 2 * CHR_H + 1, WHITE, SelectedItem == 1 );
	display.print( line2 );
	setTextProp( 2, 2, 16 + 4 * CHR_H + 2, WHITE, SelectedItem == 2 );
	display.drawRect( 0, SelectedItem == 0 ? 16 :
	                  SelectedItem == 1 ? 16 + 2 * CHR_H + 1 :
	                  16 + 4 * CHR_H + 2, 2, 2 * CHR_H, WHITE );
	display.display();
}

/**
 *  \brief      Displays a menu type 1 on the LCD.
 *  \remarks    This is a three line selection menu. If the title pointer is NULL then a standard
 *              status line is drawn otherwise the title is drawn on the screen.
 *  \param [in] const __FlashStringHelper *title       Menu title.
 *  \param [in] const __FlashStringHelper *line1       Line 1 text.
 *  \param [in] const __FlashStringHelper *line2       Line 2 text.
 *  \param [in] const __FlashStringHelper *line3       Line 3 text.
 *  \param [in] uint8_t                   SelectedItem Selected menu item.
 */
void displayMenuType1( const __FlashStringHelper *title,
                       const __FlashStringHelper *line1,
                       const __FlashStringHelper *line2,
                       const __FlashStringHelper *line3,
                       uint8_t SelectedItem ) {

	display.clearDisplay();

	if ( title == NULL )
		drawStatusLine();
	else {
		setTextProp( 1, 1, 1, WHITE );
		display.print( title );
		display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );
	}

	setTextProp( 2, 2, 16, WHITE, SelectedItem == 0 );
	display.print( line1 );
	setTextProp( 2, 2, 16 + 2 * CHR_H + 1, WHITE, SelectedItem == 1 );
	display.print( line2 );
	setTextProp( 2, 2, 16 + 4 * CHR_H + 2, WHITE, SelectedItem == 2 );
	display.print( line3 );
	display.drawRect( 0, SelectedItem == 0 ? 16 :
	                  SelectedItem == 1 ? 16 + 2 * CHR_H + 1 :
	                  16 + 4 * CHR_H + 2, 2, 2 * CHR_H, WHITE );
	display.display();
}
/**
 *  \brief      Displays a menu type 2 on the LCD.
 *  \remarks    This is a variable display and adjustment dialog. Normally the second parameter
 *              is a pointer to a string value in ram and the third parameter is a pointer to the
 *              units string in flash. If the second parameter is NULL then the third parameter
 *              is treated as the second parameter. This enables the display of two adjacent
 *              flash based strings.
 *  \param [in] const __FlashStringHelper *title  Menu title.
 *  \param [in] const char                *value  Item value.
 *  \param [in] const __FlashStringHelper *units  Item units.
 */
void displayMenuType2( const __FlashStringHelper *title,
                       const char *value,
                       const __FlashStringHelper *units ) {

	display.clearDisplay();

	setTextProp( 1, 1, 1, WHITE );
	display.print( title );
	display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );
	setTextProp( 2, 2, 16 + LINE_H );

	if ( value == NULL ) display.print( units );
	else {
		display.print( value );
		setTextProp( 1, 2, 16 + 3 * LINE_H );
		display.print( units );
	}

	display.display();
}

/***************************************************************************************************
* Display management routines                                                                      *
***************************************************************************************************/
/**
 *  \brief                      Sets the properties of a text string to be drawn to the display.
 *  \param [in] uint8_t  size   Text size.
 *  \param [in] uint8_t  xpos   Text x-coordinate in pixels from left.
 *  \param [in] uint8_t  ypos   Text x-coordinate in pixels from top.
 *  \param [in] uint16_t color  Text foreground  color (default WHITE).
 *  \param [in] boolean  invert True if colors are to be inverted (bg=color,fg=BLACK) default false.
 */
void setTextProp( uint8_t size, uint8_t xpos, uint8_t ypos, uint16_t color, boolean invert ) {

	display.setTextSize( size );
	display.setCursor( xpos, ypos );

	if ( invert )
		display.setTextColor( BLACK, color );
	else
		display.setTextColor( color );
}

/**
 *  \brief    Draws a status line to the LCD.
 *  \remarks  The status line shows the pulse trigger mode and the total number of welds performed.
 *            On dual color displays the status line is within the top colored display region. An
 *            underline is drawn under the status line.
 */
void drawStatusLine() {
	char str[8];

	if (Security_Switch_Closed)
	{

		if (DAC_Error)
		{
			display.print(FPSTR(LS_DACERROR));

		}
		else
			if (TempSensor_Error)
			{
				display.print(FPSTR(LS_TEMPERROR));

			}

	}
	else
	{
		setTextProp(1, 1, 1, WHITE, true);
		display.print(FPSTR(LS_NOSECURITY));
	}

	setTextProp( 1, 14 * CHR_W, 1 );

	  display.print(pData.TCelsius ? valStr( str, TCelsius,VF_TEMPC ) : valStr( str, DallasTemperature::toFahrenheit(TCelsius), VF_TEMPF ));

	// Put an underline under the status line.
	display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );


}

/**
 *  \brief  Draw the main screen to the LCD.
 */
void displayMainScreen(bool signaled)
{
	char str[5];
	uint16_t Drehzahl;

	display.clearDisplay();
	drawStatusLine();

	// Write the current value of the weld pulse duration and its units.
	setTextProp(4, 1, 16 + CHR_H / 2);

	if (Runing) display.print(valStr(str, (uint16_t) SpindleRotational_speed, VF_RPM));
	else
		display.print(valStr(str, (uint16_t) 0, VF_RPM));

	setTextProp(1, 4 * CHR_W * 4, 16 + CHR_H / 2 + 3 * CHR_H - 2 * CHR_H - 2, WHITE, signaled);
	display.print(FPSTR(LS_RPM));

	setTextProp(1, 4 * CHR_W * 4 + 6, 16 + CHR_H / 2 + 5 * CHR_H - 2 * CHR_H - 2, WHITE, signaled);

	display.print(DP4 ? FPSTR(LS_LEFT) : FPSTR(LS_RIGHT));

	// Write a message under the value.

#define DownStatus_Line 55

	setTextProp(0, 0, DownStatus_Line);
	display.print(FPSTR(LS_SPINDEL_HIGH));

	setTextProp(0, 17, DownStatus_Line);
	Drehzahl = ((pData.REDRATIO * pData.HIGHRATIO) / 100) * (MotorRotational_speed / 100);

	display.print(valStr(str, (uint16_t) Drehzahl, VF_RPM));

	setTextProp(0, 48, DownStatus_Line);
	display.print(FPSTR(LS_SPINDEL_LOW));

	setTextProp(0, 60, DownStatus_Line);
	display.print(
			valStr(str, (uint16_t) ((pData.REDRATIO * pData.LOWRATIO) / 100) * (MotorRotational_speed / 100), VF_RPM));

	setTextProp(0, 90, DownStatus_Line);
	display.print(FPSTR(LS_MOTOR_DREHZAHL));

	display.print(valStr(str, (uint16_t) MotorRotational_speed, VF_RPM));

	display.display();
}


/**
 *  \brief    	Berechnet den DAC Wert für die eingestellte Drehzahl.
 *  \remarks  	in Abhängigkeit von Getriebeuntersetzung und Antriebintersetzung
 *  			Rückgabewert ist die Drehzahl für die Anzeige im Display.
 *  			Der Min Max Rohwert ergibt sich aus #define MIN_ROTATIONALSPEED	 und MAX_DREHZAHL
 *
 *
 *
 *  \param [in] uint16_t Drehzahl
 */


void SetDAC_Volt (uint16_t Volt)
{

	DAC_VAL = ((float) Volt / 1000)* MCP4725_BITS_PER_VOLT;
	dac.setValue((uint16_t)DAC_VAL);

}




int16_t SetDrehzahl(int16_t Drehzahl)
{
	float Volt, RAMP_VAL = 0;

	if (Drehzahl == HALT)
	{
		RAMP_VAL = DAC_VAL;

		if (RAMP_VAL > 2484)
		{

			do
			{
				if (DIR_Timer == 0)
				{
					RAMP_VAL = RAMP_VAL - (float) (_DAC_VAL100_ * 2);
					dac.setValue(RAMP_VAL);
					DIR_Timer = _RAMPDOWNTIME_;

				}
			}
			while (RAMP_VAL >= 1000);					// langsam absenken

			dac.setValue(STOP_VOLT);
		}
		else

			dac.setValue(STOP_VOLT);
	}
	else
		if (Drehzahl == STOP)
		{

		// stop immediately
			dac.setValue(STOP_VOLT);

		}

	else
		// Berechnung: (Drehzahl /100) * VOLTDIFF_100 + VOLT_BASIS100


	{

		Volt = ((float)Drehzahl *  (float)VOLTDIFF_100/100) + (float)pData.RPMBASISVOLT/100;
		DAC_VAL = Volt * MCP4725_BITS_PER_VOLT;

		dac.setValue((uint16_t)DAC_VAL);
	};

	return Drehzahl;

}



void displayHighTemperature() {
  char str[5];

  display.clearDisplay();
  drawStatusLine();

  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_HIGHT ) - 1 ) * CHR_W * 2 ) / 2, 16 );
  display.print(pData.TCelsius ? valStr( str, TCelsius, VF_TEMPC ) : valStr( str, DallasTemperature::toFahrenheit(TCelsius), VF_TEMPF ));
  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_HIGHT ) - 1 ) * CHR_W * 2 ) / 2, 16 + 2 * LINE_H );
  display.print( FPSTR( LS_HIGHT ) );
  setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_COOL )- 1 )* CHR_W ) / 2, 16 + 4 * LINE_H );
  display.print( FPSTR( LS_COOL ) );

  display.display();
}

void displaySecurityStop() {

  display.clearDisplay();
  drawStatusLine();

  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_STOP ) - 1 ) * CHR_W * 2 ) / 2, 16,WHITE,true );
  display.print( FPSTR( LS_STOP ) );
  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_STOP1 ) - 1 ) * CHR_W * 2 ) / 2, 16 + 2 * LINE_H ,WHITE,true );
  display.print( FPSTR( LS_STOP1) );
  setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_STOPTEXT )- 1 )* CHR_W ) / 2, 16 + 4 * LINE_H );
  display.print( FPSTR( LS_STOPTEXT ) );

  display.display();
}

void displayEEPROM_Reset(){


  display.clearDisplay();

  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_EEP ) - 1 ) * CHR_W * 2 ) / 2, 16,WHITE,true );
  display.print( FPSTR( LS_EEP ) );
  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_EEP1 ) - 1 ) * CHR_W * 2 ) / 2, 16 + 2 * LINE_H ,WHITE,true );
  display.print( FPSTR( LS_EEP1) );
  setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_EEPKEY )- 1 )* CHR_W ) / 2, 16 + 4 * LINE_H );
  display.print( FPSTR( LS_EEPKEY ) );

  display.display();
}

/**
 *  \brief    Display a MESSAGE screen.
 *  \remarks  A message screen consists of a screen header with underline and 3 rows of text.
 *            The first row is double size font, other rows are normal size.
 *            The message screen is displayed for a specified number of seconds before
 *            returning to the caller. The default time is zero for immediate return.
 *  \param [in] const __FlashStringHelper* line1        Line 1 text.
 *  \param [in] const __FlashStringHelper* line1        Line 2 text.
 *  \param [in] const __FlashStringHelper* line3        Line 3 text.
 *  \param [in] uint8_t                    displayTime  Delay time (default = 0)
 */
void message( const __FlashStringHelper *line1,
              const __FlashStringHelper *line2,
              const __FlashStringHelper *line3,
              uint8_t displayTime ) {

	display.clearDisplay();

	setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_MSGHDR ) - 1 ) * CHR_W ) / 2, 1 );
	display.print( FPSTR( LS_MSGHDR ) );
	display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );

	setTextProp( 2, 1, 16 );
	display.print( line1 );
	setTextProp( 1, 1, 16 + 2 * LINE_H );
	display.print( line2 );
	setTextProp( 1, 1, 16 + 3 * LINE_H );
	display.print( line3 );

	display.display();

	if ( displayTime ) delay( displayTime * 1000 );
}

/**
 *  \brief    Display the SPLASH screen.
 *  \remarks  This is shown once only at start-up and is for general information and advertising.
 */
void splash() {
	display.clearDisplay();
	display.display();

	setTextProp( 1, 1, 1 );
	display.print( F( _DEVICENAME_ ) ); // 21 chars per line
	setTextProp( 1, 1, 16 );
	display.print( F( "V." xstr( _VERSION_MAJOR_ ) "." xstr( _VERSION_MINOR_ ) "."
	                  xstr( _REVISION_ ) " " __DATE__ ) );
	setTextProp( 1, 1, 16 + LINE_H );
	display.print( F( "by " _AUTHOR_ ) );
	setTextProp( 1, 1, 16 + 2 * LINE_H );
	display.print( F( "Copyright (c) " _COPYRIGHT_ ) );
	setTextProp( 1, 1, 16 + 3 * LINE_H );
//	display.print( F( _COMPANY_ ) );
	display.print( F( _RIGHTS_ ) );
	setTextProp( 1, 1, 16 + 4 * LINE_H );

	display.print( F( _FREERAM_ ) );
	display.print(getFreeRAM());

	display.display();

	uint16_t timer = 0;

	while (Encoder.getButton() != ClickEncoder::Pressed && timer < SPLASHTIME)
	{
		delay(10);
		timer += 10;
	}


	display.clearDisplay();
	display.display();
}



/***************************************************************************************************
* Utility Conversion Functions                                                                     *
***************************************************************************************************/
/**
 *  \brief                    Returns a character string representing a formatted numeric value.
 *  \param [in] char *str     Pointer to the string to receive the formatted result.
 *  \param [in] uint16_t val_u  The integer value to be formatted.
 *  \param [in] float    val_f  The integer value to be formatted.
 *  \param [in] vf_Type fType The type of variable to be formatted.
 *  \return     char*         Pointer to the formatted string.
 */
char *valStr( char *str, uint16_t val_u, vf_Type fType ) {

	// We must resort to this awkward kludge to format the variables because variable width and
	// precision (* specifier) are not implemented in avr gcc - bugger!!!

	switch ( fType ) {
	    case VF_TEMPC:    sprintf_P( str, PSTR( "%5.1u C" ), val_u );                 break;
	    case VF_TEMPF:    sprintf_P( str, PSTR( "%5.1u F" ), val_u );                 break;
		case VF_RPM:  sprintf_P( str, PSTR( "%4.1u" ), val_u );                 break;
		case VF_VOLT:    sprintf_P( str, PSTR( "%5.1u" ), val_u );   break;
		case VF_RATIO:	 sprintf_P( str, PSTR( "1:%d.%02d" ), val_u/1000, val_u %1000 );  break;
		case VF_FLOAT:	sprintf_P( str, PSTR( "%d.%02d" ), val_u/100, val_u %100 );  break;
	}

	return str;
}

/***************************************************************************************************
* Utility EEPROM Functions                                                                         *
***************************************************************************************************/
/**
 *  \brief                    Reset the EEPROM and program data to factory default settings.
 *  \remarks                  EEPROM data is only written if the new data is different to the
 *                            existing data to limit EEPROM wearout.
 *  \param [in] boolean full  True to reset the weld count, battery offset, and screen inversion.
 */
void resetEeprom( boolean full ) {

	// Write the factory default data to the eeprom. In the case of a full reset, the weld count,
	// battery offset, and screen orientation are zeroed, otherwise they are left unchanged.


	pData.oledInvert 					= full ? DEF_OLED_INVERT : pData.oledInvert;
	pData.RPMBASISVOLT  				= VOLT_BASIS100;
	pData.REDRATIO						= DEF_REDRATIO;
	pData.HIGHRATIO						= DEF_HIGHRATIO;
	pData.LOWRATIO						= DEF_LOWRATIO;
	pData.TCelsius						= true;				// Celsius
	pData.maxTemperatur					= DEF_HIGH_TEMP_ALARM;
	pData.EEP_Version					= EEPROM_Version;
	pData.EncDiscSlots		    		= DEF_ENCDISCSLOT;
	pData.Last_RPM						= 100;
	// The put function does not write new data if the existing data is the same thereby
	// limiting eeprom wearout.
	EEPROM.put( EEA_PDATA, pData );

	// The unique id is a simple method to ensure that a valid data set exists in the eeprom
	// (there are much better methods but we don't have the code space to spare).
	EEPROM.put( EEA_ID, EE_UNIQUEID );

#if defined  _DEVELOPMENT_ || defined _BOOTSYS_

		if ( full ) Serial.println( F( "EEPROM Full Reset" ) );
		else Serial.println( F( "EEPROM Reset" ) );

#endif /* _DEVELOPMENT_ || _BOOTSYS_*/
}

bool loadEeprom() {
	// Check the eeprom integrity by reading a magic number. If it is corrupt then return = false
	// else return = true

	uint32_t uniqueID;
	bool EEPROM_OK = true;

	EEPROM.get( EEA_ID, uniqueID );

	if ( uniqueID != EE_UNIQUEID )	EEPROM_OK = false;
	else
		EEPROM.get( EEA_PDATA, pData );

	if (pData.EEP_Version != EEPROM_Version) EEPROM_OK = false;

	return EEPROM_OK;

}
/**
 *  \brief    Udates the EEPROM data with local program data structure.
 *  \remarks  EEPROM data is only written if the new data is different to the
 *            existing data to limit EEPROM wearout.
 */
void updateEeprom(bool force)
{
	static unsigned long lastEEUpdatetime = 0;

	//
//	// Do not do this too often to prevent premature eeprom wearout.
	if ((millis() - lastEEUpdatetime > EEPROM_UPDATE_T) || force)
	{
		lastEEUpdatetime = millis();

		// Write the current program data to the eeprom.
		EEPROM.put( EEA_PDATA, pData);

#ifdef _DEVELOPMENT_
			Serial.println( F( "Updated EEPROM" ) );
#endif /* _DEVELOPMENT_ */
	}
}


/************************************************************************
* Funktion: gedrückte und entprellte Taste ermitteln					*
************************************************************************/
uint8_t get_key_press( uint8_t key_mask )
{
    uint8_t sreg = SREG;

    cli();                      // disable all interrupts

    key_mask &= key_press;      // read key(s)
    key_press ^= key_mask;      // clear key(s)

    SREG = sreg;                // restore status register

    return key_mask;
}




///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state(uint8_t key_mask)

{
    uint8_t sreg = SREG;

	cli();
	// disable all interrupts

	key_mask &= key_state;

    SREG = sreg;                // restore status register
	return key_mask;
}



int getFreeRAM()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//void I2C_Scanner()
//{
//  byte error, address;
//  int nDevices;
//
//  Serial.println("Scanning...");
//
//  nDevices = 0;
//  for(address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
//
//    if (error == 0)
//    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
//
//      nDevices++;
//    }
//    else if (error==4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
//    }
//  }
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
//
////  delay(5000);           // wait 5 seconds for next scan
//}

