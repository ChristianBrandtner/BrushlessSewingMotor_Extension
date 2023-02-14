#ifndef _SMC_OLED_V15_H
#define _SMC_OLED_V15_H

// General macros
#define str(s)              #s
#define xstr(s)             str(s)


#define _DEVICENAME_        "SMC Extension"

#define _AUTHOR_            "Chr.Brandtner"
#define _VERSION_MAJOR_     15
#define _VERSION_MINOR_     3
#define _REVISION_          0
#define _COPYRIGHT_         "2022"
//#define _COMPANY_           " "
#define _RIGHTS_            "GPL-3.0 license"
#define _FREERAM_			"Free RAM (byte): "

/***************************************************************************************************
* Configuration                                                                               *
***************************************************************************************************/

//#define _PCB_11_5_							// first Version
#define _PCB_14_5_							// second Version


#define _HARDWARE_RPM_						// RPM Messung über Proximity Sensor oder Lichtschranke

#define _LANG_EN_                           /**< Language:  _LANG_EN/DE/FR/ES/IT_ */

#define _SERIAL_BAUD_       115200          /**< Comms rate for serial debugging */

#define _DP4_Low_Timeout_ 	30				// nach 30ms wird automatisch DP4 auf aus gesetzt
#define _RAMPDOWNTIME_		25				// ms
#define _DAC_VAL100_		19				// DAC Wert je 100 U/Min Differenz


/***************************************************************************************************
* Pin  definitions                                                                    *
***************************************************************************************************/

// Data wire is plugged into port A0 on the Arduino
#define ONE_WIRE_BUS 		A0
#define DAC_ADRESS			0x60


#define ENCODER_PINA  A3
#define ENCODER_PINB  A2
#define ENCODER_BTN	  A1
#define ENCODER_STEPS_PER_NOTCH    4   // Change this depending on which encoder is used

#define START_BUTTON_PIN 	PD4
#define START_LED_PIN		11
#define STOP_BUTTON_PIN 	PD3
#define STOP_LED_PIN		10
#define DIR_BUTTON_PIN 		PD5

#define DIR_LED_PIN			9
#define BOARD_LED_PIN		13

#define RPM_SENSOR_PIN		8
#define SECURITY_SW_PIN		PD2
#define	SMC_DP4_PIN			PD7			// Input from SMC
#define SMC_SW_PIN			PD6			// Output to SMC
#define	ADC_INPUT_PIN		A6

#define START_LED		0
#define STOP_LED		1
#define DIR_LED			2
#define BOARD_LED		3




#define KEY_PIN             PIND // Port D als Input

#define KEY_ALL             ((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << DIR_BUTTON_PIN)| (1 <<SMC_DP4_PIN))

#define LONG_MASK           ((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << DIR_BUTTON_PIN)| (1 <<SMC_DP4_PIN))

#define REPEAT_MASK         ((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << DIR_BUTTON_PIN)| (1 <<SMC_DP4_PIN))
#define LONG_REPEAT_MASK    ((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << DIR_BUTTON_PIN)| (1 <<SMC_DP4_PIN))
#define LONG_REPEAT_SP_MASK ((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << DIR_BUTTON_PIN)| (1 <<SMC_DP4_PIN))

#define REPEAT_START       70   // after 700ms
#define REPEAT_NEXT        15   // every 150ms
#define REPEAT_SPEED_1     20   // every 200ms
#define REPEAT_SPEED_2      8   // every  80ms
#define REPEAT_SPEED_3      1   // every  10ms


/***************************************************************************************************
* Defaults                                                                                          *
***************************************************************************************************/


#define DEF_ROTATIONALSPEED			1000               /**< Default RPM */
#define DEF_MAX_ROTATIONALSPEED    	4500             /**< Default maximum RPM */
#define DEF_HIGH_TEMP_ALARM 		65              /**< Default high temperature, Celsius */
#define DEF_GEARSET		    		true            /**< Default Getriebestellung High */
#define DEF_OLED_INVERT     		false           /**< Default OLED orientation */
#define DEF_REDRATIO				53			   /**< Default Reduction Ratio Antrieb 16Z Abtrieb 30Z*/
#define DEF_HIGHRATIO				95				/**< Default Reduction Ratio High Speed*/
#define DEF_LOWRATIO				41				/**< Default Reduction Ratio Low Speed*/
#define DEF_ENCDISCSLOT				24				/**< Default slots in encoderdisc*/

// Daten für die Berechnung des DAC Wertes RPM
// Berechnung: (Drehzahl  * (VOLTDIFF_100/100) + VOLT_BASIS100

#define VOLTDIFF_100				0.023			/**< Spannungsdifferenz je 100 RPM */
#define	VOLT_BASIS100				301				/**< mV Basisspannung für 100 RPM */

#define MIN_BASIS100				200				/**< minimale mV Basisspannung für 100 RPM */
#define MAX_BASIS100				400				/**< maximale mV Basisspannung für 100 RPM */

#define MIN_ROTATIONALSPEED	        100             /**< Minimum RPM */
#define MAX_ROTATIONALSPEED	        4500            /**< Maximum RPM */

#define STOP_VOLT			1.2 * MCP4725_BITS_PER_VOLT		/**< Volt Motor Stop*/
#define HALT				1
#define STOP				0


#define MIN_REDRATIO		1
#define MAX_REDRATIO		3000

#define MIN_GEARRATIO		1
#define MAX_GEARRATIO		100

#define MIN_TEMP			1
#define MAX_TEMP			90

#define MIN_ENCSLOTS		4
#define MAX_ENCSLOTS		120

// Display screen layout
#define CHR_W               6               /**< Width of character [size=1] (pixels) */   
#define CHR_H               8               /**< Height of character [size=1] (pixels) */   
#define LINE_H              (CHR_H+2)       /**< Height of line [size=1] (pixels) */   

// Macros to define logical states
#define B_DN                true            /**< General macro for DOWN state */
#define B_UP                false           /**< General macro for UP state */

// EEPROM macros
#define EEA_ID              0               /**< Address of unique ID */
#define EEA_PDATA           (EEA_ID+4)      /**< Eeprom address of program data */
#define EE_UNIQUEID         0x18fae9cb      /**< Unique Eeprom verification ID */
#define EE_FULL_RESET       true            /**< Reset parameter to reset all Eeprom parameters */

/** WString.h */
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))


// Macros masquerading as functions - makes the code more readable
/** This macro reads the state of the DP4  Input . */
#define DP4State()          (digitalRead(SMC_DP4_PIN))

#define btnState()          (!digitalRead(ENCODER_BTN))

/***************************************************************************************************
* OLED Display Configuration                                                                       *
***************************************************************************************************/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET          -1               /**< OLED mode */
#define OLED_INVERT         2               /**< OLED orientation mode */
#define SPLASHTIME          3000            /**< Splash screen time (ms) */

/***************************************************************************************************
* Temperatursensor Configuration                                                                       *
***************************************************************************************************/

#define TEMPCHECKTIME  10000			// Intervall zum Temperaturmessen (ms)

/***************************************************************************************************
* EEPROM Update Configuration                                                                       *
***************************************************************************************************/

#define EEPROM_UPDATE_T     60000            /**< EEPROM update time (ms) */


/***************************************************************************************************
* Configdata Configuration                                                                       *
***************************************************************************************************/

#define EEPROM_Version			2			/**< EEPROM Version*/

typedef  struct   progData {                /**< Program operating data structure */
         uint8_t  TCelsius;                 /**< Temperature in Celsius/Fahrenheit */
         uint8_t  maxTemperatur;            /**< maximum Temperature in Celsius */
         int16_t RPMBASISVOLT;				/**< mV Basis Wert für 0 RPM Motor*/
         int16_t REDRATIO;					/**< Motor Untersetzung Antrieb/Abtrieb Zahnriemen*/
         int16_t HIGHRATIO;					/**< Spindelgetriebe Untersetzung Highspeed*/
         int16_t LOWRATIO;					/**< Spindelgetriebe Untersetzung Lowspeed*/
         unsigned oledInvert;				/**< Display Ausrichtung */
         uint8_t EncDiscSlots;				/**< number of slots eg. magnets in Encoderdisc */
         int16_t Last_RPM;					/**< last active RPM */
         uint16_t EEP_Version;				/**< Versionnumber, at increased every struct change*/

} progData;






/***************************************************************************************************
* MCP4725 additional things                                                                  *
***************************************************************************************************/


#define MCP4725_RESOLUTION           12		// dacs resolution
#define MCP4725_STEPS                4096	// dacs avaible steps


#define MCP4725_REFERENCE_VOLTAGE    5.0   	// ref voltage set to 5 V
#define MCP4725_MAX_VALUE            4095 	// steps max value, 4096 - 1

#define MCP4725_BITS_PER_VOLT		MCP4725_STEPS / MCP4725_REFERENCE_VOLTAGE

#define _DAC_INITVAL		1474	// = 1,8V Init Value for DAC EEprom

/***************************************************************************************************
* Structure, union, and enumerated type definitions                                                *
***************************************************************************************************/

typedef  enum {                             /**< Type enumerations for format of variables */
         VF_TEMPC,                          /**< Temperature Celsius*/
		 VF_TEMPF,                          /**< Temperature Fahrenheit */
		 VF_RPM,   		                    /**< Rational Speed*/
		 VF_VOLT,							/**< DAC RPM Volt */
		 VF_RATIO,							/**< Gear Ratio */
		 VF_FLOAT							/**< uint16_t to float z.B 301 > 3.01 */
} vf_Type;

/***************************************************************************************************
* Procedure prototypes                                                                             *
***************************************************************************************************/

void    stateMachine();

void    resetEeprom(boolean = false);
bool    loadEeprom();

void 	updateEeprom(bool force);
void 	checkEncoderEvent();
void    checkForLowVoltageEvent();
void    checkForSleepEvent();
void    checkForBtnEvent();
void 	checkForDP4Event();
int16_t CalcRPM(int16_t Drehzahl);

void 	CheckButtons();
void     checkTemperatur();

void     isr();
void     splash();

void     message(const __FlashStringHelper*, const __FlashStringHelper*,
                 const __FlashStringHelper*, uint8_t = 0);
void     displayMenuType1(const __FlashStringHelper*, const __FlashStringHelper*,
                          const __FlashStringHelper*, const __FlashStringHelper*, 
                          uint8_t SelectedItem);
void     displayMenuType2(const __FlashStringHelper*, const char*, const __FlashStringHelper*);
void     displayMainScreen();
void     displayPulseData();
void     displayLowBattery();
void     displayHighBattery();
void     displayHighTemperature();
void 	 displaySecurityStop();
void 	 displayEEPROM_Reset();
void     drawStatusLine();
void     setTextProp(uint8_t, uint8_t, uint8_t, uint16_t = WHITE, boolean = false);

char 	*valStr( char *str, uint16_t val_u, vf_Type fType );

void drawValue(  uint16_t Value);

int16_t SetDrehzahl(int16_t Drehzahl);

void SetDAC_Volt (uint16_t Volt);

uint8_t get_key_press( uint8_t key_mask );
uint8_t get_key_release(uint8_t key_mask);
uint8_t get_key_state(uint8_t key_mask);
uint8_t DebouncePin(uint8_t pin);

int getFreeRAM();

void I2C_Scanner();

void InitializeInterrupt();


/***************************************************************************************************
* Language strings (very simple language implementation - English is the default)                  *
***************************************************************************************************/

// Copy the language strings from the else clause into your language specific clause, then alter
// the strings to suit your language. Define your language at the top of this header file. It is
// important to maintain the correct formatting of the strings. Each string has an inline comment
// defining its format.

// Comment legend: field width          21 for small font, 10 for large font
//                 justification        left, centre, right
//                 padded               spaces both ends to fill field

#ifdef _LANG_DE_
          
#elif defined _LANG_FR_

#elif defined _LANG_ES_

#elif defined _LANG_IT_

#else
//                                           0123456789               // large font
//                                           012345678901234567890    // small font
//System Menu
static const char LS_SYSMENU[]    	PROGMEM = "System Menu";            // 21 char, left
static const char LS_RPMDISP[]    	PROGMEM = "Speed";            	// 10 char, centre, padded
static const char LS_TEMPERATUR[]  	PROGMEM = "temperatur";            // 10 char, centre, padded

static const char LS_GEARENC[]    	PROGMEM = "gear / ENC";            // 10 char, centre, padded

// Encoder menü
static const char LS_DACMENU[]    	PROGMEM = "gear/encoder settings";           // 21 char, left

static const char LS_ENCSLOTS_S[]  PROGMEM = "# encoderdisc slots";    	// 21 char, left
static const char LS_ENCSLOTS[]   	PROGMEM = "# slots";            	// 10 char, centre, padded

static const char LS_GEAR[]        	PROGMEM = "gears";             // 10 char, centre, padded

//Gear Menu
static const char LS_GEARMENU[]    	PROGMEM = "gear ratio settings";          // 21 char, left

static const char LS_DRIVERATIO[]  	PROGMEM = "driveRatio";            	// 10 char, centre, padded
static const char LS_DRIVERATIO_S[] PROGMEM = "motor reduct. ratio";   	// 21 char,

static const char LS_HIRATIO[]    	PROGMEM = "speed  Hi";            	// 10 char, centre, padde
static const char LS_HIRATIO_S[]  	PROGMEM = "speed shifter Hi";   	// 21 char,

static const char LS_LORATIO[]    	PROGMEM = "speed  Lo";            	// 10 char, centre, padde
static const char LS_LORATIO_S[]  	PROGMEM = "speed shifter Lo";   	// 21 char,

static const char LS_REDRATIO[]    	PROGMEM = "red. ratio";            	// 10 char, centre, padde


static const char LS_SPINDEL_LOW[]   PROGMEM = "Lo";    // 21 char, left
static const char LS_SPINDEL_HIGH[]   PROGMEM = "Hi";    // 21 char, left
static const char LS_MOTOR_DREHZAHL[]     PROGMEM = "M:";    // 21 char, l

// Temperatur Menu
static const char LS_TEMPALM[]    PROGMEM = "Temperature Settings";      // 21 char, left
static const char LS_CEL[]        PROGMEM = "celsius";        // 21 char, left
static const char LS_FAH[]        PROGMEM = "fahrenheit";        // 21 char, left

static const char LS_TEMP[]       PROGMEM = "TEMP";                   // 10 char, left
static const char LS_HIGHT[]      PROGMEM = "Temp.Alarm";              // 10 char, left

static const char LS_COOL[]       PROGMEM = "COOL DOWN!";       // 21 char, left

static const char LS_STOP[]       PROGMEM = "Security";            // 10 char, left
static const char LS_STOP1[]      PROGMEM = "Switch";              // 10 char, left
static const char LS_STOPTEXT[]   PROGMEM = "SECURITY SWITCH OPEN";       // 21 char, left

static const char LS_EEP[]        PROGMEM = "EEPROM";            	// 10 char, left
static const char LS_EEP1[]      PROGMEM = "RESET";              	// 10 char, left
static const char LS_EEPKEY[]     PROGMEM = "Click "; // 21 char, left

static const char LS_EXIT[]       PROGMEM = "Exit";             // 10 char, centre, padded

static const char LS_LEFT[]			PROGMEM = "L";				// 10 char, centre, padded
static const char LS_RIGHT[]		PROGMEM = "R";				// 10 char, centre, padded

#define 		  LS_EEPROMDASI   LS_EEP				// 10 char, centre, padded
static const char LS_EEPROMUPD[]  PROGMEM =  "data backup";		// 21 char, left

static const char LS_RPM[]         PROGMEM = "RPM";                     // 2  char, left

static const char LS_BLANK[]      PROGMEM = "          ";                      // 1  char, left
static const char LS_TCUNITS[]     PROGMEM = " C";                    // 2  char, left
static const char LS_TFUNITS[]     PROGMEM = " F";                    // 2  char, left

static const char LS_DISPLAY[]    PROGMEM = "Display";             // 10 char, centre, padded
static const char LS_SETTMENU[]   PROGMEM = "System Settings";        // 21 char, left

static const char LS_EEPROM[]    PROGMEM =   "EEP Reset";             // 10 char, centre, padded

static const char LS_INVERTMENU[] PROGMEM = "Orientation";     // 21 char, left
static const char LS_SCRNORM[]    PROGMEM = "NORMAL";                 // 10 char, left
static const char LS_SCRINV[]     PROGMEM = "INVERTED";               // 10 char, left
static const char LS_MSGHDR[]     PROGMEM = "System";         // 21 char, left


# define 		  LS_NOSECURITY    LS_STOP               // 10 char, centre
static const char LS_DACERROR[]      PROGMEM = "DAC/I2C Error";          // 10 char, centre
static const char LS_TEMPERROR[]     PROGMEM = "Temp Sens Error";        // 10 char, centre
static const char LS_SMCERROR[]     PROGMEM = "SMC Error";        // 10 char, centre

#endif            
#endif //


 
