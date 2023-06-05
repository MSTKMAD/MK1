
//******************************
//   VERSION r10.0-30
//*****************************
#define VERSION 730
/**************************************************
*******************
  EDCPSU Tattoo DUAL edition HW r10.0 (MK2 DUAL OUTPUT)
  14 mar 2022

  VERSION: see above

*********************************************************************/

//----------------------------------------------  INCLUDE SECTION --------------------------------------------
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//----------------------------------------------  MACROS DEFINITION --------------------------------------------
// macros from DateTime.h
#define SECS_PER_MIN (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY (SECS_PER_HOUR * 24L)
#define SECS_9H (SECS_PER_HOUR * 9L)

/* Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) ((_time_ % SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) (_time_ / SECS_PER_DAY)

//--------------------------------------------------------------------------------------------------------------

// EDCPSU Configuration
#define OLED_DC 12
#define OLED_CS 10
#define OLED_CLK 13
#define OLED_MOSI 11
#define OLED_RESET 8

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 1
#define I2C_NOINTERRUPT 1
#define I2C_SLOWMODE 1
#include <SoftI2CMaster.h>

Adafruit_SSD1306 display(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define XPOS 0
#define YPOS 1
#define DELTAY 2

//----------------------------------------------  CONST DEFINITION --------------------------------------------

const uint16_t LenDCDCLookupTable = 141; // 2V to 16V
const byte ADDR_I2C_DCDC = 0;
const int DEBOUNCING = 0x55;
const int NO_DEBOUNCING = 0xAA;
const int DEBOUNCE_PUSHB_TIME = 100;   //  milliseconds
const int DEBOUNCE_ROTPUSH_TIME = 200; //  milliseconds
const int DEBOUNCE_ROTARY_TIME = 30;   //  milliseconds
const int DEBOUNCE_PEDAL_TIME = 200;   //  milliseconds
const int DCDC_ENABLED = HIGH;
const int DCDC_DISABLED = LOW;
const int PEDAL_ON = LOW;
const int PEDAL_OFF = HIGH;
const int LED_ON = LOW;
const int LED_OFF = HIGH;
const int PUSHBUTTON_ON = LOW;
const int PUSHBUTTON_OFF = HIGH;
const int FLAG_ON = true;
const int FLAG_OFF = false;
const int NUM_MEMORY = 2;
const int SAVE_MEM_TIMER = 2000;       // Milliseconds
const int EEPROM_RECORDED_DONE = 0xDA; // THis signature indicates if the EEPROM is recorded or if it is brand new and not recorded
const int EEPROM_POLARITY_STATUS = 0x19;
;                                     //Initial value for the polarity
const int EEPROM_NITRO_STATUS = 0X20; // Position of EEPROM NITRO STATUS  --> INDICATES if NITRO is active or not
const int EEPROM_RECORD_STAT = 0x21;  // Position of EEPROM RECORDED FLAG --> INDICATES if EEPROM is recorded (when value = EEPROM_RECORDED_DONE)
const int EEPROM_MNF_YEAR = 0xF0;     // Position of EEPROM MANUFACTURED YEAR (Binary)
const int EEPROM_MNF_MONTH = 0xF1;    // Position of EEPROM MANUFACTURED YEAR (Binary)
const int EEPROM_SERIAL_NO = 0xF2;    // Position of EEPROM MANUFACTURED YEAR (Binary)
const byte NITRO_CFG_NO = 1;          // Nitro Signature configured as NOT (menu selection 1)
const byte NITRO_CFG_YES = 2;         // Nitro Signature configured as YES (menu selection 2)
const int MACHINE_EEPROM_POS0 = 0;    // Position 1 of EEPROM --> Stores the position 0 of Machine 1 memory
const int MACHINE_EEPROM_POS1 = 1;    // Position 1 of EEPROM --> Stores the position 0 of Machine 1 memory
const int MACHINE_EEPROM_POS2 = 2;    // Position 1 of EEPROM --> Stores the position 0 of Machine 1 memory
const int MACHINE_EEPROM_POS3 = 3;    // Position 1 of EEPROM --> Stores the position 0 of Machine 1 memory
const int MACHINE1_OFFSET = 0;        // Offset for both MachinesMemory[] array and EEPROM corresponding to MACHINE_1 WorkingPoint Memories
const int MACHINE2_OFFSET = 4;        // Offset for both MachinesMemory[] array and EEPROM corresponding to MACHINE_2 WorkingPoint Memories
const int MACHINE3_OFFSET = 8;        // Offset for both MachinesMemory[] array and EEPROM corresponding to MACHINE_3 WorkingPoint Memories
const byte NITRO_SETUP_OFF = 0;       // If Nitro setup is taking place
const byte NITRO_SETUP_ON = 0xAA;     // If No Nitro setup is taking place
const byte BEEP_IS_TRUE = true;
const byte BEEP_IS_FALSE = false;
const byte WRITE_MESSG = 1;
const byte DELETE_MESSG = 2;
const byte INFO_MESSG = 1;
const byte WARNING_MESSG = 2;
const byte VIP_MESSG = 3;
const byte NITRO_MESSG = 4;
const byte RUNMODE_NORMAL = 0;
const byte RUNMODE_CONFIG_TOP = 1;
const byte RUNMODE_CONFIG_NITRO = 2;
const byte RUNMODE_MENU_TIMER = 3;
const byte RUNMODE_CHANGE_POL = 4;
const byte LEN_VOUT_BUFFER = 4; // Lenght of VoutBuffer
const byte OVC_SCAN_INPROGRESS = 1;
const int LOW_PITCH = 500;                  // Microseconds between buzzer pulses
const int HIGH_PITCH = 200;                 // Microseconds between buzzer pulses
const int OVC_RECOVERY_TIME = 1500;         // Milliseconds
const unsigned long STANDBY_TIME = 3600000; // 1h
const int POL_NORMAL = 0;                   // Normal polarity = Relay resting/not-energized
const int POL_REVERSE = 1;                  // Revere polarity = Relay energized
const int MAX_THRESHOLD_NITRO = 95;         // Corresponds to EncoderPosition for 11.5V
const int HIGH_THRESHOLD_NITRO = 50;        // Corresponds to EncoderPosition for 7V
const int PUSHBUTTON_IDLE = 0;
const int PUSHBUTTON_ON_EDGE = 1;
const int PUSHBUTTON_FALL = 2;
const int PUSHBUTTON_LONGPRESS = 3;
const unsigned long LONG_PRESS_TIME = 300;   // Milliseconds
const unsigned long OVC_ALARM_TIMER = 2000;  // Milliseconds
const unsigned long MEM_DISPLAY_TIME = 2000; // Milliseconds
const int LenNITROLookupTable = 20;

const int OVC_SENSE_MAX_TIME = 2000;          // Maximum time that can last the overcurrent with SENSE method (in Milliseconds)
const int OVC_UVOLT_MAX_TIME = 1000;          // Maximum time that can last the overcurrent with the UNDERVOLTAGE method (Milliseconds)
const unsigned int OVC_SENSE_LIMIT_INF = 760; // Current limit above which it is still considered an overcurrent sense situation 2.6A
const unsigned int OVC_SENSE_LIMIT_SUP = 800; // Current limit above which it is Triggered the overcurrent sense situation 2.7A real (in theory this value should trigger at 3.1A but Rsense is contaminated by a trace track to the amplifier)
const byte MAX_OVC_ERRORS = 5;          // Max number of OVC errors per session in order to request the release of the PEDAL to the user
const int UNDERVOLT_1V5 = 100;          // Sensed voltage corresponding to 2V for undervoltage sensing
const int UNDERVOLT_1V8 = 121;          // Sensed voltage corresponding to 1.8V for undervoltage sensing
const float DISP_TO_VTARGET_CONV = 5.2; // Conversion factor to get display values into same scale as VoutSense values (1/19.6)*1023 = 52.2 --> to DISP 52/10=5.2
const byte DISPLAY_MEM = 1;
const byte NO_DISPLAY_MEM = 2;
const byte CLEAR_DISPLAY_MEM = 3;

const byte NO_LONGPRESS_INFO = 0;
const byte LONGPRESS_INFO_CONT = 1;
const byte LONGPRESS_INFO_NO_CONT = 2;
const byte LONGPRESS_INFO_TIMER_ON = 3;
const byte LONGPRESS_INFO_TIMER_OFF = 4;
const unsigned long SHOW_LONGPRESS_TIME = 800; // Time that the longpress information is shown on display (Milliseconds)

const unsigned long RX_CHAR_TIMEOUT = 10; // (ms) time allowed for an orphan char received via Serial
const byte TLG_RX_BYTES = 3;              // Bytes compounding a Serial telegram of the Test mode

const byte ALARM_FLAG = 0; // Flag to report that OVC_ALARM sensor is activated. This is set in ReportFlags report byte in the Test Mode
const byte PEDAL_FLAG = 1; // Flag to report that PEDAL input is activated. This is set in ReportFlags report byte in the Test Mode
// ------------------------------------------------- PIN DEFINITION r7,r6 -------------------------------------------
const int DCDC_EN = A1;  //
const int LED_FRONT = 9; // Arduino 7 (pin 11 in ATMEGA368)
const int CHG_POL = 7;   // Arduino 9 (pin 13 in ATMEGA368)
const int ISEN = A7;
const int VOSEN = A6;
const int PUSHBUTTON_IP = A2; // HIGH = OFF; LOW = ON
const int ROTPUSH_IP = A3;    // HIGH = OFF; LOW = ON
const int ROTA = 4;
const int OVC_ALARM = 5; // New in r8.0 (before pin 2)
const int ROTB = 3;
const int BUZZ = 6;
const int PEDAL_IP = 2;      // New in r8.0 (before pin 5)
const int ANALOG_PEDAL = A0; // New in r8.0 (before LED)
const int CHANNEL_SEL = 0;   // RXi-- caution! this digital line overlaps with RXi, so Serial has to be disabled
const int ENA_OUT = 1;       // TXo-- caution! this digital line overlaps with TXo, so Serial has to be disabled

//-------------------------- PROGMEM DEFINITION -----------------------------------------------------------------

const static unsigned char __attribute__((progmem)) MusotokuLogo[] = {
    0xE0, 0x06, 0x30, 0x0C, 0x3F, 0x00, 0xFC, 0x00, // ###          ##   ##        ##    ######        ######
    0xE0, 0x06, 0x30, 0x0C, 0x71, 0x81, 0xFE, 0x00, // ###          ##   ##        ##   ###   ##      ########
    0xE0, 0x0E, 0x30, 0x0C, 0x60, 0xC3, 0x03, 0x00, // ###         ###   ##        ##   ##     ##    ##      ##
    0xE0, 0x0E, 0x30, 0x0C, 0xC0, 0x62, 0x03, 0x00, // ###         ###   ##        ##  ##       ##   #       ##
    0xA0, 0x0A, 0x30, 0x0C, 0xC0, 0x62, 0x01, 0x00, // # #         # #   ##        ##  ##       ##   #        #
    0xB0, 0x0A, 0x30, 0x0C, 0xC0, 0x66, 0x01, 0x80, // # ##        # #   ##        ##  ##       ##  ##        ##
    0xB0, 0x0A, 0x30, 0x0C, 0xC0, 0x66, 0x01, 0x80, // # ##        # #   ##        ##  ##       ##  ##        ##
    0x90, 0x1A, 0x30, 0x0C, 0xC0, 0x06, 0x01, 0x80, // #  #       ## #   ##        ##  ##           ##        ##
    0x90, 0x12, 0x30, 0x0C, 0xC0, 0x06, 0x01, 0x80, // #  #       #  #   ##        ##  ##           ##        ##
    0x98, 0x12, 0x30, 0x0C, 0x60, 0x06, 0x01, 0x80, // #  ##      #  #   ##        ##   ##          ##        ##
    0x98, 0x32, 0x30, 0x0C, 0x70, 0x06, 0x01, 0x80, // #  ##     ##  #   ##        ##   ###         ##        ##
    0x98, 0x32, 0x30, 0x0C, 0x18, 0x06, 0x01, 0x80, // #  ##     ##  #   ##        ##     ##        ##        ##
    0x88, 0x32, 0x30, 0x0C, 0x0E, 0x06, 0x01, 0x80, // #   #     ##  #   ##        ##      ###      ##        ##
    0x8C, 0x22, 0x30, 0x0C, 0x07, 0x06, 0x01, 0x80, // #   ##    #   #   ##        ##       ###     ##        ##
    0x8C, 0x62, 0x30, 0x0C, 0x03, 0x86, 0x01, 0x80, // #   ##   ##   #   ##        ##        ###    ##        ##
    0x8C, 0x62, 0x30, 0x0C, 0x00, 0xC6, 0x01, 0x80, // #   ##   ##   #   ##        ##          ##   ##        ##
    0x84, 0x62, 0x30, 0x0C, 0x00, 0xC6, 0x01, 0x80, // #    #   ##   #   ##        ##          ##   ##        ##
    0x84, 0x42, 0x30, 0x0C, 0x00, 0x66, 0x01, 0x80, // #    #   #    #   ##        ##           ##  ##        ##
    0x86, 0x42, 0x30, 0x0C, 0xC0, 0x66, 0x01, 0x80, // #    ##  #    #   ##        ##  ##       ##  ##        ##
    0x86, 0xC2, 0x30, 0x0C, 0xC0, 0x66, 0x01, 0x80, // #    ## ##    #   ##        ##  ##       ##  ##        ##
    0x82, 0xC2, 0x30, 0x1C, 0xC0, 0x66, 0x01, 0x00, // #     # ##    #   ##       ###  ##       ##  ##        #
    0x82, 0x82, 0x30, 0x1C, 0xC0, 0x62, 0x01, 0x00, // #     # #     #   ##       ###  ##       ##   #        #
    0x83, 0x82, 0x18, 0x18, 0xC0, 0xC3, 0x03, 0x00, // #     ###     #    ##      ##   ##      ##    ##      ##
    0x83, 0x82, 0x1C, 0x30, 0x61, 0xC1, 0xFE, 0x00, // #     ###     #    ###    ##     ##    ###     ########
    0x83, 0x82, 0x0F, 0xE0, 0x3F, 0x80, 0xFC, 0x00, // #     ###     #     #######       #######       ######
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //
    0x07, 0xFE, 0x0F, 0x84, 0x02, 0x20, 0x10, 0x00, //      ##########     #####    #        #   #        #
    0x00, 0x20, 0x10, 0x84, 0x02, 0x20, 0x10, 0x00, //           #        #    #    #        #   #        #
    0x00, 0x20, 0x20, 0x44, 0x04, 0x20, 0x10, 0x00, //           #       #      #   #       #    #        #
    0x00, 0x20, 0x20, 0x44, 0x08, 0x20, 0x10, 0x00, //           #       #      #   #      #     #        #
    0x00, 0x20, 0x40, 0x24, 0x00, 0x20, 0x10, 0x00, //           #      #        #  #            #        #
    0x00, 0x20, 0x40, 0x24, 0x10, 0x20, 0x10, 0x00, //           #      #        #  #     #      #        #
    0x00, 0x20, 0x40, 0x24, 0x20, 0x20, 0x10, 0x00, //           #      #        #  #    #       #        #
    0x00, 0x20, 0x40, 0x24, 0x20, 0x20, 0x10, 0x00, //           #      #        #  #    #       #        #
    0x00, 0x20, 0x40, 0x24, 0x40, 0x20, 0x10, 0x00, //           #      #        #  #   #        #        #
    0x00, 0x20, 0x40, 0x24, 0x80, 0x20, 0x10, 0x00, //           #      #        #  #  #         #        #
    0x00, 0x20, 0x40, 0x24, 0x80, 0x20, 0x10, 0x00, //           #      #        #  #  #         #        #
    0x00, 0x20, 0x40, 0x25, 0x80, 0x20, 0x10, 0x00, //           #      #        #  # ##         #        #
    0x00, 0x20, 0x40, 0x26, 0xC0, 0x20, 0x10, 0x00, //           #      #        #  ## ##        #        #
    0x00, 0x20, 0x40, 0x26, 0x40, 0x20, 0x10, 0x00, //           #      #        #  ##  #        #        #
    0x00, 0x20, 0x40, 0x24, 0x20, 0x20, 0x10, 0x00, //           #      #        #  #    #       #        #
    0x00, 0x20, 0x40, 0x24, 0x10, 0x20, 0x10, 0x00, //           #      #        #  #     #      #        #
    0x00, 0x20, 0x40, 0x24, 0x10, 0x20, 0x10, 0x00, //           #      #        #  #     #      #        #
    0x00, 0x20, 0x40, 0x24, 0x08, 0x20, 0x10, 0x00, //           #      #        #  #      #     #        #
    0x00, 0x20, 0x20, 0x44, 0x08, 0x30, 0x30, 0x00, //           #       #      #   #      #     ##      ##
    0x00, 0x20, 0x20, 0x44, 0x04, 0x10, 0x20, 0x00, //           #       #      #   #       #     #      #
    0x00, 0x20, 0x20, 0x44, 0x02, 0x10, 0x20, 0x00, //           #       #      #   #        #    #      #
    0x00, 0x20, 0x1F, 0x84, 0x02, 0x0F, 0xC0, 0x00, //           #        ######    #        #     ######
};

const static unsigned char __attribute__((progmem)) NitroLogo[] = {
    0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        ###########
    0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        ###########
    0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        ###########
    0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        ###########
    0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        ###########
    0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        ###########
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################                    ####
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################                   #####
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################                  ######
    0x7C, 0x00, 0x1F, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####                 ######
    0x7C, 0x00, 0x1F, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####                ######
    0x7C, 0x00, 0x1F, 0x00, 0x01, 0xF8, 0x00, 0xE0, 0x38, 0xF0, 0xC0, 0x00, 0x00, 0x00, //  #####             #####               ######           ###       ###   ####    ##
    0x7C, 0x00, 0x1F, 0x00, 0x03, 0xF0, 0x00, 0xF0, 0x38, 0xF3, 0xC0, 0x00, 0x00, 0x00, //  #####             #####              ######            ####      ###   ####  ####
    0x7C, 0x00, 0x1F, 0x00, 0x07, 0xE0, 0x00, 0xF8, 0x38, 0x03, 0xC0, 0x00, 0x00, 0x00, //  #####             #####             ######             #####     ###         ####
    0x7C, 0x00, 0x1F, 0x00, 0x0F, 0xE0, 0x00, 0xF8, 0x38, 0xF7, 0xF3, 0xDC, 0x7C, 0x00, //  #####             #####            #######             #####     ###   #### #######  #### ###   #####
    0x7C, 0x00, 0x1F, 0x00, 0x1F, 0xC0, 0x00, 0xFC, 0x38, 0xF7, 0xF3, 0xFC, 0xFF, 0x00, //  #####             #####           #######              ######    ###   #### #######  ########  ########
    0x7C, 0x00, 0x1F, 0x00, 0x3F, 0x80, 0x00, 0xFE, 0x38, 0xF7, 0xF3, 0xF9, 0xFF, 0x00, //  #####             #####          #######               #######   ###   #### #######  #######  #########
    0x7C, 0x00, 0x1F, 0x00, 0x7F, 0x00, 0x00, 0xFF, 0x38, 0xF3, 0xC3, 0xE3, 0xEF, 0x80, //  #####             #####         #######                ########  ###   ####  ####    #####   ##### #####
    0x7C, 0x00, 0x1F, 0x00, 0xFF, 0xFF, 0x00, 0xF7, 0x38, 0xF3, 0xC3, 0xC3, 0xC7, 0x80, //  #####             #####        ################        #### ###  ###   ####  ####    ####    ####   ####
    0x7C, 0x00, 0x1F, 0x01, 0xFF, 0xFF, 0x00, 0xF3, 0xB8, 0xF3, 0xC3, 0xC3, 0xC7, 0x80, //  #####             #####       #################        ####  ### ###   ####  ####    ####    ####   ####
    0x7C, 0x00, 0x1F, 0x01, 0xFF, 0xFF, 0x00, 0xF1, 0xF8, 0xF3, 0xC3, 0xC3, 0xC7, 0x80, //  #####             #####       #################        ####   ######   ####  ####    ####    ####   ####
    0x7C, 0x00, 0x1F, 0x01, 0xFF, 0xFE, 0x00, 0xF0, 0xF8, 0xF3, 0xC3, 0xC3, 0xEF, 0x80, //  #####             #####       ################         ####    #####   ####  ####    ####    ##### #####
    0x7C, 0x00, 0x1F, 0x00, 0xFF, 0xFC, 0x00, 0xF0, 0xF8, 0xF3, 0xF3, 0xC1, 0xFF, 0x00, //  #####             #####        ##############          ####    #####   ####  ######  ####     #########
    0x7C, 0x00, 0x1F, 0x00, 0x01, 0xFC, 0x00, 0xF0, 0x78, 0xF3, 0xF3, 0xC0, 0xFE, 0x00, //  #####             #####               #######          ####     ####   ####  ######  ####      #######
    0x7C, 0x00, 0x1F, 0x00, 0x03, 0xF8, 0x00, 0xF0, 0x38, 0xF1, 0xF3, 0xC0, 0x7C, 0x00, //  #####             #####              #######           ####      ###   ####   #####  ####       #####
    0x7C, 0x00, 0x1F, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####              ######
    0x7C, 0x00, 0x1F, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####             ######
    0x7C, 0x00, 0x1F, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####            ######
    0x7C, 0x00, 0x1F, 0x00, 0x1F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####           #######
    0x7C, 0x00, 0x1F, 0x02, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####      #   #######
    0x7C, 0x00, 0x1F, 0x03, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####      ## #######
    0x7C, 0x00, 0x1F, 0x03, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####      #########
    0x7C, 0x00, 0x1F, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####      ########
    0x7C, 0x00, 0x1F, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####      ########
    0x7C, 0x00, 0x1F, 0x03, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####             #####      #######
    0x7F, 0xFF, 0xFF, 0x03, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################      #######
    0x7F, 0xFF, 0xFF, 0x03, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################      ########
    0x7F, 0xFF, 0xFF, 0x03, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################      #########
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################
    0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #######################
};

const static unsigned char __attribute__((progmem)) ResetTimerLogo[] = {
    0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                                ###
    0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x3F, 0xFC, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                               #####                       ############  ####
    0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x3F, 0xFC, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                               ######                      ############  ####
    0x00, 0x00, 0x00, 0x03, 0xF8, 0x00, 0x00, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                               #######                     ############
    0x00, 0x00, 0x00, 0x01, 0xFC, 0x00, 0x00, 0x03, 0xC0, 0xF3, 0xDE, 0x70, 0x0F, 0x83, 0xDC, 0x00, //                                #######                        ####      ####  #### ####  ###        #####     #### ###
    0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x03, 0xC0, 0xF3, 0xFF, 0xF8, 0x3F, 0xE3, 0xFC, 0x00, //                                 #######                       ####      ####  ###############     #########   ########
    0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x03, 0xC0, 0xF3, 0xFF, 0xFC, 0x3F, 0xE3, 0xF8, 0x00, //                                  #######                      ####      ####  ################    #########   #######
    0x00, 0x00, 0x00, 0x00, 0x3F, 0x80, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x78, 0xF3, 0xE0, 0x00, //                                   #######                     ####      ####  ####  ####  ####   ####   ####  #####
    0x00, 0x00, 0x7F, 0x80, 0x1F, 0xC0, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x7F, 0xF3, 0xC0, 0x00, //                  ########          #######                    ####      ####  ####  ####  ####   ###########  ####
    0x00, 0x03, 0xFF, 0xF0, 0x1F, 0xE0, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x7F, 0xF3, 0xC0, 0x00, //               ##############       ########                   ####      ####  ####  ####  ####   ###########  ####
    0x00, 0x0F, 0xFF, 0xFC, 0x3F, 0xF0, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x78, 0x03, 0xC0, 0x00, //             ##################    ##########                  ####      ####  ####  ####  ####   ####         ####
    0x00, 0x3F, 0xFF, 0xFF, 0x7F, 0xF8, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x7C, 0xF3, 0xC0, 0x00, //           ###################### ############                 ####      ####  ####  ####  ####   #####  ####  ####
    0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x3F, 0xE3, 0xC0, 0x00, //          #####################################                ####      ####  ####  ####  ####    #########   ####
    0x00, 0xFF, 0x84, 0x7F, 0xFC, 0xFE, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x3F, 0xE3, 0xC0, 0x00, //         #########    #   #############  #######               ####      ####  ####  ####  ####    #########   ####
    0x03, 0xFE, 0x04, 0x1F, 0xF8, 0x7F, 0x00, 0x03, 0xC0, 0xF3, 0xCF, 0x3C, 0x0F, 0x83, 0xC0, 0x00, //       #########      #     ##########    #######              ####      ####  ####  ####  ####      #####     ####
    0x03, 0xF8, 0x04, 0x07, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //       #######        #       ########     ######
    0x07, 0xF0, 0x00, 0x01, 0xF8, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //      #######                   ######      #####
    0x0F, 0xC0, 0x00, 0x00, 0xFC, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     ######                      ######      ###
    0x1F, 0xC0, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    #######                       ######
    0x1F, 0xA0, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    ###### #                     #######
    0x3F, 0x10, 0x00, 0x01, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   ######   #                   #  ######
    0x3E, 0x08, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   #####     #                      #####
    0x7E, 0x04, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  ######      #                     ######
    0x7C, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####                              #####
    0x7C, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####                              #####
    0x7C, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####                              #####
    0xFC, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ######                               #####
    0xF8, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####                                #####
    0xF8, 0x00, 0x3C, 0x00, 0x27, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####             ####            #  #####
    0xF8, 0x00, 0x3C, 0x00, 0x37, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####             ####            ## #####
    0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ########          #########################
    0xF8, 0x00, 0x3C, 0x00, 0x37, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####             ####            ## #####
    0xF8, 0x00, 0x3C, 0x00, 0x27, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####             ####            #  #####
    0xF8, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####                                #####
    0xF8, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // #####                                #####
    0x7C, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####                              #####
    0x7C, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####                              #####
    0x7C, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  #####                              #####
    0x7E, 0x00, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  ######                            ######
    0x3E, 0x00, 0x00, 0x02, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   #####                       #    #####
    0x3F, 0x08, 0x00, 0x01, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   ######    #                  #  ######
    0x1F, 0x90, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    ######  #                    #######
    0x1F, 0xA0, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    ###### #                      ######
    0x0F, 0xC0, 0x00, 0x00, 0xFC, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     ######                      ######            ##
    0x07, 0xF0, 0x00, 0x01, 0xF8, 0x00, 0x38, 0x00, 0x01, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, //      #######                   ######             ###                  #############
    0x07, 0xF8, 0x04, 0x07, 0xF8, 0x00, 0x3E, 0x00, 0x01, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, //      ########        #       ########             #####                #############
    0x03, 0xFE, 0x04, 0x1F, 0xF0, 0x00, 0x3F, 0x00, 0x01, 0xFF, 0xF0, 0x00, 0xE0, 0x07, 0x03, 0x80, //       #########      #     #########              ######               #############            ###          ###      ###
    0x01, 0xFF, 0x84, 0x7F, 0xE0, 0x00, 0x3F, 0xC0, 0x01, 0xFF, 0xF0, 0x01, 0xF0, 0x0F, 0x87, 0xC0, //        ##########    #   ##########               ########             #############           #####        #####    #####
    0x00, 0x7F, 0xFF, 0xFF, 0xC0, 0x00, 0x3F, 0xE0, 0x01, 0xFF, 0xF0, 0x03, 0xB8, 0x1D, 0xCE, 0xE0, //          #########################                #########            #############          ### ###      ### ###  ### ###
    0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xF0, 0x01, 0xFF, 0xF0, 0x03, 0xBB, 0x9D, 0xCE, 0xE0, //           ######################                  ##########           #############          ### ### ###  ### ###  ### ###
    0x00, 0x0F, 0xFF, 0xFE, 0x00, 0x00, 0x3F, 0xF8, 0x01, 0xFF, 0xF0, 0x03, 0xBB, 0x9D, 0xCE, 0xE0, //             ###################                   ###########          #############          ### ### ###  ### ###  ### ###
    0x00, 0x03, 0xFF, 0xF8, 0x00, 0x00, 0x3F, 0xFC, 0x01, 0xFF, 0xF0, 0x03, 0xBB, 0x9D, 0xCE, 0xE0, //               ###############                     ############         #############          ### ### ###  ### ###  ### ###
    0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0x3F, 0xFC, 0x01, 0xFF, 0xF0, 0x03, 0xB8, 0x1D, 0xCE, 0xE0, //                 ##########                        ############         #############          ### ###      ### ###  ### ###
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0x01, 0xFF, 0xF0, 0x03, 0xBB, 0x9D, 0xCE, 0xE0, //                                                   ###########          #############          ### ### ###  ### ###  ### ###
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x01, 0xFF, 0xF0, 0x01, 0xF3, 0x8F, 0x87, 0xC0, //                                                   ##########           #############           #####  ###   #####    #####
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xE0, 0x01, 0xFF, 0xF0, 0x01, 0xF3, 0x8F, 0x87, 0xC0, //                                                   #########            #############           #####  ###   #####    #####
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x80, 0x01, 0xFF, 0xF0, 0x00, 0xE0, 0x07, 0x03, 0x80, //                                                   #######              #############            ###          ###      ###
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x01, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, //                                                   ######               #############
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                                                   #####
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                                                   ###

};

const static unsigned char __attribute__((progmem)) OverCurrentLogo[] = {
    0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                  #################################
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                 ###################################
    0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0xF8, 0x63, 0x3E, 0x7C, 0x00, 0x00, //                #####################################                            #####    ##   ##  #####  #####
    0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x01, 0xFC, 0x63, 0x3E, 0x7E, 0x00, 0x00, //                #####################################                           #######   ##   ##  #####  ######
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x03, 0x8E, 0x63, 0x30, 0x66, 0x00, 0x00, //               #######################################                         ###   ###  ##   ##  ##     ##  ##
    0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x03, 0x06, 0x36, 0x3E, 0x66, 0x00, 0x00, //               #####                             #####                         ##     ##   ## ##   #####  ##  ##
    0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x03, 0x06, 0x36, 0x3E, 0x7E, 0x00, 0x00, //              ######                             ######                        ##     ##   ## ##   #####  ######
    0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x03, 0x06, 0x36, 0x30, 0x7C, 0x00, 0x00, //              #####                               ######                       ##     ##   ## ##   ##     #####
    0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x03, 0x8E, 0x1C, 0x30, 0x6C, 0x00, 0x00, //             ######                               ######                       ###   ###    ###    ##     ## ##
    0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x01, 0xFC, 0x1C, 0x3E, 0x66, 0x00, 0x00, //            ######                                 ######                       #######     ###    #####  ##  ##
    0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xF8, 0x1C, 0x3E, 0x66, 0x00, 0x00, //            ######                                 ######                        #####      ###    #####  ##  ##
    0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //           ######                                   ######
    0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //           ######                                    #####
    0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //          ######                                     ######
    0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x03, 0xC3, 0x33, 0xE3, 0xE3, 0xE6, 0x1B, 0xF0, //          #####                                       #####            ####    ##  ##  #####   #####   #####  ##    ## ######
    0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x07, 0xF3, 0x33, 0xF3, 0xF3, 0xE7, 0x1B, 0xF0, //         ######                                       ######          #######  ##  ##  ######  ######  #####  ###   ## ######
    0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x0E, 0x23, 0x33, 0x33, 0x33, 0x07, 0x98, 0xC0, //         #####                                         #####         ###   #   ##  ##  ##  ##  ##  ##  ##     ####  ##   ##
    0x01, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x0C, 0x03, 0x33, 0x33, 0x33, 0xE7, 0x98, 0xC0, //        ######                                         ######        ##        ##  ##  ##  ##  ##  ##  #####  ####  ##   ##
    0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x0C, 0x03, 0x33, 0xF3, 0xF3, 0xE6, 0xD8, 0xC0, //        #####                                           #####        ##        ##  ##  ######  ######  #####  ## ## ##   ##
    0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x0C, 0x03, 0x33, 0xE3, 0xE3, 0x06, 0x78, 0xC0, //       ######                                           ######       ##        ##  ##  #####   #####   ##     ##  ####   ##
    0x03, 0xE0, 0xF0, 0xFF, 0x87, 0x03, 0xF0, 0xF8, 0x0E, 0x23, 0x33, 0x63, 0x63, 0x06, 0x78, 0xC0, //       #####     ####    #########    ###      ######    #####       ###   #   ##  ##  ## ##   ## ##   ##     ##  ####   ##
    0x07, 0xE0, 0xF8, 0xFF, 0x9F, 0x83, 0xF8, 0xFC, 0x07, 0xF3, 0xF3, 0x33, 0x33, 0xE6, 0x38, 0xC0, //      ######     #####   #########  ######     #######   ######       #######  ######  ##  ##  ##  ##  #####  ##   ###   ##
    0x07, 0xC3, 0xFC, 0xFF, 0xBF, 0xC3, 0xF8, 0x7E, 0x03, 0xC1, 0xE3, 0x33, 0x33, 0xE6, 0x18, 0xC0, //      #####    ########  ######### ########    #######    ######       ####     ####   ##  ##  ##  ##  #####  ##    ##   ##
    0x0F, 0xC3, 0x9E, 0x18, 0x39, 0xC3, 0x9C, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     ######    ###  ####    ##     ###  ###    ###  ###   ######
    0x1F, 0x83, 0x9E, 0x18, 0x79, 0xE3, 0x9C, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    ######     ###  ####    ##    ####  ####   ###  ###    ######
    0x1F, 0x83, 0x9E, 0x18, 0x79, 0xE3, 0x9C, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    ######     ###  ####    ##    ####  ####   ###  ###    ######
    0x3F, 0x03, 0x80, 0x18, 0x79, 0xE3, 0x9C, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   ######      ###          ##    ####  ####   ###  ###     ######
    0x3F, 0x03, 0x80, 0x18, 0x79, 0xE3, 0x9C, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   ######      ###          ##    ####  ####   ###  ###      #####
    0x7E, 0x01, 0x80, 0x18, 0x79, 0xE3, 0x9C, 0x0F, 0xC0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, //  ######        ##          ##    ####  ####   ###  ###      ######                           ###
    0x7C, 0x01, 0xF0, 0x18, 0x79, 0xE3, 0xFC, 0x07, 0xC0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //  #####         #####       ##    ####  ####   ########       #####                          #####
    0xFC, 0x00, 0xFC, 0x18, 0x79, 0xE3, 0xF8, 0x07, 0xE0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, // ######          ######     ##    ####  ####   #######        ######                         #####
    0xF8, 0x00, 0xFC, 0x18, 0x79, 0xE3, 0xF0, 0x03, 0xE0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, // #####           ######     ##    ####  ####   ######          #####                         #####
    0xFC, 0x00, 0x1E, 0x18, 0x79, 0xE3, 0x80, 0x07, 0xE0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, // ######             ####    ##    ####  ####   ###            ######                         #####
    0x7C, 0x00, 0x1E, 0x18, 0x79, 0xE3, 0x80, 0x07, 0xC0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //  #####             ####    ##    ####  ####   ###            #####                          #####
    0x7E, 0x00, 0x1E, 0x18, 0x79, 0xE3, 0x80, 0x0F, 0xC0, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //  ######            ####    ##    ####  ####   ###           ######                          #####
    0x3E, 0x03, 0x1E, 0x18, 0x79, 0xE3, 0x80, 0x0F, 0x80, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //   #####       ##   ####    ##    ####  ####   ###           #####                           #####
    0x3F, 0x03, 0x1E, 0x18, 0x39, 0xE3, 0x80, 0x1F, 0x80, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //   ######      ##   ####    ##     ###  ####   ###          ######                           #####
    0x1F, 0x83, 0x9E, 0x18, 0x3D, 0xC3, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //    ######     ###  ####    ##     #### ###    ###          #####                            #####
    0x1F, 0x81, 0xFC, 0x18, 0x3F, 0xC3, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //    ######      #######     ##     ########    ###         ######                            #####
    0x0F, 0xC0, 0xFC, 0x18, 0x1F, 0x83, 0x80, 0x7E, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //     ######      ######     ##      ######     ###        ######                             #####
    0x0F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //     ######                                               ######                             #####
    0x07, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //      ######                                             ######                              #####
    0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //       #####                                             ######                              #####
    0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //       ######                                           ######                               #####
    0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //        #####                                           #####                                #####
    0x01, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //        ######                                         ######                                #####
    0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //         #####                                         #####                                 #####
    0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //         ######                                       ######                                 #####
    0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //          #####                                       #####                                  #####
    0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //          ######                                     ######                                  #####
    0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, //           ######                                    #####                                    ###
    0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //           ######                                   ######
    0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //            ######                                 ######
    0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //            ######                                 ######
    0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, //             ######                               ######                                      ###
    0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //              #####                               ######                                     #####
    0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x00, 0x00, 0x00, //              ######                             ######                                     #######
    0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x00, 0x00, 0x00, //               #####                             #####                                      #######
    0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, //               #######################################                                       #####
    0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, //                #####################################                                         ###
    0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                #####################################
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                 ###################################
    0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                  #################################
};

const static unsigned char __attribute__((progmem)) ChangePolLogo[] = {
    0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xF9, 0x81, 0x83, 0x81, 0xC0, 0xC3, 0xF1, 0xFC, //                                  #                              #####  ##      ##     ###      ###      ##    ######   #######
    0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x03, 0xFD, 0x81, 0x83, 0x81, 0xE0, 0xCF, 0xF9, 0xFC, //                                  ###                          ######## ##      ##     ###      ####     ##  #########  #######
    0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x05, 0x81, 0x86, 0xC1, 0xE0, 0xCC, 0x09, 0x80, //                                  ####                         ##     # ##      ##    ## ##     ####     ##  ##      #  ##
    0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x06, 0x01, 0x81, 0x86, 0xC1, 0xB0, 0xD8, 0x01, 0x80, //                                  #####                       ##        ##      ##    ## ##     ## ##    ## ##          ##
    0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x06, 0x01, 0x81, 0x86, 0xC1, 0xB8, 0xD8, 0x01, 0x80, //                                  #######                     ##        ##      ##    ## ##     ## ###   ## ##          ##
    0x00, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x00, 0x06, 0x01, 0xFF, 0x8C, 0x61, 0x98, 0xD8, 0xF9, 0xF8, //                                  ########                    ##        ##########   ##   ##    ##  ##   ## ##   #####  ######
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x06, 0x01, 0xFF, 0x8C, 0x61, 0x8C, 0xD8, 0xF9, 0xF8, //                 ###########################                  ##        ##########   ##   ##    ##   ##  ## ##   #####  ######
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x06, 0x01, 0x81, 0x9F, 0xF1, 0x8E, 0xD8, 0x19, 0x80, //                 ############################                 ##        ##      ##  #########   ##   ### ## ##      ##  ##
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x06, 0x01, 0x81, 0x9F, 0xF1, 0x86, 0xD8, 0x19, 0x80, //                 ##############################               ##        ##      ##  #########   ##    ## ## ##      ##  ##
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x03, 0x05, 0x81, 0x98, 0x31, 0x83, 0xCC, 0x19, 0x80, //                 ###############################               ##     # ##      ##  ##     ##   ##     ####  ##     ##  ##
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x03, 0xFD, 0x81, 0xB0, 0x19, 0x83, 0xCF, 0xF9, 0xFC, //                 ################################              ######## ##      ## ##       ##  ##     ####  #########  #######
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0xF9, 0x81, 0xB0, 0x19, 0x81, 0xC3, 0xF1, 0xFC, //                 ##################################              #####  ##      ## ##       ##  ##      ###    ######   #######
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                 ###################################
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                 ##################################
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                 ################################
    0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0xF8, 0x00, 0x00, 0x3F, 0x83, 0x1F, 0xC0, 0x00, 0x00, //         #####   ###############################         #####                     #######     ##   #######
    0x03, 0xFE, 0x3F, 0xFF, 0xFF, 0xFC, 0x03, 0xFE, 0x00, 0x00, 0x3F, 0xE3, 0x1F, 0xE0, 0x00, 0x00, //       #########   ############################        #########                   #########   ##   ########
    0x0F, 0xFF, 0x9F, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0x80, 0x00, 0x30, 0x63, 0x18, 0x60, 0x00, 0x00, //     #############  #########################        #############                 ##     ##   ##   ##    ##
    0x1F, 0x07, 0xCF, 0xFF, 0xFF, 0xE0, 0x1F, 0x07, 0xC0, 0x00, 0x30, 0x33, 0x18, 0x60, 0x00, 0x00, //    #####     #####  #######################        #####     #####                ##      ##  ##   ##    ##
    0x3C, 0x01, 0xE0, 0x00, 0x7F, 0x80, 0x3C, 0x01, 0xE0, 0x00, 0x30, 0x33, 0x18, 0x60, 0x00, 0x00, //   ####         ####              ########         ####         ####               ##      ##  ##   ##    ##
    0x38, 0x00, 0x60, 0x00, 0x7F, 0x00, 0x38, 0x70, 0x60, 0x00, 0x30, 0x33, 0x1F, 0xC0, 0x00, 0x00, //   ###            ##              #######          ###    ###     ##               ##      ##  ##   #######
    0x70, 0x00, 0x70, 0x00, 0x7C, 0x00, 0x70, 0x70, 0x70, 0x00, 0x30, 0x33, 0x1F, 0x80, 0x00, 0x00, //  ###             ###             #####           ###     ###     ###              ##      ##  ##   ######
    0x70, 0x00, 0x70, 0x00, 0x78, 0x00, 0x70, 0x70, 0x70, 0x00, 0x30, 0x33, 0x18, 0xC0, 0x00, 0x00, //  ###             ###             ####            ###     ###     ###              ##      ##  ##   ##   ##
    0xE0, 0x00, 0x38, 0x00, 0x70, 0x00, 0xE0, 0x70, 0x38, 0x00, 0x30, 0x33, 0x18, 0x60, 0x00, 0x00, // ###               ###            ###            ###      ###      ###             ##      ##  ##   ##    ##
    0xE0, 0x00, 0x38, 0x00, 0x40, 0x00, 0xE0, 0x70, 0x38, 0x00, 0x30, 0x63, 0x18, 0x60, 0x00, 0x00, // ###               ###            #              ###      ###      ###             ##     ##   ##   ##    ##
    0xEF, 0xFF, 0xB8, 0x00, 0x00, 0x00, 0xEF, 0xFF, 0xB8, 0x00, 0x3F, 0xE3, 0x18, 0x30, 0x00, 0x00, // ### ############# ###                           ### ############# ###             #########   ##   ##     ##
    0xEF, 0xFF, 0xB8, 0x00, 0x40, 0x00, 0xEF, 0xFF, 0xB8, 0x00, 0x3F, 0x83, 0x18, 0x30, 0x00, 0x00, // ### ############# ###            #              ### ############# ###             #######     ##   ##     ##
    0xEF, 0xFF, 0xB8, 0x01, 0xC0, 0x00, 0xEF, 0xFF, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ### ############# ###          ###              ### ############# ###
    0xE0, 0x00, 0x38, 0x03, 0xC0, 0x00, 0xE0, 0x70, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ###               ###         ####              ###      ###      ###
    0x70, 0x00, 0x70, 0x07, 0xC0, 0x00, 0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  ###             ###         #####               ###     ###     ###
    0x70, 0x00, 0x70, 0x1F, 0xC0, 0x00, 0x70, 0x70, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  ###             ###       #######               ###     ###     ###
    0x30, 0x00, 0x60, 0x3F, 0xC0, 0x00, 0x30, 0x70, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   ##             ##       ########                ##     ###     ##
    0x38, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x38, 0x70, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   ###           ###     ########################  ###    ###    ###
    0x1F, 0x07, 0xC1, 0xFF, 0xFF, 0xFF, 0x9F, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //    #####     #####     ##########################  #####     #####
    0x0F, 0xFF, 0x87, 0xFF, 0xFF, 0xFF, 0x8F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //     #############    ############################   #############
    0x07, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xC7, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //      ###########    ##############################   ###########
    0x01, 0xFC, 0x3F, 0xFF, 0xFF, 0xFF, 0xE1, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //        #######    #################################    #######
    0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                  ##################################
    0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                 ###################################
    0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                  ##################################
    0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                    ################################
    0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                     ##############################
    0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                      ############################
    0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                        ##########################
    0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                         ########################
    0x00, 0x00, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                           ########
    0x00, 0x00, 0x00, 0x1F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                            #######
    0x00, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                              #####
    0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                               ####
    0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                                ###
    0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //                                  #
};

PROGMEM const byte TPICLookupTable[LenDCDCLookupTable] = {
    200, 168, 104, 24, 152, 216, 56, 120, 4, 132, 196, 164, 228, 148, 84, 52, 180, 244, 140, 204, 172, 236, 28, 92, 220, 188, 124, 2,
    130, 194, 34, 98, 226, 146, 82, 50, 178, 242, 10, 74, 202, 170, 106, 26, 154, 218, 58, 122, 250, 134, 70, 38, 166, 230, 22, 86, 214,
    182, 118, 14, 142, 206, 46, 110, 238, 158, 94, 62, 190, 254, 129, 65, 193, 161, 97, 17, 145, 209, 49, 113, 241, 137, 73, 41, 169,
    233, 25, 89, 217, 185, 121, 5, 133, 197, 37, 101, 229, 149, 85, 53, 181, 245, 13, 77, 205, 173, 109, 29, 157, 221, 61, 125, 253,
    131, 67, 35, 163, 227, 19, 147, 83, 211, 51, 179, 115, 243, 139, 75, 203, 43, 171, 107, 235, 155, 91, 219, 123, 251, 7, 135, 199}; //OVERDRIVE version (+0.4v)

PROGMEM const byte DisplayValues[LenDCDCLookupTable] = {
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81,
    82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133,
    134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157,
    158, 159, 160};

PROGMEM const byte NitroLookupTableLow[LenNITROLookupTable] = {1, 3, 11, 21, 36, 51, 63, 71, 74, 75, 76, 76, 76, 75, 74, 73, 73, 72, 71, 71};  // Vout= 2-(nitro_encoder*0.1) These are encoderPos location along the TPICLookupTable (max volt=9.3)
PROGMEM const byte NitroLookupTableHigh[LenNITROLookupTable] = {1, 2, 11, 21, 36, 51, 68, 80, 93, 96, 98, 99, 99, 99, 96, 94, 90, 90, 90, 90}; // Vout= 2-(nitro_encoder*0.1) These are encoderPos location along the TPICLookupTable (max volt=9.3)

// ------------------------------------------ FUNCTIONS --------------------------------------
void Write_TPIC2810(byte address, byte data);
void ShowDisplayValue(int value);
void DisplayMessage(byte RunMode, byte WriteORdelete, char Message[10], byte TypeOfMessg, int DisplayValue);
void NitroStart(byte NGrade, int encoderPosition);
void ReadPushbutton(int PIN_ip, int *StatNow, int *StatPrev, unsigned long *debouncerTimer, int DEBOUNCING_TIME, int *PushB_Event, byte BeepEnable, unsigned long *LongPressTimer);
unsigned int Read_Analog(int IP_ANALOG);
void BuzzerClick(int Pitch, int Pulses);
void Standby_Handler(unsigned long *StandbyTimer);
void Mitigate_OVChazard(byte *OVCerrorsConsecutive);
void DisplayTimer(int Hours, int Minutes);
void DisplayMem(int MachinePosition);
void RunTimer(unsigned long val, int *hours, int *minutes);
void Handle_ConfigMenus(byte RunMode, int index, byte *NextRunMode, byte *NitroStartGrade, byte *ResetRunTimer, int *ChangePol, byte *RuntimerEnable);
void StartupFrontLEDs();
// ------------------------------------------ VARIABLES -----------------------------------------
byte MachinesMemory[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Holds the 3 working points for each of the 3 Machines (9 values in total)
int MachineOffset = MACHINE1_OFFSET;                          // Holds the actual value of the offset needed for indexing the MachinesMemory[]
int i;
byte TPICvalue;
unsigned int DisplayValue;

int angle = 0;
int val;
int encoderPos;            // INDEX for the TPICLookupTable ==>  Vout = (encoderPos+20)-1  <----{ x10 range, i.e  110 = 11V }
int EncoderChange;         // Rotary spin right/left --> EncoderChange = +1/-1;
int EncoderChangePrev = 1; // Rotary spin right/left --> EncoderChange = +1/-1;
boolean encoderA;
boolean encoderALast = LOW; // remembers the previous pin state
unsigned long Time;         // Accounts for the General Time of the application (milliseconds from start up)
int PedalStatus;            //
int PushbuttonNow = PUSHBUTTON_OFF;
int PedalNow = PEDAL_OFF;
int RotPushNow = PUSHBUTTON_OFF;

int PushbuttonAction = PUSHBUTTON_IDLE;
int PedalEvent = PUSHBUTTON_IDLE;
int RotPushEvent = PUSHBUTTON_IDLE;
int PushbuttonPrev = PUSHBUTTON_OFF;
int RotPushPrev = PUSHBUTTON_OFF;
int PedalPrev = PEDAL_OFF;
int debouncingStat = NO_DEBOUNCING;
unsigned long debouncerPushTime;
unsigned long debouncerPedalTime;
unsigned long debouncerRotPushTime;
unsigned long debouncerRotaryTime;
unsigned long LongPressPushBTime;
unsigned long LongPressRotPushBTime;
unsigned long LongPressPedalTime;
int MachineMemPos = 0;
int RotaryChangedFlag = FLAG_OFF;
unsigned long SaveMemoryTimer;
byte EEPROMaux; // used to write and read values from EEPROM memory and to be index
byte updateDisplayVoltsFLAG = FLAG_ON;
byte NitroStartGrade;
byte NitroStartGradePrev;
byte NitroSetupStat = NITRO_SETUP_OFF;
byte RunMode = RUNMODE_NORMAL;
int MenuSelection = 1;                 // To navigate thru the Menu
byte updateMenuDisplayFLAG = FLAG_OFF; // Indicates if the configuration menu display has to be updated
byte RotaryDebounced = true;
int IoutSense = 0;
int VoutSense = 0;
unsigned int VoutBuffer[LEN_VOUT_BUFFER] = {0, 0, 0, 0};

unsigned long OVCrecoveryTimer; // Timer to count the recovery time needed for the pedal to be off prior to turn on the voltage again
unsigned long StandbyGlobalTimer = 0;

int OVC_AlarmDetected = false;
unsigned long OVC_AlarmTimer;
int continuousMode = false;               // Pedal-free function Enabled when true
int OutLatchState = false;                // When in continuous mode OutLatchState controls the ON/OFF of the machine output
int BlankingPushB_action = false;         // After long press of pushbutton (enters into continuous mode). To avoid memory change also (fall edge) this variable signal the event
int PUSHB_Longpress_1st_Action = true;    // If true then it is 1st action. THis avoids continuous entering in the LONGPRESS action
int ROTPUSHB_Longpress_1st_Action = true; // If true then it is 1st action. THis avoids continuous entering in the LONGPRESS action
int NitroForContinuousMode;               // This varible enables the "RISE event" of the continuous mode and therefore facilitates the Nitro
unsigned long PartialRuntimer = 0;        // Runtimer counter that accounts for the current segment of time being added up to the total counter.
// Each time that it is around 1 minute accumulated, it is transferred to the TotalRuntimer and the PartialRuntimer is cleared.
unsigned long TotalRuntimer = 0; // Accounts for the total run time of this session
byte RuntimerEnable = true;      // Variable used for getting noticed for Runtimer to run.
int runningHours;                // Used by Runtimer
int runningMinutes;              // Used by Runtimer
byte ResetRunTimer = false;      // Indicates if the TotalRuntimer has to be reset
byte NextRunMode;                // Used on the Top configuration menu to learn about the next selected menu

int PolarityStatus;            // Holds the current polarity status. At the start up is initializated with the EEPROM_POLARITY_STATUS value
int ChangePol = false;         // Managed in the configuration Menu to indicate if a polarity change is needed
unsigned long OVCsenseTime;    // Counts the time that the overcurrent is lasting
int VoutTarget;                // Vout value as it should be when sampled on VOSEN.
byte showMem = NO_DISPLAY_MEM; // Show information of memory position on display
unsigned long Show_Mem_Timer;  // Accounts for the total run time of this session
byte toggleTimer = false;
unsigned long ShowLongPressTimer;           // Timer to control the longpress information shown on display (Milliseconds)
byte ShowLongpressInfo = NO_LONGPRESS_INFO; // ATTENTION: Global var! Determines if Longpress info has to be shown. Used on other display
// functions to enable or not the display info.
byte OVCerrorsConsecutive = 0; // Number of consecutive OVC errors. After reached a limit, it is requiered that pedal is released by user in order to continue

byte RxBuffer;                  // Number of bytes received at Serial
unsigned long TimedataInBuffer; // Timer for Serial reception in Test Mode
byte TestCommands;              // Flags commands in Test Mode
char Message[5];                // For displaying messages in Testmode
byte ReportFlags;               // Collection of flags that reports the status in TestMode to the host system
//---------------------------------------------------------------------------------------------------------------------------

void setup()
{
  digitalWrite(LED_FRONT, LOW);
  pinMode(LED_FRONT, OUTPUT);

  //Serial.begin(9600);
  //Serial.print("Initializing version: ");
  //Serial.println(VERSION);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  //Wire.begin();
  i2c_init();

  // ------ VAR INITIALIZATION ------

  RunMode = RUNMODE_NORMAL;

  digitalWrite(LED_FRONT, LOW);
  pinMode(LED_FRONT, OUTPUT);
  pinMode(ANALOG_PEDAL, INPUT);
  pinMode(PUSHBUTTON_IP, INPUT);
  pinMode(ROTPUSH_IP, INPUT);
  pinMode(ROTA, INPUT);
  pinMode(ROTB, INPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(PEDAL_IP, INPUT);
  pinMode(ISEN, INPUT);
  pinMode(VOSEN, INPUT);
  pinMode(DCDC_EN, OUTPUT);
  pinMode(CHG_POL, OUTPUT);
  pinMode(ENA_OUT, OUTPUT);
  pinMode(CHANNEL_SEL, OUTPUT);
  pinMode(OVC_ALARM, INPUT);

  //-----------------

  digitalWrite(ENA_OUT, LOW);     // O/P Disabled --> works for led yellow/green
  digitalWrite(CHANNEL_SEL, LOW); // Channel = Machine 1
  digitalWrite(LED_FRONT, LOW);
  digitalWrite(CHG_POL, POL_NORMAL);
  digitalWrite(BUZZ, LOW);
  digitalWrite(PEDAL_IP, HIGH); // Pullup
  //  digitalWrite(ROTA,    HIGH);  // Pullup
  //  digitalWrite(ROTB,    HIGH);  // Pullup

  display.clearDisplay();
  display.drawRect(29, 0, 71, 64, WHITE);
  display.drawBitmap(36, 7, MusotokuLogo, 57, 50, WHITE);
  display.display();
  StartupFrontLEDs();
  digitalWrite(LED_FRONT, HIGH);
  delay(1000);

  //------------- TEST MODE -------
  // Evaluate RX Data::::::::::::
  // Lee telegramas de datos de 4 bytes.
  // Después de recibir el 1er byte arranca un contador de TIME OUT. Si el
  // timer se cumple, se tiran los datos y se vuelve a la escucha.
  // Cuando se completa la recepción de los 4 bytes antes del final del TIME OUT
  // se procesa la cadena y se vuelve a la escucha.
  // No hay cabeceras ni de comienzo ni de fin de telegrama. El protocolo está
  // basado en que hay envíos de 2bytes consecutivos y grandes tiempos (comparativamente)
  // sin envíos entre telegrama y telegrama.

  /*
    RxBuffer = Serial.available();        // Data in Rx buffer?  ----> Enter TEST Mode?

    if(RxBuffer > 0)
    {
    TestCommands = Serial.read();     // Reads the password letter
    }

    if(TestCommands == 'm')             // 'm' = password letter, if other character is in buffer, it doesn't enter in Test Mode
    {
    TestCommands = 0;  // Resets the input

    //---- TEST MODE STARTS HERE ! -------
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(10,8);
    display.setTextColor(WHITE);
    display.print("TEST MODE");
    sprintf(Message, "v.%d", VERSION);
    display.setCursor(70,8);
    display.print(Message);

    display.setCursor(2,20);
    display.print("SERIAL#: ");
    EEPROMaux = EEPROM[ EEPROM_MNF_YEAR ];
    itoa(EEPROMaux, Message, 10);
    sprintf(Message, "%d", EEPROMaux);
    display.setCursor(56,20);
    display.print(Message);
    EEPROMaux = EEPROM[ EEPROM_MNF_MONTH ];
    itoa(EEPROMaux, Message, 10);
    sprintf(Message, "%d-", EEPROMaux);
    display.setCursor(75,20);
    display.print(Message);
    EEPROMaux = EEPROM[ EEPROM_SERIAL_NO ];
    itoa(EEPROMaux, Message, 10);
    sprintf(Message, "%d", EEPROMaux);
    display.setCursor(90,20);
    display.print(Message);

    display.display();

    TimedataInBuffer = 0;

    while(1) // Test mode is forever (power cycle needed again)
    {
      RxBuffer = Serial.available();        // Data in Rx buffer?

      //---------------------- ORPHAN CHECK ------------------
      if((RxBuffer > 0) && (RxBuffer < TLG_RX_BYTES))  // Stops waiting for chars when TLG is complete
      {
        Serial.println(RxBuffer);

        if(TimedataInBuffer != 0)           // There is orphan chars.
        {
          if(millis() - TimedataInBuffer > RX_CHAR_TIMEOUT)
          {
            while(Serial.available())         // FLUSH serial input buffer
            {
              Serial.read();
            }
            TimedataInBuffer = 0;
          }
        }
        else
        {
          TimedataInBuffer = millis();
        }
      }

      // ------- READ SERIAL TELEGRAM  -----------
      else if(RxBuffer >= TLG_RX_BYTES)       // Standard reception
      {
        Serial.println("TLG");
        TimedataInBuffer = 0;      // Timeout var is cleared

        TestCommands    = Serial.read(); // Read
        encoderPos      = Serial.read(); // Read
                          Serial.read(); // The last byte is none of our business (Testbench equipment's)


        Serial.println("----------");
        Serial.print("TestCommands= ");
        Serial.println(TestCommands);
        Serial.print("encoderPos= ");
        Serial.println(encoderPos);

      }

      // ------- COMMANDS TO ACTIONS -----------
      TPICvalue    = pgm_read_byte_near(TPICLookupTable + encoderPos);
      Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);
      if(TestCommands == 48)
      {
        digitalWrite(DCDC_EN, DCDC_DISABLED);
      }
      else
      {
        digitalWrite(DCDC_EN, DCDC_ENABLED);
      }

      if(TestCommands == 51)
      {
        for(int i=0; i<64; i++)
        {
          display.drawFastHLine(0, i, 128, WHITE);
          display.display();
        }
        delay(2222);
        display.clearDisplay();
        display.setCursor(10,8);
        display.print("TEST MODE");
        display.display();
      }


      // ------- WRITE TO SERIAL  -----------

      ReportFlags = 0;
      if(digitalRead(OVC_ALARM)== HIGH)
      {
        bitSet(ReportFlags, ALARM_FLAG);
      }

      if(digitalRead(PEDAL_IP)== LOW)
      {
        bitSet(ReportFlags, PEDAL_FLAG);
      }

      IoutSense = Read_Analog(ISEN);
      VoutSense = Read_Analog(VOSEN);
      Serial.print(VoutSense);
      Serial.print('\t');
      Serial.print(IoutSense);
      Serial.print('\t');
      Serial.print(ReportFlags);
      Serial.println();
      delay(100);

    }//while(1)
    }
  */

  //-------------------------------
  display.clearDisplay(); // clears the screen and buffer

  // text display CONFIG
  display.setTextSize(7); // size 4 is perfect for 2 lines. Size 7 is perfect for 1 line with 3 chars (no decimal point in char)
  display.setTextColor(WHITE);

  //------------ EEPROM RECORDED?? --------
  EEPROMaux = EEPROM[EEPROM_RECORD_STAT];
  if (EEPROMaux != EEPROM_RECORDED_DONE)

  {
    EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS0)] = 10; // RECORDS the 1st position of Machine1 memory with default value
    EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS1)] = 20; // RECORDS the 2nd position of Machine1 memory with default value
    EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS2)] = 40; // RECORDS the 3rd position of Machine1 memory with default value
    EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS3)] = 50; // RECORDS the 4rd position of Machine1 memory with default value

    EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS0)] = 11; // RECORDS the 1st position of Machine2 memory with default value
    EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS1)] = 21; // RECORDS the 2nd position of Machine2 memory with default value
    EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS2)] = 41; // RECORDS the 3rd position of Machine2 memory with default value
    EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS3)] = 51; // RECORDS the 4rd position of Machine2 memory with default value

    EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS0)] = 12; // RECORDS the 1st position of Machine3 memory with default value
    EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS1)] = 22; // RECORDS the 2nd position of Machine3 memory with default value
    EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS2)] = 42; // RECORDS the 3rd position of Machine3 memory with default value
    EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS3)] = 52; // RECORDS the 4rd position of Machine3 memory with default value

    EEPROM[EEPROM_NITRO_STATUS] = NITRO_CFG_NO;        // NITRO configuration
    EEPROM[EEPROM_POLARITY_STATUS] = POL_NORMAL;       // Polarity set to normal by default (relay in resting mode)
    EEPROM[EEPROM_RECORD_STAT] = EEPROM_RECORDED_DONE; // Now signature is set to indicate that EEPROM is recorded
  }

  //------- DUMP EEPROM VALUES INTO RAM ARRAY -------

  MachinesMemory[0] = EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS0)];
  MachinesMemory[1] = EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS1)];
  MachinesMemory[2] = EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS2)];
  MachinesMemory[3] = EEPROM[(MACHINE1_OFFSET + MACHINE_EEPROM_POS3)];

  MachinesMemory[4] = EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS0)];
  MachinesMemory[5] = EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS1)];
  MachinesMemory[6] = EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS2)];
  MachinesMemory[7] = EEPROM[(MACHINE2_OFFSET + MACHINE_EEPROM_POS3)];

  MachinesMemory[8] = EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS0)];
  MachinesMemory[9] = EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS1)];
  MachinesMemory[10] = EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS2)];
  MachinesMemory[11] = EEPROM[(MACHINE3_OFFSET + MACHINE_EEPROM_POS3)];

  NitroStartGrade = EEPROM[EEPROM_NITRO_STATUS];
  PolarityStatus = EEPROM[EEPROM_POLARITY_STATUS];

  //------- VAR INITIALIZATION ------------
  NitroStartGradePrev = NitroStartGrade;

  // Polarity configuration
  if (PolarityStatus == POL_NORMAL)
  {
    digitalWrite(CHG_POL, POL_NORMAL);
  }
  else
  {
    digitalWrite(CHG_POL, POL_REVERSE);
  }

  encoderPos = MachinesMemory[MachineOffset]; // Starts in the 1st memory position of the 1st machinef
  Time = millis();
  debouncerPushTime = Time;
  debouncerPedalTime = Time;
  PartialRuntimer = Time;

  StandbyGlobalTimer = Time; // Reset the standby timer

  //Serial.print("Initialization done");
}

void loop()
{
  Time = millis();

  if (RunMode == RUNMODE_NORMAL) // Only display if NOT in any configuration menu
  {
    if (RuntimerEnable == true) //PLAY
    {
      if ((Time - PartialRuntimer) > 10000)
      {
        TotalRuntimer = TotalRuntimer + (Time - PartialRuntimer);
        PartialRuntimer = Time; // Reset the partial count
        RunTimer(TotalRuntimer, &runningHours, &runningMinutes);
        DisplayTimer(runningHours, runningMinutes);
      }
    }
    else if ((Time - PartialRuntimer) > 1000) // RuntimerEnable == STOP -->> Blinks the timer
    {
      PartialRuntimer = Time; // Reset the partial count

      if (toggleTimer == true)
      {
        toggleTimer = false;
        DisplayTimer(runningHours, runningMinutes);
      }
      else
      {
        toggleTimer = true;
        display.fillRect(95, 57, (128 - 95 - 1), 7, BLACK); // Clears the print area before the text (x0,y0,w,h,color)
        display.display();
      }
    } // RuntimerEnable == STOP
  }   //RunMode == RUNMODE_NORMAL

  Standby_Handler(&StandbyGlobalTimer); // Checks if go to STANDBY

  //----- DCDC ADJUSTMENT TO ENCODER POSITION -----
  // This section of code does not execute while on MENU CONFIGURATION mode

  if (updateDisplayVoltsFLAG == FLAG_ON)
  {
    updateDisplayVoltsFLAG = FLAG_OFF;

    TPICvalue = pgm_read_byte_near(TPICLookupTable + encoderPos);
    Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);

    DisplayValue = pgm_read_byte_near(DisplayValues + encoderPos);
    VoutTarget = int(DisplayValue * DISP_TO_VTARGET_CONV);

    ShowDisplayValue(DisplayValue);
    if (showMem == DISPLAY_MEM)
    {
      showMem = CLEAR_DISPLAY_MEM;
      //DisplayMem(MachineMemPos);    // Dual model must not display the memory memory because there are only
                                      // two memories that are associated to each channel.
    }
    DisplayTimer(runningHours, runningMinutes);
  }

  //----------------- UPDATE MEMORY ---------------------------
  if (RotaryChangedFlag == FLAG_ON)
  {
    if ((Time - SaveMemoryTimer) >= SAVE_MEM_TIMER)
    {
      MachinesMemory[(MachineOffset + MachineMemPos)] = encoderPos;
      EEPROMaux = MachineMemPos + MACHINE1_OFFSET;
      EEPROM[EEPROMaux] = (byte)encoderPos;
      RotaryChangedFlag = FLAG_OFF;
      DisplayMessage(RunMode, WRITE_MESSG, "REC", INFO_MESSG, DisplayValue);
      delay(300);
      DisplayMessage(RunMode, DELETE_MESSG, "REC", INFO_MESSG, DisplayValue);
    }
  }

  //------------------ CLEAR THE MEMORY INFORMATION ON DISPLAY --------------
  if (showMem == CLEAR_DISPLAY_MEM)
  {
    if ((Time - Show_Mem_Timer) > MEM_DISPLAY_TIME)
    {
      showMem = NO_DISPLAY_MEM;
      display.fillRect(0, 51, 37, 13, BLACK);
      display.display();
    }
  }

  //---------------- SPECIAL INFO ON DISPLAY (LONGPRESS ACTIONS) ---------

  if (ShowLongpressInfo != NO_LONGPRESS_INFO)
  {
    updateDisplayVoltsFLAG = FLAG_OFF;
    display.clearDisplay(); // clears the screen and buffer
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.drawRoundRect(0, 0, 128, 64, 10, WHITE);

    if (ShowLongpressInfo == LONGPRESS_INFO_CONT)
    {
      display.setCursor(33, 8);
      if (MachineMemPos == 0)
      {
        //MEMORY = 0 --> MACHINE OUTPUT 1
        display.setCursor(40, 8);
        display.print("-1-");
      }
      else
      {
        //MEMORY = 1 --> MACHINE OUTPUT 2
        display.setCursor(40, 8);
        display.print("-2-");
      }
      display.setCursor(48, 35);
      display.print("ON");
    }
    else if (ShowLongpressInfo == LONGPRESS_INFO_NO_CONT)
    {
      if (MachineMemPos == 0)
      {
        display.setCursor(40, 8);
        display.print("-1-");
      }
      else
      {
        display.setCursor(40, 8);
        display.print("-2-");
      }
      display.setCursor(41, 35);
      display.print("OFF");
    }
    else if (ShowLongpressInfo == LONGPRESS_INFO_TIMER_ON)
    {
      display.setCursor(26, 5);
      display.print("START");
      //THE TIMER
      char arr_HHMM[8] = "01:44";
      sprintf(arr_HHMM, "%01d:%02d", runningHours, runningMinutes);
      display.setCursor(28, 35);
      display.print(arr_HHMM);
    }
    else if (ShowLongpressInfo == LONGPRESS_INFO_TIMER_OFF)
    {
      display.setCursor(28, 5);
      display.print("STOP");
      //THE TIMER
      char arr_HHMM[8] = "01:44";
      sprintf(arr_HHMM, "%01d:%02d", runningHours, runningMinutes);
      display.setTextSize(3);
      display.setTextColor(WHITE);
      display.setCursor(25, 35);
      display.print(arr_HHMM);
    }

    display.display();

    if ((Time - ShowLongPressTimer) > SHOW_LONGPRESS_TIME)
    {
      ShowLongpressInfo = NO_LONGPRESS_INFO;
      //Serial.println("timer");
      updateDisplayVoltsFLAG = FLAG_ON; // To refresh the display with the output value
    }
  }
  else //NO_LONGPRESS_INFO
  {
    ShowLongPressTimer = Time;
  }

  //---------------- PUSHBUTTON ACTION EXECUTION ----------------------
  if ((PushbuttonAction == PUSHBUTTON_FALL) && (ShowLongpressInfo == NO_LONGPRESS_INFO)) //No  pushbutton available while longpress info is shown
  {
    PushbuttonAction = PUSHBUTTON_IDLE;

    if (BlankingPushB_action == false) // To avoid change memory after entering in continuous mode
    {
      RotaryChangedFlag = FLAG_OFF; // RESET the rotary change action to avoid memory update after the button is pressed

      StandbyGlobalTimer = Time;  // Reset the standby timer
      if (OutLatchState == false) // Only change MEMORY SETUP if Output is OFF
      { 
        MachineMemPos = (MachineMemPos + 1) % NUM_MEMORY;
        if (MachineMemPos == 0)
        {
          //MEMORY = 0 --> MACHINE OUTPUT 1
          digitalWrite(CHANNEL_SEL, LOW);
        }
        else
        {
          //MEMORY = 1 --> MACHINE OUTPUT 2
          digitalWrite(CHANNEL_SEL, HIGH);
        }
        encoderPos = MachinesMemory[(MachineOffset + MachineMemPos)];
        updateDisplayVoltsFLAG = FLAG_ON;
        showMem = DISPLAY_MEM; // For displaying the memory info
        Show_Mem_Timer = Time; // Ready the timer for display the mem number
      }
      else
      {
        //do nothing
        //While the output is ON (OUTLATCHSTATE = TRUE) it won't
        //change the memory
      }
    }
    else
    {
      BlankingPushB_action = false;
    }
  }

  if (PushbuttonAction == PUSHBUTTON_LONGPRESS)
  {
    BlankingPushB_action = true; // This is to avoid change in memory value after the Longpress

    if (PUSHB_Longpress_1st_Action == true)
    {
      PUSHB_Longpress_1st_Action = false; // This avoids continuous entering in PushbuttonAction, which would result in toggling the continuous mode

      BuzzerClick(HIGH_PITCH, 55);
      delay(15);
      BuzzerClick(HIGH_PITCH, 55);

      if (continuousMode == true) // Continuous Mode with toggle function in order to completely avoid the pedal if needed
      {
        digitalWrite(ENA_OUT, LOW);
        continuousMode = false;
        OutLatchState = false;
        ShowLongpressInfo = LONGPRESS_INFO_NO_CONT;
      }
      else
      {
        digitalWrite(ENA_OUT, HIGH);
        continuousMode = true;
        OutLatchState = true; // output is latched by default to ON state
        ShowLongpressInfo = LONGPRESS_INFO_CONT;
        NitroForContinuousMode = true; // This is to enable the Nitro action during the "RISE event" of continuou mode
      }
    }
  }
  else
  {
    PUSHB_Longpress_1st_Action = true; // Reset the variable. Now is ready for the next cycle.
  }

  //--------------- ROTPUSH ACTION EXECUTION ------------------
  if (RotPushEvent == PUSHBUTTON_FALL) //Important while in longpress action FALL event should not be attended
  {
    RotPushEvent = PUSHBUTTON_IDLE;

    if (ShowLongpressInfo == NO_LONGPRESS_INFO)
    {
      StandbyGlobalTimer = Time; // Reset the standby timer

      if (RunMode == RUNMODE_NORMAL) // TOGGLE action. If current is Normal -->Goes to CONFIG
      {
        RunMode = RUNMODE_CONFIG_TOP;
        updateMenuDisplayFLAG = FLAG_ON;   // Enable the MENU display view
        updateDisplayVoltsFLAG = FLAG_OFF; // Disable the Normal display view
        MenuSelection = 1;                 // Forces the Config Menu to start indexing the 1st element
      }

      else if (RunMode == RUNMODE_CONFIG_TOP)
      {
        switch (NextRunMode)
        {
        case RUNMODE_CONFIG_NITRO: //NITRO SELECTION
          RunMode = RUNMODE_CONFIG_NITRO;
          updateMenuDisplayFLAG = FLAG_ON;   // Enable the MENU display view
          updateDisplayVoltsFLAG = FLAG_OFF; // Disable the Normal display view

          // Allocation of the default selection of Nitro
          if (NitroStartGrade == NITRO_CFG_NO) // Forces the Config Menu to start with the saved setup
          {
            MenuSelection = 1;
          }
          else
          {
            MenuSelection = 2;
          }
          break;

        case RUNMODE_MENU_TIMER: //TIMER SELECTION
          RunMode = RUNMODE_MENU_TIMER;
          updateMenuDisplayFLAG = FLAG_ON;   // Enable the MENU display view
          updateDisplayVoltsFLAG = FLAG_OFF; // Disable the Normal display view
          MenuSelection = 1;                 // Forces the Config Menu to start indexing the 1st selected element
          break;

        case RUNMODE_CHANGE_POL: //CHANGE POLARITY SELECTION
          RunMode = RUNMODE_CHANGE_POL;
          updateMenuDisplayFLAG = FLAG_ON;   // Enable the MENU display view
          updateDisplayVoltsFLAG = FLAG_OFF; // Disable the Normal display view
          MenuSelection = 1;                 // Forces the Config Menu to start indexing the 1st selected element
          break;

        case RUNMODE_NORMAL: //EXIT SELECTION
          RunMode = RUNMODE_NORMAL;
          updateMenuDisplayFLAG = FLAG_OFF; // Enable the MENU display view
          updateDisplayVoltsFLAG = FLAG_ON; // Disable the Normal display view
          break;
        }
      } //---------------END RUNMODE_TOP_CONFIG

      else if (RunMode == RUNMODE_CONFIG_NITRO)
      {
        RunMode = RUNMODE_NORMAL;
        updateDisplayVoltsFLAG = FLAG_ON; // For refreshing the Normal display view
        updateMenuDisplayFLAG = FLAG_OFF; // Disable the Menu display view
        EEPROM[EEPROM_NITRO_STATUS] = NitroStartGrade;
      }

      else if (RunMode == RUNMODE_MENU_TIMER)
      {
        RunMode = RUNMODE_NORMAL;
        if (ResetRunTimer == true)
        {
          ResetRunTimer = false;
          TotalRuntimer = 0;
          RunTimer(TotalRuntimer, &runningHours, &runningMinutes); //Now updates the timer display to show 00:00
          DisplayTimer(runningHours, runningMinutes);
        }
        updateDisplayVoltsFLAG = FLAG_ON; // For refreshing the Normal display view
        updateMenuDisplayFLAG = FLAG_OFF; // Disable the Menu display view
        EEPROM[EEPROM_NITRO_STATUS] = NitroStartGrade;
      }

      else if (RunMode == RUNMODE_CHANGE_POL)
      {
        RunMode = RUNMODE_NORMAL; // Next mode
        if (ChangePol == true)
        {
          ChangePol = false;
          digitalWrite(DCDC_EN, DCDC_DISABLED);
          delay(333);

          if (PolarityStatus == POL_NORMAL)
          {
            PolarityStatus = POL_REVERSE;
            digitalWrite(CHG_POL, POL_REVERSE);
          }
          else
          {
            PolarityStatus = POL_NORMAL;
            digitalWrite(CHG_POL, POL_NORMAL);
          }
        }
        updateDisplayVoltsFLAG = FLAG_ON; // For refreshing the Normal display view
        updateMenuDisplayFLAG = FLAG_OFF; // Disable the Menu display view
        EEPROM[EEPROM_POLARITY_STATUS] = PolarityStatus;
      }
      else
      {
        //Error
      }
    } //--ShowLongpressInfo == NO_LONGPRESS_INFO
  }   //-------------RotPushEvent == PUSHBUTTON_ON_EDGE

  if (RotPushEvent == PUSHBUTTON_LONGPRESS)
  {
    if (ROTPUSHB_Longpress_1st_Action == true)
    {
      ROTPUSHB_Longpress_1st_Action = false; // This avoids continuous entering in PushbuttonAction, which would result in toggling the continuous mode

      RunMode = RUNMODE_NORMAL;
      updateMenuDisplayFLAG = FLAG_OFF; // Enable the MENU display view
      updateDisplayVoltsFLAG = FLAG_ON; // Disable the Normal display view

      BuzzerClick(HIGH_PITCH, 55);
      delay(15);
      BuzzerClick(HIGH_PITCH, 55);

      if (RuntimerEnable == true) // Continuous Mode with toggle function in order to completely avoid the pedal if needed
      {
        RuntimerEnable = false;
        ShowLongpressInfo = LONGPRESS_INFO_TIMER_OFF;
      }
      else
      {
        RuntimerEnable = true;
        ShowLongpressInfo = LONGPRESS_INFO_TIMER_ON;
      }
    }
  }
  else
  {
    ROTPUSHB_Longpress_1st_Action = true; // Reset the variable. Now is ready for the next cycle.
  }

  //----------------------- READ ALL PUSHBUTTONS -------------------------
  if (RunMode == RUNMODE_NORMAL)
  {
    // Section executed ONLY on Normal Mode
    ReadPushbutton(PUSHBUTTON_IP, &PushbuttonNow, &PushbuttonPrev, &debouncerPushTime, DEBOUNCE_PUSHB_TIME, &PushbuttonAction, BEEP_IS_TRUE, &LongPressPushBTime);
  }

  ReadPushbutton(PEDAL_IP, &PedalNow, &PedalPrev, &debouncerPedalTime, DEBOUNCE_PEDAL_TIME, &PedalEvent, BEEP_IS_FALSE, &LongPressPedalTime);
  ReadPushbutton(ROTPUSH_IP, &RotPushNow, &RotPushPrev, &debouncerRotPushTime, DEBOUNCE_ROTPUSH_TIME, &RotPushEvent, BEEP_IS_TRUE, &LongPressRotPushBTime);

  // --------------------- HANDLE ROTARY ENCODER -----------------------------

  Time = millis();
  encoderA = digitalRead(ROTA);

  if ((encoderALast == HIGH) && (encoderA == LOW))
  {
    StandbyGlobalTimer = Time; // Reset the standby timer
    //--Detect Spin direction--
    if (digitalRead(ROTB) == LOW)
    {
      EncoderChange = -1;
    }
    else
    {
      EncoderChange = 1;
    }

    //--Debounding the spin--
    if (EncoderChange != EncoderChangePrev)
    {
      if ((Time - debouncerRotaryTime) >= DEBOUNCE_ROTARY_TIME)
      {
        RotaryDebounced = true; // Debounced --> Now change in spin direction is permited
      }
      else
      {
        RotaryDebounced = false;
      }
    }

    //--Performs Spin Action depending on Run Mode--
    if (RotaryDebounced == true)
    {
      EncoderChangePrev = EncoderChange; // EncoderChangePrev is ONLY updated once the spin direction change is consolidated
      debouncerRotaryTime = Time;        // Debouncing action counts from here!

      ShowLongpressInfo = NO_LONGPRESS_INFO;

      if (RunMode == RUNMODE_NORMAL)
      {
        BuzzerClick(LOW_PITCH, 2);
        updateDisplayVoltsFLAG = FLAG_ON;
        RotaryChangedFlag = FLAG_ON; // For memory recording action
        SaveMemoryTimer = Time;      // For memory recording action

        encoderPos = encoderPos + EncoderChange;
        encoderPos = constrain(encoderPos, 0, (LenDCDCLookupTable - 1));
      }
      else if (RunMode == RUNMODE_CONFIG_TOP) //---------------------- MENU MODE ---------------------
      {
        BuzzerClick(LOW_PITCH, 2);
        updateMenuDisplayFLAG = FLAG_ON;
        MenuSelection = MenuSelection + EncoderChange;
        MenuSelection = constrain(MenuSelection, 1, 3); //1.TIMER - 2.NITRO - 3.EXIT
      }
      else if (RunMode == RUNMODE_CONFIG_NITRO)
      {
        BuzzerClick(LOW_PITCH, 2);
        updateMenuDisplayFLAG = FLAG_ON;
        MenuSelection = MenuSelection + EncoderChange;
        MenuSelection = constrain(MenuSelection, 1, 2); //1.YES - 2.NO
      }

      else if (RunMode == RUNMODE_MENU_TIMER)
      {
        BuzzerClick(LOW_PITCH, 2);
        updateMenuDisplayFLAG = FLAG_ON;
        MenuSelection = MenuSelection + EncoderChange;
        MenuSelection = constrain(MenuSelection, 1, 3); //1.START - 2.STOP - 3.RESET
      }

      else if (RunMode == RUNMODE_CHANGE_POL)
      {
        BuzzerClick(LOW_PITCH, 2);
        updateMenuDisplayFLAG = FLAG_ON;
        MenuSelection = MenuSelection + EncoderChange;
        MenuSelection = constrain(MenuSelection, 1, 2); //1.YES - 2.NO
      }
      else
      {
        // Error
        RunMode = RUNMODE_NORMAL; // Correction action: set to the default status
      }
    } // RotaryDebounce = true
  }
  encoderALast = encoderA;

  // --------------------- HANDLE CONFIGURATION -------------------------------------

  if (RunMode != RUNMODE_NORMAL) // If No NORMAL, then we are in configuration
  {
    // Read the RotPush to detect LONGPRESS

    if (updateMenuDisplayFLAG == FLAG_ON)
    {
      updateMenuDisplayFLAG = FLAG_OFF;

      Handle_ConfigMenus(RunMode, MenuSelection, &NextRunMode, &NitroStartGrade, &ResetRunTimer, &ChangePol, &RuntimerEnable);
    }
  }

  //------------------- PEDAL ACTION MANAGEMENT ------------------------
  if (PedalEvent == PUSHBUTTON_ON_EDGE)
  {
    PedalEvent = PUSHBUTTON_IDLE;
    if (continuousMode == false) // Pedal works different during Continuous mode: when pressed --> toggles the latch output (OutLatchState)
    {
      NitroStart(NitroStartGrade, encoderPos);
      digitalWrite(DCDC_EN, DCDC_ENABLED);
      //updateDisplayVoltsFLAG = FLAG_ON;          // NOTE: ELIMINATED TO AVOID LOSS OF DISPLAY WHILE IN CONFIG MENU AND PEDAL IS PUSHED (On test...)
      // Second Note: this line was initially added on version 24.
    }
    else // (continuousMode==true)-->Pedal toggles latch output state
    {
      if (OutLatchState == true)
      {
        OutLatchState = false;
        digitalWrite(ENA_OUT, LOW);
      }
      else
      {
        OutLatchState = true;
        digitalWrite(ENA_OUT, HIGH);
        NitroForContinuousMode = true; // This is to enable the Nitro action during the continuous mode
      }
    }
  }

  //-----NITRO for continuous mode is treated separately
  if ((NitroForContinuousMode == true) && (OutLatchState == true))
  {
    NitroForContinuousMode = false;
    NitroStart(NitroStartGrade, encoderPos);
    digitalWrite(DCDC_EN, DCDC_ENABLED);
    updateDisplayVoltsFLAG = FLAG_ON; // To force renewing the display and the output value
  }
  //-----------End Continuous with Nitro for continuous mode------

  if ((PedalNow == PEDAL_ON) || (OutLatchState == true))
  {
    StandbyGlobalTimer = Time; // Reset the standby timer
    digitalWrite(DCDC_EN, DCDC_ENABLED);
  }
  else
  {
    digitalWrite(DCDC_EN, DCDC_DISABLED);
  }

  //----------- OVC ALARM MANAGEMENT --------------
  
  //--------------------------------------------------------------------------------------------------------------
  // 5 oct 2021 - OVC ALARM is disabled to allow working with more demanding machines. Also, OVC_ALARM has
  //              proved to be not very reliable way of detecting overcircuit since it relies on Schmidtt trigger
  //              levels that can change from batch to batch.
  //              OVC_AlarmDetected is set to FALSE permanently.
  //--------------------------------------------------------------------------------------------------------------

//  if (digitalRead(OVC_ALARM) == HIGH)
//  {
//    Time = millis();
//    if (OVC_AlarmDetected == true) //Previous OVC ALARM detected?
//    {
//      if ((Time - OVC_AlarmTimer) > OVC_ALARM_TIMER)
//      {
//        OVC_AlarmDetected = false;
//        //Serial.println("OVC ALARM");
//        OVCerrorsConsecutive++;
//        Mitigate_OVChazard(&OVCerrorsConsecutive);
//        PedalNow = PEDAL_OFF; // After mitigate_ovcHazard the pedal is OFF. It is updated to prevent the following
//        // over current test to trigger double
//
//        continuousMode = false; // To prevent re-entering continuously (same as PEDAL_OFF above)
//        NitroForContinuousMode = false;
//        OutLatchState = false;
//        digitalWrite(ENA_OUT, LOW);
//
//        updateDisplayVoltsFLAG = FLAG_ON; // To bring the normal display ON again
//      }
//    }
//    else // First edge of OVC ALARM detected
//    {
//      OVC_AlarmTimer = Time;
//      OVC_AlarmDetected = true;
//    }
//  }
//  else
//  {
    OVC_AlarmDetected = false;
//  }

  //----------- OUT SENSING --------------

  if ((PedalNow == PEDAL_ON) || (OutLatchState == true))
  {
    Time = millis();
    IoutSense = Read_Analog(ISEN);
    VoutSense = Read_Analog(VOSEN);
    //Serial.println(VoutSense);
  }

  //------UNDERVOLTAGE LIMIT----------
  if ((PedalNow == PEDAL_ON) || (OutLatchState == true))
  {
    if ((VoutTarget - VoutSense) >= UNDERVOLT_1V8)
    {
      OVCsenseTime = Time;

      boolean OVCerror = true;
      while ((OVCerror == true) && ((Time - OVCsenseTime) < OVC_UVOLT_MAX_TIME))
      {
        //Serial.print('.');
        VoutSense = Read_Analog(VOSEN);
        if ((VoutTarget - VoutSense) < UNDERVOLT_1V5)
        {
          OVCerror = false;
        }
        Time = millis();
      }
      if (OVCerror == true)
      {
        //Serial.println("UNDERVOLTAGE");
        OVCerrorsConsecutive++;
        Mitigate_OVChazard(&OVCerrorsConsecutive);
        PedalNow = PEDAL_OFF; // After mitigate_ovcHazard the pedal is OFF. It is updated to prevent the following
        // over current test to trigger double

        continuousMode = false; // To prevent re-entering continuously (same as PEDAL_OFF above)
        NitroForContinuousMode = false;
        OutLatchState = false;
        digitalWrite(ENA_OUT, LOW);

        updateDisplayVoltsFLAG = FLAG_ON; // To bring the normal display ON again
      }
    }
  }

  //-------- OVER CURRENT LIMIT --------
  if ((PedalNow == PEDAL_ON) || (OutLatchState == true))
  {
    if (IoutSense > OVC_SENSE_LIMIT_SUP)
    {
      OVCsenseTime = Time;

      boolean OVCerror = true;
      while ((OVCerror == true) && ((Time - OVCsenseTime) < OVC_SENSE_MAX_TIME))
      {
        //Serial.print('-');
        IoutSense = Read_Analog(ISEN);
        if (IoutSense < OVC_SENSE_LIMIT_INF)
        {
          OVCerror = false;
        }
        Time = millis();
      }
      if (OVCerror == true)
      {
        //Serial.println("OVC SENSE");
        OVCerrorsConsecutive++;
        Mitigate_OVChazard(&OVCerrorsConsecutive);
        updateDisplayVoltsFLAG = FLAG_ON; // To bring the normal display ON again
        PedalNow = PEDAL_OFF;             // After mitigate_ovcHazard the pedal is OFF. It is updated to prevent the following
        // over current test to trigger double
      }
    }
  }

} // End main

//--------------------------------------------------------------------------------------

//***************************************************
// ================  FUNCTIONS =====================*
//***************************************************

// ======================================  Write_TPIC2810 ==========================================================
//
//
//==================================================================================================================

void Write_TPIC2810(byte address, byte data)
{
  int result;
  int loops;
  int danger;

  i2c_stop();
  delay(1);
  i2c_stop();
  delay(1);

  result = 0;
  loops = 0;
  danger = 1;
  while ((result == 0) && (loops < 10))
  {
    result = 1;
    loops++;

    if (!i2c_start(192 | I2C_WRITE))
    {
      //Serial.print("I2Cerr_1");
      result = 0;
      danger = 0;
    }
    if (!i2c_write(68))
    {
      //Serial.print("I2Cerr_2");
      result = 0;
      danger = 0;
    }
    if (!i2c_write(data))
    {
      //Serial.println("I2Cerr_3");
      result = 0;
    }
    i2c_stop();
    delay(1);
  }

  if (danger == 0)
  {
    //Serial.println("Retry I2C");

    result = 0;
    loops = 0;
    while ((result == 0) && (loops < 10))
    {
      result = 1;
      danger = 1;
      loops++;

      if (!i2c_start(192 | I2C_WRITE))
      {
        //Serial.print("I2Cerr_1+");
        result = 0;
        danger = 0;
      }
      if (!i2c_write(68))
      {
        //Serial.print("I2Cerr_2+");
        result = 0;
        danger = 0;
      }
      if (!i2c_write(data))
      {
        //Serial.println("I2Cerr_3+");
        result = 0;
      }
      i2c_stop();
      delay(1);
    }
  }
  else
  {
    //Serial.println("------");
  }
}
//void Write_TPIC2810(byte address, byte data)
//{
//  Wire.beginTransmission(byte(96)); // transmit command to device TPIC2810
//  Wire.write(byte(68));             // Command to transfer next value to output register
//  Wire.write(byte(data));
//  Wire.endTransmission();     // stop transmitting
//}

void ShowDisplayValue(int value)
{
  int intPart;
  int fracPart;
  int cursor;

  // Config the text
  display.setTextSize(7);
  display.setTextColor(WHITE);

  display.clearDisplay(); // clears the screen and buffer
  intPart = value / 10;
  fracPart = value - 10 * intPart;

  if (value >= 100)
  {
    //fracPart = value - 100;
    display.fillCircle(83, 55, 4, WHITE); // Decimal point 2
    cursor = 89;
  }
  else
  {
    //fracPart = value - 10*intPart;
    display.fillCircle(41, 55, 4, WHITE); // Decimal point 1
    cursor = 53;
  }
  display.setCursor(0, 0);
  display.print(intPart);
  display.setCursor(cursor, 0);
  display.println(fracPart);
  display.display();
}

// ================  DisplayMessage ================================================================================
// WriteORdelete: If delete, then writes in BLACK
// Message: Message to be written or deleted
// TypeOfMessg: INFO, WARNING, ERROR --> Each of them with specific display attributes
// DisplayValue: Value of the voltage (1 to 140). If < 100, then decimal '.' is aligned to left.
//               This is used to align the message and not overlap with the '.' (to the left or to the right)
//==================================================================================================================

void DisplayMessage(byte RunMode, byte WriteORdelete, char Message[25], byte TypeOfMessg, int DisplayValue)
{
  if ((RunMode == RUNMODE_NORMAL) && (ShowLongpressInfo == NO_LONGPRESS_INFO)) // When in config mode, no none of this info should appear on the display
  {
    switch (TypeOfMessg)
    {
    case INFO_MESSG:
      display.setTextSize(1);
      if (DisplayValue < 100)
        display.setCursor(62, 57);
      else
        display.setCursor(2, 57);
      break;

    case WARNING_MESSG:
      display.setTextSize(1);
      if (DisplayValue < 100)
        display.setCursor(62, 57);
      else
        display.setCursor(2, 57);
      break;

    case VIP_MESSG:
      display.setTextSize(3);
      display.setCursor(2, 10);
      break;

    case NITRO_MESSG:
      display.setTextSize(1);
      display.setCursor(95, 2);
      //display.drawRect(91,0,33,11,WHITE);
      break;
    }

    // WRITE OR DELETE THE MESSAGE
    if (WriteORdelete == WRITE_MESSG)
    {
      display.setTextColor(WHITE);
      display.print(Message);
      display.display();
    }
    else if (WriteORdelete == DELETE_MESSG)
    {
      display.setTextColor(BLACK);
      display.print(Message);
      display.display();
    }
    else
    {
      //error
    }
  } //RunMode != CONFIG_MODE
}

// ==============================================  NitroStart ======================================================
//
//
//==================================================================================================================

void NitroStart(byte NGrade, int encoderPosition)
{
  int NitroStepDuration = 10; // Milliseconds
  byte NitroIndex;
  byte TPICvalue;

  if ((encoderPosition < MAX_THRESHOLD_NITRO) && (NGrade != NITRO_CFG_NO)) // No NITRO for Vout > 11.5V or NITRO= OFF either
  {

    //-------Brings the programmed output to the lowest value to avoid first output value bounce-----------
    TPICvalue = pgm_read_byte_near(TPICLookupTable); // Lowest value of output corresponds to just the start of the TPICLookupTable
    Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);
    digitalWrite(DCDC_EN, DCDC_ENABLED);

    if (encoderPosition < HIGH_THRESHOLD_NITRO)
    {
      DisplayMessage(RunMode, WRITE_MESSG, "NITRO", NITRO_MESSG, DisplayValue);

      //------------ NITRO LOW profile  -----------------
      for (int n = 0; n < LenNITROLookupTable; n++)
      {
        NitroIndex = pgm_read_byte_near(NitroLookupTableLow + n);     // Recover the index in the TPICLookupTable
        TPICvalue = pgm_read_byte_near(TPICLookupTable + NitroIndex); // Get the value corresponding to such index
        Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);                     // Program the DCDC with that value
        delay(NitroStepDuration);
      }
    }
    else // NITRO_HIGH
    {
      //------------ NITRO HIGH profile  -----------------
      for (int n = 0; n < LenNITROLookupTable; n++)
      {
        NitroIndex = pgm_read_byte_near(NitroLookupTableHigh + n);    // Recover the index in the TPICLookupTable
        TPICvalue = pgm_read_byte_near(TPICLookupTable + NitroIndex); // Get the value corresponding to such index
        Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);                     // Program the DCDC with that value
        delay(NitroStepDuration);
      }
    }

    //------------ NITRO OUTPUT FALLING profile -----------------

    //NitroIndex is now pointing to the maximum value
    //of the Nitro profile. Let's begin the fall profile
    //from this point.

    while ((NitroIndex - encoderPosition) > 0)
    {
      TPICvalue = pgm_read_byte_near(TPICLookupTable + NitroIndex); // Get the value corresponding to such index
      Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);                     // Program the DCDC with that value

      delay(2);
      NitroIndex--;
    }

    //------------ DELETE THE NITRO TEXT ------------------------
    if (encoderPosition < HIGH_THRESHOLD_NITRO)
    {
      DisplayMessage(RunMode, DELETE_MESSG, "NITRO", NITRO_MESSG, DisplayValue);
    }
  }
}

//==========================================ReadPushbutton=======================================================
// PIN_IP: HW input
// StatNow:  Current cycle pushbutton status (ON/OFF)
// StatPrev:  Last cycle pushbutton status (ON/OFF)
// debouncerTimer: Debouncing timer
// PushB_Action: Event detected: RISE or IDLE
// BeepEnable: Indicates if it has to beep on push detection
//==================================================================================================================

void ReadPushbutton(int PIN_ip, int *StatNow, int *StatPrev, unsigned long *debouncerTimer, int DEBOUNCING_TIME, int *PushB_Event, byte BeepEnable, unsigned long *LongPressTimer)
{
  *StatNow = digitalRead(PIN_ip);
  if (*StatNow == PUSHBUTTON_ON)
  {
    if (*StatPrev == PUSHBUTTON_OFF)
    {
      *StatPrev = PUSHBUTTON_ON;

      if ((Time - *debouncerTimer) >= DEBOUNCING_TIME)
      {
        *PushB_Event = PUSHBUTTON_ON_EDGE;
        *LongPressTimer = Time;
        if (BeepEnable == true)
        {
          BuzzerClick(HIGH_PITCH, 77);
        }
      }
    }

    if (((Time - *LongPressTimer) >= LONG_PRESS_TIME)) // && (*PushB_Event != PUSHBUTTON_IDLE))
    {
      *PushB_Event = PUSHBUTTON_LONGPRESS;
      if (BeepEnable == true)
      {
        //            BuzzerClick(HIGH_PITCH, 55);
        //            delay(15);
        //            BuzzerClick(HIGH_PITCH, 55);
      }
    }
  }
  else //(PIN_IP = OFF)
  {
    if (*StatPrev == PUSHBUTTON_ON)
    {
      *StatPrev = PUSHBUTTON_OFF;
      *debouncerTimer = Time;

      if (*PushB_Event != PUSHBUTTON_IDLE)
      {
        *PushB_Event = PUSHBUTTON_FALL;
      }
    }
    else
    {
      *PushB_Event == PUSHBUTTON_IDLE;
    }
  }
}

//==========================================Handle_ConfigMenus======================================================
//
//
//==================================================================================================================

void Handle_ConfigMenus(byte RunMode, int index, byte *NextRunMode, byte *NitroStartGrade, byte *ResetRunTimer, int *ChangePol, byte *RuntimerEnable)
{

  display.clearDisplay(); // clears the screen and buffer

  if (RunMode == RUNMODE_CONFIG_TOP)
  {
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(35, 5);
    display.print("TIMER");
    display.setCursor(35, 26);
    display.print("NITRO");
    display.setCursor(35, 47);
    //display.print("FW/BW");
    display.print("EXIT");

    switch (index)
    {
    case 1:
      *NextRunMode = RUNMODE_MENU_TIMER;
      display.drawRect(29, 1, 70, 21, WHITE);
      break;

    case 2:
      *NextRunMode = RUNMODE_CONFIG_NITRO;
      display.drawRect(29, 22, 70, 21, WHITE);
      break;

    case 3:
      *NextRunMode = RUNMODE_NORMAL;
      display.drawRect(29, 43, 70, 21, WHITE);
      break;
    }
  } //---------------END CONFIG_TOP

  else if (RunMode == RUNMODE_CONFIG_NITRO)
  {
    display.clearDisplay(); // clears the screen and buffer

    // NITRO Logo
    display.drawBitmap(4, 4, NitroLogo, 105, 42, WHITE);

    // TEXT Settings
    display.setTextSize(2);
    display.setCursor(55, 45);

    switch (index)
    {
    case 1:
      *NitroStartGrade = NITRO_CFG_NO;

      display.fillRect(10, 20, 13, 19, BLACK);
      display.setTextColor(WHITE);
      display.print("NO");
      display.setTextColor(WHITE);
      display.print(" YES");
      display.drawRoundRect(50, 40, 32, 23, 5, WHITE);
      break;

    case 2:
      *NitroStartGrade = NITRO_CFG_YES;

      display.fillRect(10, 22, 13, 19, WHITE);
      display.setTextColor(WHITE);
      display.print("NO ");
      display.setTextColor(WHITE);
      display.print("YES");
      display.drawRoundRect(88, 40, 40, 23, 5, WHITE);
      display.setTextColor(BLACK);
      display.setCursor(11, 24);
      display.print("+");
      break;
    }
  } //-------------END CONFIG_NITRO

  else if (RunMode == RUNMODE_MENU_TIMER)
  {
    display.clearDisplay(); // clears the screen and buffer

    // RESET TIMER Logo
    display.drawBitmap(0, 0, ResetTimerLogo, 123, 60, WHITE);

    switch (index)
    {
    case 1: //START
      *ResetRunTimer = false;
      *RuntimerEnable = true;
      display.drawRoundRect(43, 40, 24, 23, 5, WHITE);
      break;

    case 2: //STOP
      *ResetRunTimer = false;
      *RuntimerEnable = false;
      display.drawRoundRect(65, 40, 25, 23, 5, WHITE);
      break;

    case 3: //RESET
      *ResetRunTimer = true;
      *RuntimerEnable = true;
      display.drawRoundRect(90, 40, 37, 23, 5, WHITE);
      break;
    }

    //THE TIMER
    char arr_HHMM[8] = "01:44";
    sprintf(arr_HHMM, "%01d:%02d", runningHours, runningMinutes);
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(53, 17);
    display.print(arr_HHMM);
  }

  // else if(RunMode == RUNMODE_CHANGE_POL)
  // {
  //  display.clearDisplay();   // clears the screen and buffer
  //
  //  // CHANGE POLARITY Logo
  //  display.drawBitmap(0, 0, ChangePolLogo, 126, 51, WHITE);
  //
  //  // TEXT Settings
  //  display.setTextSize(2);
  //  display.setCursor(55, 45);
  //
  //
  //  switch(index)
  //  {
  //    case 1:
  //    *ChangePol = false;
  //    display.setTextColor(WHITE);
  //    display.print("NO");
  //    display.setTextColor(WHITE);
  //    display.print(" YES");
  //    display.drawRoundRect(50,40,32,23,5,WHITE);
  //    break;
  //
  //    case 2:
  //    *ChangePol = true;
  //    display.setTextColor(WHITE);
  //    display.print("NO ");
  //    display.setTextColor(WHITE);
  //    display.print("YES");
  //    display.drawRoundRect(88,40,40,23,5,WHITE);
  //    break;
  //  }
  // }
  else
  {
    // The EXIT selection
  }

  display.display();
}

//===============================================Read_Analog=======================================================
// Reads the voltage at IP_ANALOG .
// TWO consecutive measurements are made and then averaged to avoid wrong samples.
// Sampling period of arduino is 100us, so total time is 200 us to sample and average the two readouts.
//
//===============================================================================================================

unsigned int Read_Analog(int IP_ANALOG)
{
  unsigned int AnalogValue;

  AnalogValue = analogRead(IP_ANALOG);
  AnalogValue = AnalogValue + analogRead(IP_ANALOG);
  AnalogValue = AnalogValue >> 1;
  return (AnalogValue);
}

//===============================================BuzzerClick=======================================================
// Generates a square waveform with period = Pitch
// Pitch: number of microseconds between HIGH and LOW
// Pulses: number of periods
//===============================================================================================================

void BuzzerClick(int Pitch, int Pulses)
{
  for (int m = 0; m < Pulses; m++)
  {
    digitalWrite(BUZZ, HIGH);
    delayMicroseconds(Pitch);
    digitalWrite(BUZZ, LOW);
    delayMicroseconds(Pitch);
  }
}

void Standby_Handler(unsigned long *StandbyTimer)
{
  byte encoderStatus;
  byte wakeup;
  byte inputsSTOP;

  if ((Time - *StandbyTimer) > STANDBY_TIME)
  {
    display.clearDisplay();
    display.display();
    DisplayMessage(RunMode, WRITE_MESSG, "BYE,BYE", VIP_MESSG, 0);
    delay(2000);
    display.clearDisplay(); // clears the screen and buffer
    display.display();

    //-----GET ROTARY INFO BEFORE SLEEP-----
    encoderStatus = digitalRead(ROTA);

    //-----SCAN FOR CHANGES to WAKE UP-----
    Time = millis();
    wakeup = false;
    while (wakeup == false)
    {
      if (digitalRead(ROTA) != encoderStatus)
      {
        wakeup = true;
      }
      if (digitalRead(PUSHBUTTON_IP) == PUSHBUTTON_ON)
      {
        wakeup = true;
      }

      if (digitalRead(PEDAL_IP) == PUSHBUTTON_ON)
      {
        wakeup = true;
      }

      if (digitalRead(ROTPUSH_IP) == PUSHBUTTON_ON)
      {
        wakeup = true;
      }

    } //While_standby

    //------WAKE UP ROUTINE------
    display.clearDisplay();
    display.display();
    DisplayMessage(RunMode, WRITE_MESSG, "HELLO  (again)", VIP_MESSG, 0);
    BuzzerClick(HIGH_PITCH, 100);
    delay(1000);
    display.clearDisplay();
    display.display();

    // Wait for the user to release the buttons
    inputsSTOP = false;
    while (inputsSTOP == false)
    {
      if (digitalRead(PUSHBUTTON_IP) == PUSHBUTTON_OFF)
      {
        if (digitalRead(PEDAL_IP) == PUSHBUTTON_OFF)
        {
          if (digitalRead(ROTPUSH_IP) == PUSHBUTTON_OFF)
          {
            inputsSTOP = true;
          }
          else
          {
            DisplayMessage(RunMode, WRITE_MESSG, "PLEASE, STOP BUTTONS", INFO_MESSG, 150);
          }
        }
        else
        {
          DisplayMessage(RunMode, WRITE_MESSG, "PLEASE, STOP BUTTONS", INFO_MESSG, 150);
        }
      }
      else
      {
        DisplayMessage(RunMode, WRITE_MESSG, "PLEASE, STOP BUTTONS", INFO_MESSG, 150);
      }
    }

    display.clearDisplay(); // clears the screen and buffer
    display.display();

    Time = millis();
    *StandbyTimer = Time;             // Reset the timer before coming back to active
    updateDisplayVoltsFLAG = FLAG_ON; // To bring the normal display ON again

  } //Standby_Mode

} //END Standby_Handler

//===============================================Mitigate_OVChazard=======================================================
// Display OVERCURRENT message
// Disable the DC/DC output
// Waits for the user to stop the pedal
// Finally, waits another 2 seconds more
//===============================================================================================================

void Mitigate_OVChazard(byte *OVCerrorsConsecutive)
{
  digitalWrite(DCDC_EN, DCDC_DISABLED); // Output is DISABLED

  // DISPLAY ERROR MESSAGE "OVERCURRENT!!) "STOP YOUR PEDAL NOW"
  // DisplayMessage(RunMode, WRITE_MESSG, "STOP PEDAL!",   INFO_MESSG, DisplayValue);
  display.clearDisplay(); // clears the screen and buffer
  display.drawBitmap(0, 0, OverCurrentLogo, 124, 63, WHITE);
  display.display();

  BuzzerClick(HIGH_PITCH, 100);
  delay(50);
  BuzzerClick(HIGH_PITCH, 100);
  delay(50);
  BuzzerClick(HIGH_PITCH, 100);

  delay(OVC_RECOVERY_TIME); // Waits at least some time with Pedal OFF

  // If a MAX_OVC_ERRORS number of consecutive errors --> Wait for Pedal==OFF
  if (*OVCerrorsConsecutive > MAX_OVC_ERRORS)
  {
    while ((digitalRead(PEDAL_IP) == PEDAL_ON) || (continuousMode == true))
    {
      *OVCerrorsConsecutive = 0;
      OutLatchState = false;  // Just in case continuousMode is ON, the output latch is stopped.
      continuousMode = false; // And the continuous mode is stopped
      digitalWrite(ENA_OUT, LOW);
    }
  }

  display.clearDisplay(); // clears the screen and buffer
  display.display();
}

//===============================================DisplayTimer=======================================================

//===============================================================================================================

void DisplayTimer(int Hours, int Minutes)
{
  if (ShowLongpressInfo == NO_LONGPRESS_INFO)
  {
    char arr_HHMM[8];

    sprintf(arr_HHMM, "%02d:%02d", Hours, Minutes);

    display.fillRect(95, 57, (128 - 95 - 1), 7, BLACK); // Clears the print area before the text (x0,y0,w,h,color)

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(95, 57);

    display.print(arr_HHMM);
    display.display();
  }
}

//===============================================DisplayMem=======================================================
// Shows the current memory status on screen
//
//===============================================================================================================
void DisplayMem(int MachinePosition)
{
  if (ShowLongpressInfo == NO_LONGPRESS_INFO)
  {
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.fillRect(0, 51, 37, 13, BLACK); // Clear the space before printing
    display.drawRect(0, 51, 34, 13, WHITE); // Rectangle for Memory indicator

    if (MachinePosition == 0)
    {
      display.setCursor(3, 54);
      display.drawPixel(12, 57, WHITE);
      display.drawPixel(19, 57, WHITE);
      display.drawPixel(26, 57, WHITE);
    }
    else if (MachinePosition == 1)
    {
      display.setCursor(11, 54);
      display.drawPixel(5, 57, WHITE);
      display.drawPixel(19, 57, WHITE);
      display.drawPixel(26, 57, WHITE);
    }
    else if (MachinePosition == 2)
    {
      display.setCursor(17, 54);
      display.drawPixel(5, 57, WHITE);
      display.drawPixel(12, 57, WHITE);
      display.drawPixel(26, 57, WHITE);
    }
    else
    {
      display.setCursor(25, 54);
      display.drawPixel(5, 57, WHITE);
      display.drawPixel(12, 57, WHITE);
      display.drawPixel(19, 57, WHITE);
    }
    display.print(MachinePosition + 1);

    display.display();
  }
}

//===============================================RunTimer=======================================================
//
//
//===============================================================================================================

void RunTimer(unsigned long val, int *hours, int *minutes)
{
  val = val / 1000; // Move Milliseconds into Seconds

  val = constrain(val, 0, SECS_9H); // Constrain the value to no more than 9:00 hours
  *hours = numberOfHours(val);
  *minutes = numberOfMinutes(val);
}

//===============================================RunTimer=======================================================
//
//
//===============================================================================================================

void StartupFrontLEDs()
{
  byte i, j, k, m, n;
  i = 1;
  j = 20;
  m = 0;
  n = 10;

  while (m < 19)
  {
    for (k = 0; k < n; k++)
    {
      digitalWrite(LED_FRONT, HIGH);
      delay(i);
      digitalWrite(LED_FRONT, LOW);
      delay(j);
    }
    i++;
    j--;
    m++;
    n--;
    n = constrain(n, 5, 10);
  }
}
