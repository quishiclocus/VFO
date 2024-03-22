/*  
Arduino Nano script for homebrew multiband CW transceivers. 
Borrowed from Paul Taylor, VK3HN (https://vk3hn.wordpress.com/) standing on the shoulders of:
  - Przemek Sadowski, SQ9NJE (basic controller script)
  - Jason Mildrum NT7S (si5351 library)
  - too many others to mention (ideas, code snippets). 
  
Heavily modified, so starting the versioning over.

V1.0, 22 Mar 2024 - Code modified for only one radio - based on Paul VK3HN's SP9.

[Please note: not all documented features are implemented currently.]
Labels that need to be #define'd for your target radio/rig/project:
  Rotary encoder         {ENCODER_OPTICAL_360, ENCODER_MECHANICAL} 
  Display technology     {DISPLAY_LCD, DISPLAY_OLED}
  Display type           {LCD_20X4, LCD_16X2, LCD_8X2, OLED_128X64}
  Project name           {SP_IV, SP_11, SP_V, SS_EI9GQ, SP_6, SP_7, etc }
  BFO enable             {BFO_ENABLED}
  VSWR meter             {VSWR_METER}
  CW keyer               {CW_KEYER} 
  Tune mS                {TUNE_MS}            Typical value: 3000mS
  Held button mS         {BUTTON_HELD_MS}     Typical value: 700mS
  Diagnostics on display {DIAGNOSTIC_DISPLAY} 
  VFO/BFO swap between transmit and receive {VFO_BFO_SWAP}
*/

// common libraries
#include <Rotary.h>
#include <si5351.h>     // Etherkit si3531 library from NT7S,  V2.1.4   https://github.com/etherkit/Si5351Arduino 
#include <PCF8574.h>    // pcf8574 library by Rob Tillaart 0.3.2
#include <Wire.h>
#include <EEPROM.h>

#define DISPLAY_OLED
#define OLED_128X64
#define BFO_ENABLED
#define CW_KEYER
#define ENCODER_OPTICAL
#define ENCODER_OPTICAL_360

#ifdef ENCODER_OPTICAL
  uint8_t vfo_enc_clk = 2;
  uint8_t vfo_enc_dir = 3;
  uint8_t up_step_counter = 0;
  uint8_t down_step_counter = 0;
  uint8_t enc_dir;
  bool invert_encoder_direction = false; // change direction of rotary encoder counting true/false 
  #define ENCODER_A          vfo_enc_clk  // Encoder pin B
  #define ENCODER_B          vfo_enc_dir  // Encoder pin A
  Rotary r = Rotary(ENCODER_A, ENCODER_B);
  volatile long temp, counter = 0;
  byte dial_speed=6;
#endif
// ------------------------------------------------------------------------------------------------------------------
// #define one (and only one) label to pull in the right code for the specific project

// [ Summit Prowlers ]
#define SP_9         // 5-band compact SSB/CW transceiver (5/2020) 

// ------------------------------------------------------------------------------------------------------------------

int updateSysStack = 0;

// // common #define's that precede other declarations
// #define LCD_RS    8  // Register Select is LCD pin 4 
// #define LCD_E     9  // Enable/clock LCD pin 6
// #define LCD_D4   10  // LCD D4 
// #define LCD_D5   11  // LCD D5 
// #define LCD_D6   12  // LCD D6 
// #define LCD_D7   13  // LCD D7  

#define BUTTON_HELD_MS 700  // button down period in mS required to activate 2nd pushbutton function
#define TUNE_MS       5000  // tune (key down) period (mS)
//#define TUNE_MS       30000  // tune (key down) period (mS) 30 for testing
#define LPF_DROPOUT_DELAY 8000  // period of inactivity before the LPF drops out to save battery (mS)
#define FAN_DROPOUT_DELAY  5000  // period of inactivity before the fan drops out to save battery (mS)
#define VFO_DRIVE_THRESHOLD 18000000  // threshold freq above which a higher si5351 clock output level on the VFO clock is used
#define CO_DRIVE_THRESHOLD  18000000  // threshold freq above which a higher si5351 clock output level on the carrier oscillator (CW) clock is used
//#define CO_DRIVE_THRESHOLD  18000  // threshold freq above which a higher si5351 clock output level on the carrier oscillator (CW) clock is used

#define RX_MUTE_DELAY_MS   10 // mS delay after muting receiver and before unmuting receiver 

#define BREAK_IN_DELAY   800  // break-in hang time (mS) (may be overridden in the project sections) 

// ------------------------------------------------------------------------------------------------------------------
// Arduino Nano digital pin assignments (aligns with Raduino)

//                         0    Serial
//                         1    Serial
#ifndef ENCODER_OPTICAL
  #define ENCODER_B          2  // Encoder pin B
  #define ENCODER_A          3  // Encoder pin A
  Rotary r = Rotary(ENCODER_A, ENCODER_B);
#endif
#define PTT_SENSE          4  // sense the PTT button being pressed (low == transmit)
#define RX_MUTE_LINE       5  // receiver mute 
#define RX_MUTE_OFF_VALUE  0   // default value for an un-muted receiver (low), can be inverted in a project block
#define RX_MUTE_ON_VALUE   1   // default value for a muted receiver (high), can be inverted in a project block
//                         6    keyer sidetone, declared below 
#define TRANSMIT_LINE      7  // controls the T/R relay (high == transmit)

// Arduino Nano analogue pins <<--- Fix to use Button Fever
#define SWITCH_BANK       A0 // front panel push buttons
#ifdef  CW_KEYER
  #define PIN_PADDLE        A1 // paddle on analog pin 1
  #define PIN_PUSHBTTN_REAR A2 // keyer memory pushbuttons on analog pin 2
  #define PIN_KEYER_SPEED   A3 // speed potentiometer wiper
#endif
#define PIN_S_METER       A7 // s-meter TBA (alternately, use A7) (was A3)
//                        A4    SDA
//                        A5    SCL
#define PIN_PWR_METER     A6 // analogue pin for relative RF sensing circuit
int pwr_val = 0;             // stores the last reading of the RF power sensing input (on pin PIN_PWR_METER)


// #ifdef VSWR_METER
// #define PIN_SWR_FWD       A6 // analogue pin for SWR bridge forward
// #define PIN_SWR_REV       A7 // analogue pin for SWR bridge reverse
// #endif




// specific declarations and #define's for each project -------------------------------------------------

// 'Summit Prowlers'  - - -

#ifdef SP_9                      // Compact 5-band SSB/CW transceiver

#define BFO_ENABLED
#define BFO_TUNE_LO     8998500ULL  // lowest BFO frequency
#define BFO_TUNE_HI     9001500ULL  // highest BFO frequency
volatile uint32_t USB = 9001500ULL;  // selected sideband  
volatile uint32_t LSB = 8998500ULL;
#define DISPLAY_OLED
#define OLED_128X64
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_OLED_ADDRESS 0x3C // correct!
#define RST_PIN -1     // Define proper RST_PIN if required
  SSD1306AsciiAvrI2c oled;
#define NBR_VFOS    5  // number of selectable VFOs 
#define CW_KEYER       // include the CW keyer code
#define CO_DC_SUPPLY 11   // this  pin controls a high side DC switch to enable the carrier oscillator when keyed
// #define DIAGNOSTIC_DISPLAY // FOR TESTING THOSE PESKY PUSHBUTTONS!!
#define RX_MUTE_DELAY_MS   20 // override the mS delay after muting receiver and before unmuting receiver 

#endif
// ------------------------------------------------------------------------------------------------------------------

#ifdef DIAGNOSTIC_DISPLAY
// flags and variables to trace values to the display, for temporary diagnostics
bool diagnostic_flag = false;  // trace on the display as a diagnostic
int  diagnostic_int = 0;       // trace an integer value
// String diagnostic_string("");  // trace a String
#endif

// frequency ranges for automatic band pass and low pass filter switching
#define FILTER_630_LB  200000ULL  // Filter set lower bound (MHz)
#define FILTER_630_UB  500000ULL  // Filter set upper bound (MHz)

#define FILTER_MW_LB   500100ULL  // Filter set lower bound (MHz)
#define FILTER_MW_UB  1750000ULL  // Filter set upper bound (MHz)

#define FILTER_160_LB 1750100ULL  // Filter set lower bound (MHz)
#define FILTER_160_UB 2500000ULL  // Filter set upper bound (MHz)

#define FILTER_80_LB  2501000ULL  // Filter set lower bound (MHz)
#define FILTER_80_UB  4000000ULL  // Filter set upper bound (MHz)

#define FILTER_60_LB  4001000ULL  // Filter set lower bound (MHz)
#define FILTER_60_UB  6500000ULL  // Filter set upper bound (MHz)

#define FILTER_40_LB  6501000ULL  // Filter set lower bound (MHz)
#define FILTER_40_UB  8000000ULL  // Filter set upper bound (MHz)

#define FILTER_30_LB  8001000ULL  // Filter set lower bound (MHz)
#define FILTER_30_UB 12000000ULL  // Filter set upper bound (MHz)

#define FILTER_20_LB 12001000ULL  // Filter set lower bound (MHz)
#define FILTER_20_UB 16000000ULL  // Filter set upper bound (MHz)

#define FILTER_17_LB 16001000ULL  // Filter set lower bound (MHz)
#define FILTER_17_UB 18500000ULL  // Filter set upper bound (MHz)

#define FILTER_15_LB 19000000ULL  // Filter set lower bound (MHz)
#define FILTER_15_UB 21500000ULL  // Filter set upper bound (MHz)

// ------------------------------------------------------------------------------------------------------------------

// i2c devices and addresses:
// si5351  x60
// BPF selector PCF8574 x20 .. x38
// LPF selector PCF8574 x40

#define I2C_BPF_DEMUX   0x20         // default I2C address of the BPF PCF8574 
#define I2C_LPF_DEMUX   0x24         // default I2C address of the LPF PCF8574 

#ifdef SP_9
#define I2C_BPF_DEMUX   0x20         // I2C address of the BPF PCF8574 in this rig
#endif

PCF8574 PCF_BPF(I2C_BPF_DEMUX);       
PCF8574 PCF_LPF(I2C_LPF_DEMUX);  

// ------------------------------------------------------------------------------------------------------------------

bool message_playing = false;      // true when the keyer is playing a CW message 
#define CW_TONE_HZ       600  // CW tone frequency (Hz) (originally 700)
unsigned int dot_dash_counter = 0;  // total nbr CW chars sent since power-up
unsigned int dot_dash_sent = 0;     // nbr CW chars sent this transmit period, used for timing a refresh to the (interleaved) RF relative power meter
#define PIN_TONE_OUT       6  // digital pin with keyed audio tone on it
bool key_down = false; 
unsigned long char_sent_ms, curr_ms;
bool space_inserted;
#define KEYER_MSG_SPEED 50

#ifdef CW_KEYER
// start of CW Keyer block -------------------------------------------------------------

#define PADDLE_R           1      // value representing analog value for paddle left (dot)
#define PADDLE_L           2      // value representing analog value for paddle right (dash)

#define CW_DASH_LEN        5  // length of dash (in dots)
#define SERIAL_LINE_WIDTH 80  // number of morse chars on Serial after which we newline 

// set the CW keyer speed and canned messages for each project
// for the keyer speed, lower is faster, 60 is 10 w.p.m.

// String morse_msg[] = {"CQ CQ VK3HN VK3HN K", "DE VK3HN ", "VK3HN ", "73 TU . ." };

#ifdef SP_9
byte  dot_length_ms = 65;  // 
#define KEYER_MSG_SPEED 45 // make the keyer play out canned messages faster 
String morse_msg[] = {"CQ SOTA VK3HN/P K"};
//String morse_msg[] = {"1" };  // for testing
#endif

// morse reference table
struct morse_char_t {
  char ch[7]; 
};

morse_char_t MorseCode[] = {
  {'A', '.', '-',  0,   0,   0,   0},
  {'B', '-', '.', '.', '.',  0,   0},
  {'C', '-', '.', '-', '.',  0,   0},
  {'D', '-', '.', '.',  0,   0,   0},
  {'E', '.',  0,   0,   0,   0,   0},
  {'F', '.', '.', '-', '.',  0,   0},
  {'G', '-', '-', '.',  0,   0,   0},
  {'H', '.', '.', '.', '.',  0,   0},
  {'I', '.', '.',  0,   0,   0,   0},
  {'J', '.', '-', '-', '-',  0,   0},
  {'K', '-', '.', '-',  0,   0,   0},
  {'L', '.', '-', '.', '.',  0,   0},
  {'M', '-', '-',  0,   0,   0,   0},
  {'N', '-', '.',  0,   0,   0,   0},
  {'O', '-', '-', '-',  0,   0,   0},
  {'P', '.', '-', '-', '.',  0,   0},
  {'Q', '-', '-', '.', '-',  0,   0},
  {'R', '.', '-', '.',  0,   0,   0},
  {'S', '.', '.', '.',  0,   0,   0},
  {'T', '-',  0,   0,   0,   0,   0},
  {'U', '.', '.', '-',  0,   0,   0},
  {'V', '.', '.', '.', '-',  0,   0},
  {'W', '.', '-', '-',  0,   0,   0},
  {'X', '-', '.', '.', '-',  0,   0},
  {'Y', '-', '.', '-', '-',  0,   0},
  {'Z', '-', '-', '.', '.',  0,   0},
  {'0', '-', '-', '-', '-', '-',  0},
  {'1', '.', '-', '-', '-', '-',  0},
  {'2', '.', '.', '-', '-', '-',  0},
  {'3', '.', '.', '.', '-', '-',  0},
  {'4', '.', '.', '.', '.', '-',  0},
  {'5', '.', '.', '.', '.', '.',  0},
  {'6', '-', '.', '.', '.', '.',  0},
  {'7', '-', '-', '.', '.', '.',  0},
  {'8', '-', '-', '-', '.', '.',  0},
  {'9', '-', '-', '-', '-', '.',  0},
  {'/', '-', '.', '.', '-', '.',  0},
  {'?', '.', '.', '-', '-', '.', '.'},
  {'.', '.', '-', '.', '-', '.', '-'},
  {',', '-', '-', '.', '.', '-', '-'}
};

byte   curr_msg_nbr;   // index into morse_msg[] array
byte   cw_msg_index;   // index into morse_msg[cw_msg_index] array
#endif
// end CW Keyer block -------------------------------------------------------------

byte curr_line = 0;    // the currently selected filter control line

// struct for 'VFO parameter set' records -- the parameters that will change with each VFO
typedef struct {
  boolean  active;
  uint32_t vfo;
  uint32_t radix;
} VFOset_type;

VFOset_type VFOSet[NBR_VFOS]; // array of band parameter sets
byte v;                       // index into VFOSet array (representing the current VFO)
byte v_prev;

Si5351 si5351;                // I2C address defaults to x60 in the NT7S lib

// USB/LSB initialisation
#define SIDEBAND_THRESHOLD  10000000ULL  // threshold VFO freq for auto sideband selection: above use USB, below use LSB
volatile uint32_t bfo = LSB; // the actual BFO freq for si5351 CLK2, arbitrary set to LSB, reset in main loop  

// ------------------------------------------------------------------------------------------------------------------

// variables for transmit-receive control 
bool mode_tx = false; 
bool mode_cw = false; 
bool mode_tune = false; 

// LPF control variables
byte LPF_line=0;           // current LPF line (0..5), set in set_filters()
bool LPF_engaged = false;  // to allow the LPF to be engaged just in time
unsigned long  last_T_R_ms = 0;   // time of last Transmit to receive change

// button variables used in main loop for sensing the multiplexed buttons
byte button_nbr; 
byte old_button_nbr = 0; 

// S meter reading
String S_meter; 
int s_meter_reading=100; 
int s_meter_update_cnt = 0;
int last_s_meter_val = 0;

// ------------------------------------------------------------------------------------------------------------------
//-- VSWR meter code begins ---// Note: left without #defines purposely
  int fwd_max=0, rev_max=0; 
//-- VSWR meter code ends ---------------------------------------------

bool func_button_pressed = false; // if true, the next button pressed is interpreted as a Function 
bool BFO_tune_flg = false;       // BFO Tune feature
byte dial_tick=0;

// ------------------------------------------------------------------------------------------------------------------
// variables for controlling EEPROM writes
unsigned long last_freq_change_ms;
bool eeprom_written_since_last_freq_change; 
bool changed_f = false;

/**************************************/
/* Interrupt service routine for encoder frequency change */
/**************************************/
volatile boolean flag;
volatile boolean rot_type;
ISR(PCINT2_vect) {
  flag = true;
  #ifndef ENCODER_ROTARY
    rot_type = false;
    unsigned char result = r.process();
    if (result == DIR_CW)
      set_frequency(1);
    else if (result == DIR_CCW)
      set_frequency(-1);
  #endif
}

/**************************************/
/* Change frequency; dir=1 Increment; dir=-1 Decrement
/**************************************/
void set_frequency(short dir)
{
#ifdef ENCODER_OPTICAL_360
  if(++dial_tick%dial_speed != 0) return;  // damp the (very fast) 360PPM optical encoder
#endif

  if(mode_tx) return;  // dial locks in transmit

  if (dir == 1)
  {
      if(!BFO_tune_flg)
         VFOSet[v].vfo += VFOSet[v].radix;
      else
         if(bfo < BFO_TUNE_HI) bfo += 100;
  }
  else 
  {
     if (dir == -1)
       if(!BFO_tune_flg)
          VFOSet[v].vfo -= VFOSet[v].radix; 
       else
          if(bfo > BFO_TUNE_LO) bfo -= 100; 
  };

  if(BFO_tune_flg)
  {
    if(VFOSet[v].vfo >= SIDEBAND_THRESHOLD) 
    {
      USB = bfo;
    }
    else 
    {
      LSB = bfo;  
    }
  }  
  changed_f = 1;
};

// Decode the optical decoder
void vfo_decoder(void) {
  // No complete step yet.
  #define DIR_NONE 0x0
  // Clockwise step.
  #define DIR_CW 0x10
  // Anti-clockwise step.
  #define DIR_CCW 0x20

  up_step_counter ++;
  down_step_counter ++;
  unsigned char result;
  
  // if(m_speed != speed[gen_set_mem_array[speed_select]]){ // added to stop encoder lag when switch for slow to fast speeds
  //     up_step_counter = 0;
  //     down_step_counter = 0;
  //     m_speed = speed[gen_set_mem_array[speed_select]];  
  //   }
  if(invert_encoder_direction == true){enc_dir =0;}else{enc_dir =1;}    
    if (digitalRead(vfo_enc_dir) == enc_dir){
      down_step_counter = DIR_NONE;
      if(up_step_counter != 0){
        result = DIR_CW;
      }
    }else{
      up_step_counter = DIR_NONE;
      if(down_step_counter != 0){
        result = DIR_CCW;
    }
  } //end of digital read encoder if statement
  if (result == DIR_CW)
    set_frequency(1);
  else if (result == DIR_CCW)
    set_frequency(-1);

  Serial.println(counter);
} // end of vfo_decoder

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
    set_frequency(1);
  }else{
    counter--;
    set_frequency(-1);
  }
}// end ai0
  
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
    set_frequency(-1);
  }else{
    counter++;
    set_frequency(1);
  }
}// end ai1


int read_analogue_pin(byte p)
{
// Take an averaged reading of analogue pin 'p'  
  int i, val=0, nbr_reads=2; 
  for (i=0; i<nbr_reads; i++)
  {
    val += analogRead(p);
    delay(1); 
  }
  return val/nbr_reads; 
};


#ifdef CW_KEYER
int read_keyer_speed()
{ 
  int n = read_analogue_pin((byte)PIN_KEYER_SPEED);
  //Serial.print("Speed returned=");
  //Serial.println(n);
  dot_length_ms = 60 + (n-183)/5;   // scale to wpm (10 wpm == 60mS dot length)
                                     // '511' should be mid point of returned range
                                     // change '5' to widen/narrow speed range...
                                     // smaller number -> greater range  
  return n;
};
#endif



byte get_front_panel_button()
// Take a reading of the front panel buttons and map it to a button number (0..4)
// Take multiple readings and average them
{
  byte b=0; 
  int z;
  z = read_analogue_pin((byte)SWITCH_BANK);
//  Serial.print("Frnt bttn="); Serial.println(z);
//----------------------------------------------------------------------------------

#ifdef SP_9
  if(z > 2 && z < 100) b = 4;       // 15   B4   increment the band (with wrap-around)
  if(z >= 100 && z < 750) b = 6;    // 560   B4   increment the band (with wrap-around)
  if(z >= 750 && z < 1000) b = 1;   // 842   B4   increment the band (with wrap-around)

  // if(!digitalRead(vfo_enc_dir)){         // on this rig, the mechanical encoder's pushbutton is used to change radix
  //   while(!digitalRead(vfo_enc_dir)) ;   // spin here until encoder button is released
  //   b = 6;                           // B6  increment the radix (with wrap-around)
  // }

#ifdef DIAGNOSTIC_DISPLAY
  diagnostic_flag = true;  // trace on the display as a diagnostic
  diagnostic_int = z;      // trace 'z' 
  refresh_display();
#endif            
#endif    

  if(b>0){ Serial.print("Front button="); Serial.print(b); Serial.print("   z="); Serial.println(z);}
  return b;  
} // get_front_panel_button()



void read_meter()
// in receive mode, reads an analog port and constructs an S-meter
// in transmit mode, reads 2 analog ports and constructs a power/SWR meter
// analogue port returns 0..1023;  meter string is 8 chars. 
{
  int sum, y, z;
  byte width = 8;

  int s_point_val[width] = {0, 128, 256, 384, 512, 640,  768,  896};
  //                        s1 s3   s5   s7   s9   s9+20 s9+40 s9+60
  
  if(mode_tx)
  {
    // calculate SWR or relative power using fwd_max, rev_max 
    z = fwd_max*10;  // arbitrary scaling for a relative output power indication    
  }
  else
  {
//    if(((++s_meter_update_cnt) % 2)!=0) return; // dampen a jittery s-meter 
    // take an S-meter reading

#define NBR_SAMPLES 2
// #if defined(SP_7) <<--- Maybe need to leave this in? Maybe it's for testing? 
//     // take an averaged reading from the analogue pin with the s-meter on it
//     for (byte i=0; i<NBR_SAMPLES; i++) sum += analogRead(PIN_S_METER);    // read PIN_S_METER analog pin
//     y = sum/NBR_SAMPLES;   
//     // specific mappings for each particular receiver's AGC characteristics 
    
// #ifdef SP_7
//     switch (curr_line){
//     case 1: z = map(y, 80, 140, 900, 80); break;  // 80m
//     case 2: z = map(y, 80, 180, 900, 80); break;  // 40m
//     case 3: z = map(y, 80, 140, 900, 80); break;  // 30m
//     case 4: z = map(y, 80, 180, 900, 80); break;  // 20m
//     }
// #endif

// #else
    z = s_meter_reading + rand()/100; // assign arbitrary value to try out a moving s-meter
// #endif 
  }

//  Serial.print(curr_line ); Serial.print(" s-meter y="); Serial.print(y); Serial.print(" s-meter z="); Serial.println(z);

  S_meter = "1 :"; 
  S_meter += (char)165;    // for s1 - s9 print a small 'centered dot'

  if(!mode_tx)
  {
    if((z < s_point_val[1]) && (random(0,10) > 5)) 
    { 
      // we have a low reading, so simulate a bit of band noise (make the s-meter flicker between s1 and s2) 
      S_meter += (char)165;  // small 'centered dot'
      S_meter[0] = char(50); // '2'
      //    delay(100);  // careful, this will introduce a delay into tuning
    }
  }
  
  int s = 1;

  // now construct the meter string in either case
  for (byte j=0; j<width; j++)
  {
    if(z > s_point_val[j])
    {
      s=s+1; 
      // write in a visible char to the s-meter bar
      if(j<4)
      {
        S_meter += (char)165;    // for s1 - s9 append a small 'centered dot' 
        S_meter[0] = char(49+s); // numeric '1'..'9' 
        S_meter[1] = char(32);   // " ";
      }
      else if(j==4) // S9
      {
        S_meter += (char)165;    // for s1 - s9 append a small 'centered dot' 
        S_meter[0] = char(57);   // numeric '1'..'9' 
        S_meter[1] = char(32);   // " ";
      }
      else // j is 5..7
      {
        S_meter += (char)219;    // for s9+ append a large 'centered dot'     (char)219
        S_meter[0] = char(57);   // '9' 
        S_meter[1] = char(43);   // '+'
      }
    }
    else
      S_meter += ' ';     // append a blank
  }    

//  if(mode_tx)
//  {
//   Serial.print(sizeof(S_meter)); Serial.print(" s <"); Serial.print(S_meter); Serial.println(">");
//  }
  return;
}


// ------------------------------------------------------------------------------------------------------------------
// display routines



void refresh_display()
{
// Update the display (may be Liquid Crystal Display, OLED) 
// call thru to the function for the installed display type
#ifdef DISPLAY_OLED
  refresh_OLED();
#endif
}


void refresh_OLED(){
#ifdef DISPLAY_OLED
// Update the OLED display 

#ifdef DIAGNOSTIC_DISPLAY
  if(diagnostic_flag)
  {
    oled.clear();
    oled.set1X();
    oled.setCursor(0, 0);
    oled.print(diagnostic_int, DEC);
    return;
  };
#endif  // DIAGNOSTIC_DISPLAY
  
  
  oled.set2X();
  oled.setCursor(0, 0);

  unsigned long f_Hz = VFOSet[v].vfo;
    
  uint16_t f = f_Hz/1000;   // frequency in kHz

  int mhz = f/1000;                       // MHz (0..99)
  int khz = f - mhz*1000;                 // kHz (0..999)
//  int hz = VFOSet[v].vfo - (mhz*1000000 + khz*1000);  // Hz (0..999)   // original line 
  int hz = f_Hz - (mhz*1000000 + khz*1000);  // Hz (0..999)
  
//  Serial.print(" hz="); Serial.println(hz);
  if(mhz<10) oled.print(" ");
  oled.print(mhz, DEC);
//  if(mhz<100) oled.print(",");
  oled.print(",");
  if(khz<10) oled.print("0");
  if(khz<100) oled.print("0");
  oled.println(khz, DEC);

  oled.set1X();
  oled.setCursor(0, 4);

  if(mode_tx)
  {
    // we are transmitting
    if(mode_cw)  
      oled.print(  "CW    ");
    else 
    {
      if(mode_tune)
        oled.print("TUNE  ");
      else 
        oled.print("Tx    ");
    }       
  }
  else
  {
    // we are receiving
//    Serial.print("radix="); Serial.println(VFOSet[v].radix); 

    if(VFOSet[v].radix==10000) oled.print("10kHz ");
    if(VFOSet[v].radix==1000)  oled.print("1kHz  ");
    if(VFOSet[v].radix==100)   oled.print("100Hz ");
    if(VFOSet[v].radix==10)    oled.print("10Hz  ");
  }
  
  oled.setCursor(84, 4);  // was 96, 4
  oled.print(".");
  if(hz<10) oled.print("0");
  if(hz<100) oled.print("0");
  oled.println(hz, DEC);
  
  oled.setCursor(0, 6);
  oled.print("V");
  oled.print(v+1);
  oled.print(" ");
  oled.setCursor(0, 7);

  int val = 0, s=0;
  if(!mode_tx) 
  {
    oled.setCursor(30, 6);
    oled.print("S");
    oled.setFont(s_meter10x15);

    // read and format the S-meter 
//    byte s = (rand()%2)+2;
    val = analogRead(PIN_S_METER);

#ifdef SP_9  
    s = map(val, 20, 1023, 8, 1);  // map s-meter analogue reading to s-scale
    //  add some decay...
    if(s > last_s_meter_val) {}
    else if(s < last_s_meter_val) s = last_s_meter_val - 1;
    last_s_meter_val = s; 
#endif
   
//    Serial.print(" val=");  Serial.print(val); Serial.print(" s=");  Serial.println(s);     
    byte c=0;
    for(byte j=1; j<=s; j++){
      oled.print((char)j);
      c++;
    };
    oled.setFont(fixed_bold10x15);
    for(byte k=8; k>c; k--) oled.print(" "); // right pad the s-meter display space
  }
  else
  {
    // in transmit mode, so display a relative RF power meter
    if(!mode_cw) {
      pwr_val = read_analogue_pin(PIN_PWR_METER);
//      Serial.print(" refresh_OLED():: pwr_val=");  Serial.println(pwr_val);       
    }
    refresh_pwr_meter();
  };
#endif // #ifdef DISPLAY_OLED
}


void refresh_pwr_meter()
{
#ifdef DISPLAY_OLED
  // write to the power meter line on the OLED 
  oled.setCursor(30, 6);
  oled.print("P");
  oled.setFont(s_meter10x15);
        
  // do band-specific sensitivity scaling
//    if((VFOSet[v].vfo >= FILTER_80_LB) && (VFOSet[v].vfo <= FILTER_80_UB)) val = val*3;   // '2' is the magic number for 80m

  // NOTE: the RF power sensing analog pin is read elsewhere, and the value stored in global pwr_val 
  byte s = map(pwr_val, 0, 550, 1, 8);
    
//  Serial.print(" refresh_pwr_meter()::pwr_val=");  Serial.print(pwr_val);  Serial.print(" s=");  Serial.println(s);  
  byte c=0;
  for(byte j=1; j<=s; j++){
    oled.print((char)j);
    c++;
  };
  oled.setFont(fixed_bold10x15);
  for(byte k=8; k>c; k--) oled.print(" "); // right pad the pwr-meter display space
#endif // #ifdef DISPLAY_OLED
};

// ------------------------------------------------------------------------------------------------------------------
// filter control

void set_filters(uint32_t f)
{
    // select the appropriate filter set for the frequency passed in
    // Note that LPF_line and curr_line are global
    byte BPF_line=0;  
    bool band_changed = false;

    if ((f >= FILTER_160_LB) && (f <= FILTER_160_UB)) 
    {
      BPF_line = 1;
      LPF_line = 1;
    }
    else if((f >= FILTER_80_LB) && (f <= FILTER_80_UB))   
    {
      BPF_line = 2;
      LPF_line = 2;
    }
    else if((f >= FILTER_60_LB) && (f <= FILTER_60_UB))   
    {
      BPF_line = 7;
      LPF_line = 7;
    }
    else if((f >= FILTER_40_LB) && (f <= FILTER_40_UB))   
    {
      BPF_line = 3;
      LPF_line = 3;
    }
    else if((f >= FILTER_30_LB) && (f <= FILTER_30_UB))   
    {
      BPF_line = 4;
      LPF_line = 4;
    }
    else if((f >= FILTER_20_LB) && (f <= FILTER_20_UB))   
    {
      BPF_line = 5;
      LPF_line = 5;
    }
    else if((f >= FILTER_17_LB) && (f <= FILTER_17_UB))   
    {
      BPF_line = 6;
      LPF_line = 5;
    }
    else if((f >= FILTER_15_LB) && (f <= FILTER_15_UB))   
    {
      BPF_line = 7;
      LPF_line = 6;
    };

#ifdef SP_9
    // BPF and LPFs share the same decoder 
    // 80m BPF_line = 2->1   
    // 60m BPF_line = 7->2   
    // 40m BPF_line = 3->3   
    // 30m BPF_line = 4->4   
    // 20m BPF_line = 5->5   
  
    if(BPF_line == 2) BPF_line = 1;
    if(BPF_line == 7) BPF_line = 2;
    if(BPF_line == 3) BPF_line = 3;
    if(BPF_line == 4) BPF_line = 4;
    if(BPF_line == 5) BPF_line = 5;
    LPF_line = BPF_line;               // (LPF_line not used) 
#endif       

//-----------------------------
//   Serial.print("BPF_line, LPF_line="); Serial.print(BPF_line);Serial.print(","); Serial.println(LPF_line); 

  // handle the BPFs, if the band has changed
    byte prev_BPF_line = curr_line; 
    if(BPF_line != curr_line)
    {
        // *** raise the appropriate control line 
        // (where 0 means no line which will implement 'out of band' protection
        for (int i=0; i<8; i++) PCF_BPF.write(i, 0); //turn all pins of the I/O expander off 
        delay(50);
        if(BPF_line > 0){

           // may need to do some BPF_line mapping here...

           PCF_BPF.write(BPF_line - 1, 1);  //turn the band-specific pins of the I/O expander on 
           Serial.print("New band:");
           Serial.println(BPF_line);
           delay(50); 
           band_changed = true;
        }
        curr_line = BPF_line;  
    }

    // handle the LPFs, if the band has changed
    if(band_changed)
    {
        // the band changed, so reset the LPF
        for (byte i=0; i<8; i++) PCF_LPF.write(i, 0); //turn all pins of the I/O expander off 
                                                     // all relays will drop out
        if(LPF_line > 0){
           byte pcf_pin = LPF_line - 1;
           // handle LPF_line to PCF8574 pin mapping for specifric rigs here
           PCF_LPF.write(pcf_pin, 1);  //turn the band-specific pin of the I/O expander on 
          // Serial.print("1:");   Serial.println(PCF_LPF.lastError());

           Serial.print("New LPF:");
           Serial.println(LPF_line);
           LPF_engaged = true; 
          // delay(20); 
        }
    };
}

// ------------------------------------------------------------------------------------------------------------------
// EEPROM

void update_eeprom()
{
  if(abs( millis() - last_freq_change_ms ) > 10000)
  {
    if(eeprom_written_since_last_freq_change == false)
    {
      // do the eeprom write
      // Serial.println("*** eeprom write");
      EEPROM.write(0, v);   // write the band index (v) to the first byte
        
      int element_len = sizeof(VFOset_type);
      for(int i=0; i<NBR_VFOS ; i++)    // write each element of the VFOSet array
      {
        EEPROM.put(1 + (i * element_len), VFOSet[i]);
      }
      eeprom_written_since_last_freq_change = true;
    }
  }
};


// ------------------------------------------------------------------------------------------------------------------
// TR switching and sequencing 

void receive_to_TRANSMIT()
{
  mode_tx = true;
  Serial.println("receive_to_TRANSMIT()");
  digitalWrite(RX_MUTE_LINE, RX_MUTE_ON_VALUE);     // mute the receiver
  delay(RX_MUTE_DELAY_MS);

  // pull in the current LPF (if it is not engaged already)
  if(!LPF_engaged)
  {
    byte pcf_pin = LPF_line - 1;
    PCF_LPF.write(pcf_pin, 1);  //turn the band-specific pin of the I/O expander on 
//     Serial.print("2:");   Serial.println(PCF_LPF.lastError());

    delay(5);
    Serial.print("JIT LPF:"); Serial.println(LPF_line);
    LPF_engaged = true; 
    last_T_R_ms = millis(); 

    // start the fan, if there is one...
 
  };
  
  digitalWrite(TRANSMIT_LINE, 1); // pull in the T/R relay
  delay(10); 

  if(mode_cw)
  {
    // Serial.println("  mode_cw==true");  
    si5351.output_enable(SI5351_CLK0, 0);  // turn the VFO off
    // Serial.println("VFO disabled");

#ifdef BFO_ENABLED
    si5351.output_enable(SI5351_CLK2, 0);  // turn the BFO off
#endif
    
    // prime CLK1 to the current frequency (+- CW offset), ready to transmit

    volatile uint32_t f; 
    f = VFOSet[v].vfo; 
    // choose whether to add or subtract the CW offset
    if(f >= SIDEBAND_THRESHOLD) 
      f += CW_TONE_HZ; 
    else 
      f -= CW_TONE_HZ;
    
    Serial.print("CO:"); Serial.println(f);
    si5351.set_freq(f * SI5351_FREQ_MULT, SI5351_CLK1);
    if(f < CO_DRIVE_THRESHOLD)
    {
      si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_4MA);       
//      Serial.println("CO Lo");
    }
    else
    {
      si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA); 
//      Serial.println("CO Hi");
    }
    
    si5351.output_enable(SI5351_CLK1, 0); // turn the CW clock off until keyed
    dot_dash_sent = 0;  // count nbr chars sent this transmit period

#ifdef SP_9 
    digitalWrite(CO_DC_SUPPLY, 1);  // powers up the carrier oscillator buffer 
#endif
 
  }
  else
  {
    // mode is NOT CW ...

#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined(SS_AM_TX_TEST_VFO) || defined(SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
    si5351.set_freq(VFOSet[v].vfo * SI5351_FREQ_MULT, SI5351_CLK0); // turn on the drive (VFO on signal frequency, CLK0)
    si5351.output_enable(SI5351_CLK0, 1); 
    si5351.output_enable(SI5351_CLK2, 0);   // turn receiver VFO off, if it is being used 
    delay(20);
    digitalWrite(PWM_ENABLE_LINE, 1);     // enable the Pulse Width Modulator    
#endif
        
#ifdef VFO_BFO_SWAP
    // swap the VFO and the BFO on CLK0 and CLK2 for the transmit period
    si5351.output_enable(SI5351_CLK0, 0);  // turn VFO off 
    si5351.output_enable(SI5351_CLK2, 0);  // turn BFO off 
    delay(10);
    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK0); // set CLK0 to the BFO frequency    
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to  VFO+IF freq for current band 
    si5351.output_enable(SI5351_CLK0, 1);  // turn reversed BFO back on 
    si5351.output_enable(SI5351_CLK2, 1);  // turn reversed VFO back on 
#endif
  }
};


void TRANSMIT_to_receive()
{
  mode_tx = false;
  Serial.println("TRANSMIT_to_receive()");
  
  digitalWrite(TRANSMIT_LINE, 0); // drop out the T/R relay
  delay(10);

  if(mode_cw)
  {
    // Serial.println("  mode_cw==true");
    si5351.output_enable(SI5351_CLK0, 1);  //  turn the VFO back on again 
    if(VFOSet[v].vfo < VFO_DRIVE_THRESHOLD)
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); 
    else
    {
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA);
    } 
      
#ifdef BFO_ENABLED
    si5351.output_enable(SI5351_CLK2, 1);  //  turn the BFO back on again 
//    Serial.println("VFO enabled");
#endif 

#ifdef SP_9 
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif
  }
  else
  {
    // mode is SSB (or AM)  
    
#ifdef VFO_BFO_SWAP
    // swap the VFO and the BFO on CLK0 and CLK2 back to their normal clocks 
    si5351.output_enable(SI5351_CLK0, 0);  // turn BFO off 
    si5351.output_enable(SI5351_CLK2, 0);  // turn VFO off 
    delay(10);
    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to the BFO frequency    
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK0); // set CLK0 to  VFO+IF freq for current band 
    si5351.output_enable(SI5351_CLK0, 1);  // turn reversed VFO back on 
    si5351.output_enable(SI5351_CLK2, 1);  // turn reversed BFO back on 
#endif    
  };

  delay(3 * RX_MUTE_DELAY_MS);
  digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // un-mute the receiver

  mode_cw = false;  // mode will be set next time the paddle or keyer is touched
  last_T_R_ms = millis();  // start the LPF drop out timer from now
  pwr_val = 0;  // flush out residual RF power reading

};

// ------------------------------------------------------------------------------------------------------------------


void tune()
// puts the carrier on, for TUNE_MS milliseconds
// NOTE - this function relies on some functions inside the CW_KEYER block -- factor these out!
{
  if(!mode_tx)
  {
    // prime CLK1 to the current frequency 
     si5351.set_freq(VFOSet[v].vfo * SI5351_FREQ_MULT, SI5351_CLK1);
     si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); 
     si5351.output_enable(SI5351_CLK1, 0); // turn the CW clock off until keyed
     receive_to_TRANSMIT(); 
     
#ifdef SP_9
    digitalWrite(CO_DC_SUPPLY, 1);  // power up the carrier oscillator buffer 
#endif

     tone(PIN_TONE_OUT, CW_TONE_HZ);
     Serial.println("Tune -- Carrier on"); 
     set_key_state2('D');  
     refresh_display();

     delay(TUNE_MS);  // key down for this number of mS
     Serial.println("Tune -- Carrier off"); 
     set_key_state2('U');  

     noTone(PIN_TONE_OUT);
     
#ifdef SP_9
    digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
#endif

     TRANSMIT_to_receive(); 
  }
  mode_tune = false; 
}


void set_key_state2(char k)
{
// push the morse key down, or let it spring back up
// changes global 'key_state' (bool)

  if(!key_down and k=='D')
  {
    // do whatever you need to key the transmitter
    digitalWrite(13, 1);  // turn the Nano's LED on
    si5351.output_enable(SI5351_CLK1, 1); // turn the (CW clock) on
    key_down = true; 
  };

  if(key_down and k=='U')
  {
    // do whatever you need to un-key the transmitter 
    digitalWrite(13, 0);  // turn the Nano's LED off
    si5351.output_enable(SI5351_CLK1, 0); // turn the (CW clock) off

    char_sent_ms = millis();
    space_inserted = false;
    key_down = false; 
  };
}  

#ifdef CW_KEYER
// start of CW Keyer block ------------------------------------------------------------------------------------------

int morse_lookup(char c)
// returns the index of parameter 'c' in MorseCode array, or -1 if not found
{
  for(int i=0; i<sizeof(MorseCode); i++)
  {
    if(c == MorseCode[i].ch[0])
      return i;
  }
  return -1; 
};


byte check_keyer_pushbutton() // <<--- Hope to replace with Button Fever?
{
// Reads the keyer pushbuttons and returns the button number as a byte; 0 if not pressed  
  byte b=0; 
  int  z;
  z = read_analogue_pin(PIN_PUSHBTTN_REAR);    // read the analog pin

// Serial.print("Kyr pshbtn z="); Serial.println(z);

#ifdef SP_9
  if(z <= 100) b = 1;       //  15
#endif

  if(b>0){
      Serial.print("Keyer pushbutton="); Serial.print(b); Serial.print(", z="); Serial.println(z);
    }
  return b;
}


byte check_paddle()
{
// Reads the paddle, returns the paddle number as a byte; 0 if not pressed  
  byte b=0; 
  int  z=0;

  z = read_analogue_pin(PIN_PADDLE);    // read the analog pin
  
#ifdef DIAGNOSTIC_DISPLAY
  diagnostic_flag = true;  // trace on the display as a diagnostic
  diagnostic_int = z;      // trace 'z' 
//  delay(200);
#endif 

// Serial.print("Kyr pdl, z="); Serial.println(z);

#ifdef SP_9
  if(z < 30) b = 2;                 // L 14
  else if(z > 30 && z < 100) b = 1; // R 110 
#endif

  if(b>0){
//     Serial.print("Kyr pdl, b="); Serial.print(b); 
//     Serial.print(", z="); Serial.println(z); 
  }
  return b;
};

void activate_state2(char c)
{
// if necessary, activates the receiver or the transmitter 
// 'c' may be either 'T' or 'R' 

  if(!mode_tx && c=='T') receive_to_TRANSMIT();
  else 
    if(mode_tx && c=='R') TRANSMIT_to_receive();
  // in  all other cases, do nothing!
}

void send_dot()
{
  delay(dot_length_ms);  // wait for one dot period (space)

  // send a dot and the following space
  tone(PIN_TONE_OUT, CW_TONE_HZ);
  set_key_state2('D');  
  if(dot_dash_counter % SERIAL_LINE_WIDTH == 0) Serial.println(); 
  Serial.print(".");
  delay(dot_length_ms);  // key down for one dot period
  noTone(PIN_TONE_OUT);
  
  // read the RF power sensing circuit while the key is down and store the result for later display
  pwr_val = read_analogue_pin(PIN_PWR_METER);

  set_key_state2('U');  
  dot_dash_counter++;
  dot_dash_sent++;
}

void send_dash()
{
  delay(dot_length_ms);  // wait for one dot period (space)
  // send a dash and the following space
  tone(PIN_TONE_OUT, CW_TONE_HZ);
  set_key_state2('D');  
  if(dot_dash_counter % SERIAL_LINE_WIDTH == 0) Serial.println(); 
  Serial.print("-");
  delay(dot_length_ms * CW_DASH_LEN);  // key down for CW_DASH_LEN dot periods
  noTone(PIN_TONE_OUT);

  // read the RF power sensing circuit while the key is down and store the result for later display
  pwr_val = read_analogue_pin(PIN_PWR_METER);

  set_key_state2('U');  
  dot_dash_counter++;
  dot_dash_sent++;
}

void send_letter_space()
{
  delay(dot_length_ms * 4);  // wait for 3 dot periods
  Serial.print(" ");
}

void send_word_space()
{
  delay(dot_length_ms * 7);  // wait for 6 dot periods
  Serial.print("  ");
}

void send_morse_char(char c)
{
  // 'c' is a '.' or '-' char, so send it 
  if(c == '.') send_dot();
  else if (c == '-') send_dash();
  // ignore anything else, including 0s
}

void play_message(String m, int s)
{
// sends the message in string 'm' as CW, with inter letter and word spacing
// s is the speed to play at; if s == 0, use the current speed  
  byte j, n, old_s; 
  char buff[30];   // magic number, should guard this!

  message_playing = true; 
  Serial.println(m);
//  Serial.println(m.length());

  // use ch = m.charAt(index);
  m.toCharArray(buff, m.length()+1);

  if(s > 0)  // caller has passed in a speed to send message at 
  {
    old_s = dot_length_ms; // preserve the current keyer speed
    dot_length_ms = s;
  }
//  Serial.print("play_message()::dot_length_ms:");   Serial.println(dot_length_ms);

  //activate_state2('T'); 

  for (cw_msg_index=0; cw_msg_index<m.length(); cw_msg_index++)
  {
    if(buff[cw_msg_index] == ' ') 
    {
       send_word_space(); 
    }
    else
    {
      if( (n = morse_lookup(buff[cw_msg_index])) == -1 )
      {
        // char not found, ignore it (but report it on Serial)
//        Serial.print("!! not found <");
//        Serial.print(buff[cw_msg_index]);
//        Serial.println(">");
      }
      else
      {
        // char found, so send it as dots and dashes
        // Serial.println(n);
        refresh_display();         // first, refresh the LCD so the current CW char gets displayed

        for(j=1; j<7; j++)
          send_morse_char(MorseCode[n].ch[j]);
        send_letter_space();  // send an inter-letter space
        // if(s==0)read_keyer_speed();  // see if speed has changed mid message ***
      } // else
    } // else 
  } // for  
  Serial.println();
  if(s > 0)  // reset speed back to what it was  
    dot_length_ms = old_s;

  //activate_state2('R'); 
  message_playing = false; 
  curr_msg_nbr = 0;
  cw_msg_index = 0;
} // play_message
// end CW Keyer block -------------------------------------------------------------
#endif


// ------------------------------------------------------------------------------------------------------------------
// setup()
void setup()
{
  Serial.begin(9600);
  if (flag) {
    Serial.print("ISR interrupt has occurred. Is optical? ");
    if (rot_type) {
      Serial.println("TRUE");
    } else {
      Serial.println("FALSE");
    }
    flag = false;
  }
  Wire.begin();
  
#ifdef ENCODER_OPTICAL
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
  attachInterrupt(0, ai0, RISING);   
  attachInterrupt(1, ai1, RISING);
#endif


#ifdef SP_9
  Serial.println("SP_9 ...");
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS, RST_PIN);
#else // RST_PIN < 0
  oled.begin(&Adafruit128x64, I2C_OLED_ADDRESS);
#endif // RST_PIN >= 0
  oled.setFont(fixed_bold10x15);
  oled.clear();
  oled.println("Summit Prowler");
  oled.println("      '9'     ");
  oled.println("   < AI5LC >  ");
  oled.println(" R0.1 03/2024 ");
  delay(1000);
  oled.clear();

  // pinMode(ENCODER_BUTTON, INPUT_PULLUP); // this rig uses ENCODER_BUTTON to read the encoder pushbutton for 'step' (radix) control
  pinMode(CO_DC_SUPPLY, OUTPUT);  // this rig uses this pin to control DC power to the carrier osc buffer
  digitalWrite(CO_DC_SUPPLY, 0);  // power down the carrier oscillator buffer 
  
#ifdef  CW_KEYER
  pinMode(PIN_PADDLE, INPUT_PULLUP);
  pinMode(PIN_PUSHBTTN_REAR, INPUT_PULLUP);
#endif
#endif

// reset the Band/Low Pass Filter controllers
  for (byte i=0; i<8 ; i++) 
  {
    PCF_BPF.write(i, 0); //turn all the BPF relays off 
    PCF_LPF.write(i, 0); //turn all the LPF relays off 
//    Serial.print(" 0: ");   Serial.println(PCF_LPF.lastError());
  }

// set digital and analogue pins
  pinMode(SWITCH_BANK, INPUT_PULLUP); // switch bank is Pull-up
  pinMode(PTT_SENSE, INPUT_PULLUP);  // senses the PTT switch in the microphone, normally open, grounded when PTT pressed
  
  pinMode(RX_MUTE_LINE, OUTPUT); 
  digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // initialise mute line to 'off' value (un-mute)  
  
  pinMode(TRANSMIT_LINE, OUTPUT); 
  digitalWrite(TRANSMIT_LINE, 0); // put the transmit line low (relay not energised)

// start with transmit line low (in receive) 
  mode_tx = false;
  
  // PCICR |= (1 << PCIE2);           // Enable pin change interrupt for the encoder
  // PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  // sei();
  
  // load up VFOSet array from EEPROM

  v = EEPROM.read(0);
  Serial.print("setup() eeprom: v=");
  Serial.println(v);
  if(v >= NBR_VFOS) v = 1;  // in case NBR_VFOS has been reduced since the last run (EEPROM update)
  v_prev = v;
  
  int element_len = sizeof(VFOset_type);
  for(int i=0; i < NBR_VFOS; i++)
  {
    EEPROM.get(1 + (i * element_len), VFOSet[i]);
  };

/* // initialise VFOSet array
  for(int n=0; n<NBR_VFOS; n++) VFOSet[n].active = 0;   // make sure all are inactive to start with 
  VFOSet[0] = (VFOset_type){1,  1825000ULL, 1000};
  VFOSet[1] = (VFOset_type){1,  3525000ULL, 1000};
  VFOSet[2] = (VFOset_type){1,  3625000ULL, 1000};
  VFOSet[3] = (VFOset_type){1,  7025000ULL,  100};
  VFOSet[4] = (VFOset_type){1,  7090000ULL, 1000};
  VFOSet[5] = (VFOset_type){1,  7125000ULL, 1000};
  VFOSet[6] = (VFOset_type){1, 10105000ULL,  100};
  VFOSet[7] = (VFOset_type){1, 14060000ULL, 1000};
  VFOSet[8] = (VFOset_type){1, 18068000ULL, 1000};
  VFOSet[9] = (VFOset_type){1, 18068000ULL, 1000};
*/
// dump out the VFOset array for diagnostics
  for(int n=0; n < NBR_VFOS ; n++)
  {
    Serial.print((int)VFOSet[n].active);
    Serial.print(" ");
    Serial.print(VFOSet[n].vfo);
    Serial.print(" ");
    Serial.print((long)VFOSet[n].radix);
    Serial.println();
  }
  
// initialise and start the si5351 clocks

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); // If using 27Mhz xtal, put 27000000 instead of 0 (0 is the default xtal freq of 25Mhz)
  
#ifdef SP_V
  si5351.set_correction(15500, SI5351_PLL_INPUT_XO);    // Library update 26/4/2020: requires destination register address  ... si5351.set_correction(19100, SI5351_PLL_INPUT_XO);
#endif 
 
#ifdef SP_6
  si5351.set_correction(19100, SI5351_PLL_INPUT_XO);    // to determine the correction using the serial monitor
#endif  

#ifdef SP_7
  si5351.set_correction(19100, SI5351_PLL_INPUT_XO);    // to determine the correction using the serial monitor
#endif  

#ifdef SP_8
  si5351.set_correction(34000, SI5351_PLL_INPUT_XO);    // calibrated 18/06/2020
#endif 

#ifdef SP_9
  si5351.set_correction(33000, SI5351_PLL_INPUT_XO);    // calibrated 23/11/2020
#endif 

#ifdef SP_X
  si5351.set_correction(28000, SI5351_PLL_INPUT_XO);    // calibrated 25/01/2021
#endif 

#ifdef SS_FAT5_160AM_TX
 si5351.set_correction(160000, SI5351_PLL_INPUT_XO);    // for FAT5  
#endif  

#ifdef SS_VK3SJ_160AM_TX 
  si5351.set_correction(5000, SI5351_PLL_INPUT_XO);     // tune this
#endif 

#ifdef SS_VK3SJ_40AM_TX 
  si5351.set_correction(5000, SI5351_PLL_INPUT_XO);     // tune this
#endif 

#ifdef SS_AM_TX_TEST_VFO 
  si5351.set_correction(18800, SI5351_PLL_INPUT_XO);    // calibrated 3/11/2019 
#endif 

#ifdef SS_VK3SJ_160AM_PORTABLE_TX  
  si5351.set_correction(18800, SI5351_PLL_INPUT_XO);    // calibrated ?? 
#endif 

#ifdef UNIVERSAL_VFO_CONTROLLER
  si5351.set_correction(19100, SI5351_PLL_INPUT_XO);    // 
#endif


                                        
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
  // choose a high or low BFO frequency
  if(VFOSet[v].vfo >= SIDEBAND_THRESHOLD) 
    bfo = USB;
  else 
    bfo = LSB;

// VFO
  Serial.print("VFO=");    Serial.println(VFOSet[v].vfo + bfo);  
  
#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined (SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
  // nothing
#else
  {
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK0); // set CLK0 to VFO freq for current band 
    if(VFOSet[v].vfo < VFO_DRIVE_THRESHOLD)
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); 
    else
    {
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA); 
    }

    si5351.output_enable(SI5351_CLK0, 1);  // turn VFO on 
  }
#endif

#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined(SS_VK3SJ_40AM_TX)
  si5351.output_enable(SI5351_CLK0, 0);  // turn VFO off 
  si5351.output_enable(SI5351_CLK1, 0);  // turn CO off 
  si5351.output_enable(SI5351_CLK2, 0);  // turn BFO off 
#endif

#if defined(SS_AM_TX_TEST_VFO)
  si5351.output_enable(SI5351_CLK1, 0);  // turn VFO off 
  si5351.output_enable(SI5351_CLK2, 0);  // turn BFO off 
#endif

#if defined(SS_VK3SJ_160AM_PORTABLE_TX)
    si5351.output_enable(SI5351_CLK0, 0);  // transmit clock
    
    si5351.set_freq((VFOSet[v].vfo + bfo) * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to VFO for the inbuilt AM receiver  
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); 
    si5351.output_enable(SI5351_CLK2, 1);  

    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK1);  // init the BFO
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); 
    si5351.output_enable(SI5351_CLK1, 0);  // start the receiver in AM mode (no BFO) 
#endif


// BFO
#ifdef BFO_ENABLED  
  si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); // set CLK2 to the BFO frequency for the current band
  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);  // can change this for a stronger BFO
#endif

  changed_f = true; 
  last_freq_change_ms = millis(); 
  eeprom_written_since_last_freq_change = false; 
  LPF_engaged = false;

#ifdef SS_EI9GQ
// initialise the Real Time Clock
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }
   // may or may not be needed for the RTC to work! 
   if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
#endif

#ifdef SP_X
  // turn on the LED on the current channel
  digitalWrite(v+8, 1); 
  delay(200);
#endif

#ifdef CW_KEYER
// start of CW Keyer block -----------------------------------------------------------
  key_down = false;  
  char_sent_ms = millis();
  space_inserted = false;
  message_playing = false;
  curr_msg_nbr = 0;
  cw_msg_index = 0;
  dot_dash_counter = 0;

  // dump out the MorseCode table for diagnostic
/*  for(int i=0; i<40; i++)
  {
    Serial.print(MorseCode[i].ch[0]);
    Serial.print(' ');
    for(int j=1; j<7; j++)
      Serial.print(MorseCode[i].ch[j]);
    Serial.println();
  }

  // play the two messages as a diagnostic
  //  play_message(morse_msg[0], 0);
  //  delay(2000);
  //  play_message(morse_msg[1], 0);
  //  delay(2000);
  */
#endif
// end of CW Keyer block -------------------------------------------------------------

}


// ------------------------------------------------------------------------------------------------------------------
// loop()

void loop()
{
    update_eeprom(); 
#ifdef DISPLAY_LCD
    refresh_display();  
#endif
#ifdef DISPLAY_OLED
    if(!mode_cw) refresh_display();  // to get around slow OLED refresh slowing down CW sending!
#endif

#ifdef ENCODER_OPTICAL
  if( counter != temp ){
  Serial.println (counter);
  temp = counter;
  }
#endif
   
#ifdef VSWR_METER
//-- VSWR meter code begins -------------------------------------------
  fwd_max=0; // resets peak counters each time through loop (this timing may need to change)
  rev_max=0; 
//-- VSWR meter code ends ---------------------------------------------
#endif

  // Update the display if the frequency has been changed (and we are not transmitting)
  if (!mode_tx and changed_f)
  {
    volatile uint32_t f; 
    bool bfo_change = false; 
    
    f = VFOSet[v].vfo; 
#if defined(SS_FAT5_160AM_TX) || defined(SS_VK3SJ_160AM_TX) || defined (SS_VK3SJ_40AM_TX) || defined(SS_VK3SJ_160AM_PORTABLE_TX)
    // these transmitters have no receiver so do nothing
#else
    si5351.set_freq((bfo + f) * SI5351_FREQ_MULT, SI5351_CLK0);  
#endif

#ifdef SS_VK3SJ_160AM_PORTABLE_TX
    si5351.set_freq((bfo + f) * SI5351_FREQ_MULT, SI5351_CLK2);  // this project has an AM receiver so set the VFO on CLK2
#endif 

    // BFO logic...
    if (f >= SIDEBAND_THRESHOLD )  // f >= 10MHz so use USB 
    {
      if(bfo == LSB) // need to change BFO
      {
        bfo = USB;
        bfo_change = true; 
      }
    }
    else  // f < 10MHz 
    {
      if(bfo == USB) // need to change BFO
      {
        bfo = LSB;
        bfo_change = true; 
      }
    };
    if(bfo_change || BFO_tune_flg) 
    {
      // for rigs with an external crystal BFO, need to send a pin high to flip the crystal BFO
      // ...
#ifdef BFO_ENABLED
      si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK2); 
#endif
    }

    // make sure the right BPF/LPF filters are selected
    set_filters(f);
    
    //refresh_display();
    changed_f = false;
    last_freq_change_ms = millis(); 
    eeprom_written_since_last_freq_change = false;
  } // endif changed_f

#ifdef SP_X
  SP_X_counter++;
  if(SP_X_counter > 20){
    SP_X_counter=0;
//  if(SP_X_init){
    Serial.println("SP_X_init");
//      digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // un-mute the receiver
      // let the series speaker relay drop out (this silences a switch-on thump)
//      delay(100);
//      digitalWrite(SP_X_MUTE_SPKR, LOW);  // mute the speaker line
//            delay(200);
//      digitalWrite(SP_X_MUTE_SPKR, HIGH);  // un-mute the speaker line


//      digitalWrite(RX_MUTE_LINE, RX_MUTE_ON_VALUE);     // mute the receiver
//      delay(400);
//      digitalWrite(RX_MUTE_LINE, RX_MUTE_OFF_VALUE);     // un-mute the receiver


      SP_X_init = false;
  }
#endif

  // sense the PTT switch, paddle and key lines
  if(!mode_cw)  // CW manages T/R for itself
  {
    if(!digitalRead(PTT_SENSE) && !mode_tx) 
    {
      receive_to_TRANSMIT(); // PTT button pressed
      //refresh_display();
    };
//    delay(10);
    if(digitalRead(PTT_SENSE)  &&  mode_tx) 
    {
      TRANSMIT_to_receive(); // PTT button released
      //refresh_display();
    }
  }

  // see if the rig has been idle long enough to drop out the LPF relay...
  if( (!mode_tx) && (millis() - last_T_R_ms) > LPF_DROPOUT_DELAY )
  {
//    Serial.print("-drop LPF");     Serial.println(millis()); 
    for (byte i=0; i<8; i++) PCF_LPF.write(i, 0); // easy way (just turn them all off) 
    LPF_engaged = false;
    last_T_R_ms = millis(); 

    // stop the fan...
    
  }

  
  //------------------------------------------------------------------------
  // if any of the buttons have been pressed...
  old_button_nbr = 0;  // clear the last command 
  bool button_held = false; 
  unsigned long button_pressed_ms = millis();
  
  button_nbr = get_front_panel_button();
  byte first_button_nbr = 0;
  while (button_nbr > 0)
  {
  // one of the multiplexed switches is being held down
    delay(5);  // was 20
    if(first_button_nbr == 0) first_button_nbr = button_nbr;
    old_button_nbr = button_nbr;
    button_nbr = get_front_panel_button();
  }

  button_nbr = first_button_nbr; // experiment to accept the first reading
  
  if((millis() - button_pressed_ms) >= BUTTON_HELD_MS) button_held = true;
    
 // if one of the buttons was pressed (and is now released) act on it...
 
  if (button_nbr == 1)
  {
      if(!func_button_pressed)
      {
          Serial.println("<B1>VFO down");
          if(v == 0) v = (NBR_VFOS-1);
          else v--;
      }
      else
      {
          Serial.println("<F><B1>N/A");
          func_button_pressed = false;
      };
    changed_f = true;
  };

  if (button_nbr == 2) 
  {
    // Button 2:  
  }; 

  if (button_nbr == 3) {};  

  if (button_nbr == 4)
  {
      if(!func_button_pressed)
      {
             if(button_held) 
             {
               Serial.println("<B4>held-Tune"); 
               button_held = false;
               // default behaviour is tune
               mode_tune = true;
               tune();               
             }
             else
             {
               Serial.println("<B4>VFO up"); 
//               Serial.print("B4 v="); Serial.print(v); Serial.print(" VFO="); Serial.println(VFOSet[v].vfo);
               int v_prev = v; 
               if(v == (NBR_VFOS-1)) v = 0; else v++;
//               Serial.print("Aft v="); Serial.print(v); Serial.print(" VFO="); Serial.println(VFOSet[v].vfo);
             }
      }
      else
      {
         // BFO Tune toggle
         Serial.println("<F><B4>BFOTune");
         BFO_tune_flg = !BFO_tune_flg;
         func_button_pressed = false;
       }
    changed_f = true;
  }

  if (button_nbr == 5) 
  {
    Serial.println("<B5>Fn tgl");
    func_button_pressed = !func_button_pressed;   
    if(func_button_pressed) Serial.println("Function...");
    changed_f = true;
  }

  if (button_nbr == 6)
  // Button 6: change frequency step up
  // Fn: tbd
  {
    if(!func_button_pressed)
       Serial.println("<B6>f step r");
    else
       Serial.println("<F><B6>f step l");

    if(button_held)
    {
      Serial.println("<B6>held -- toggle IF filters"); 
    }
    else
    {
#if defined(SP_V) or defined (SP_6) or defined (SP_8) or defined (SP_9) or defined (SP_11)
      switch (VFOSet[v].radix)
      {
        case 10:
        {
           VFOSet[v].radix = 100;
        }
        break;
        case 100:
        {
           VFOSet[v].radix = 1000;
           // clear residual < 1kHz frequency component from the active VFO         
           uint16_t f = VFOSet[v].vfo % 1000;
           VFOSet[v].vfo -= f;
        }
        break;
        case 1000:
        {
           VFOSet[v].radix = 100;
        }
        break;
      
        case 10000:
        {
           VFOSet[v].radix = 1000;
        }
        break;
      } 
#else
      // default radfix increment/decrement behaviour...
      switch (VFOSet[v].radix)
      {
        case 10:
        {
            if(!func_button_pressed)
            {
                // change radix up
                VFOSet[v].radix = 10000;
                // clear residual < 1kHz frequency component from the active VFO         
                uint16_t f = VFOSet[v].vfo % 1000;
                VFOSet[v].vfo -= f;
            }
            else
            {
               func_button_pressed = false;
               // change radix down
               VFOSet[v].radix = 100;
               // clear residual < 100Hz frequency component from the active VFO         
               uint16_t f = VFOSet[v].vfo % 100;
               VFOSet[v].vfo -= f;
            }
        }
        break;
  
        case 100:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 10;
            }
            else
            {
                func_button_pressed = false;
                VFOSet[v].radix = 1000;
                // clear residual < 1kHz frequency component from the active VFO         
                uint16_t f = VFOSet[v].vfo % 1000;
                VFOSet[v].vfo -= f;
            }
        }
        break;
        
        case 1000:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 100;
            }
            else
            {
               func_button_pressed = false;
               VFOSet[v].radix = 10000;            
            }
            break;
        }
      
        case 10000:
        {
            if(!func_button_pressed)
            {
                VFOSet[v].radix = 1000;
            }
            else
            {
                func_button_pressed = false;
                VFOSet[v].radix = 10;
            }
            break;
        }
      }
#endif
    } // else
    changed_f = true;
  }

#ifdef CW_KEYER
// start of CW Keyer block -----------------------------------------------------------
//  read_keyer_speed(); // read the speed control 
// see if a memory button has been pushed
  if((curr_msg_nbr = check_keyer_pushbutton()) > 0)
  {
    mode_cw = true; 
    activate_state2('T'); 
    refresh_display();  // to show 'CW' on the display  
    byte msg_speed=0;
#if defined(SP_IV) || defined(SP_V) || defined(SP_6) || defined(SP_7)  || defined(SP_8) || defined(SP_9) || defined(SP_X) || defined(UNIVERSAL_VFO_CONTROLLER)
    msg_speed = KEYER_MSG_SPEED;   // this plays the canned keyer messages faster than the paddle, to save time on cold summits
#endif
    play_message(morse_msg[curr_msg_nbr-1], msg_speed);  
    delay(5); 
    activate_state2('R');  
  };

  // see if the paddle is being pressed
  byte j = check_paddle();
  if((j==PADDLE_L) or (j==PADDLE_R)) 
  {
    mode_cw = true;
    activate_state2('T'); 
#ifdef DISPLAY_OLED 
#ifndef SP_X
    if(dot_dash_sent == 3) refresh_pwr_meter();  // only want to do this first time, as I2C OLED refresh is *slow*
#endif 
#endif 
    if(j==PADDLE_L) send_dot();
    if(j==PADDLE_R) send_dash();
 //   Serial.print("loop()::dot_length_ms:");   Serial.println(dot_length_ms);
  };

  curr_ms = millis();
  // if paddle has been idle for BREAK_IN_DELAY drop out of transmit 
  if(mode_cw and mode_tx and ((curr_ms - char_sent_ms) > BREAK_IN_DELAY))
  {
//    Serial.print("curr_ms="); Serial.print(curr_ms); 
//    Serial.print("  char_sent_ms="); Serial.print(char_sent_ms); 
//    Serial.print("  curr_ms - char_sent_ms="); Serial.println(curr_ms - char_sent_ms); 
    // drop back to receive to implement break-in
    activate_state2('R');  
  }
// end of CW Keyer block -------------------------------------------------------------
#endif

#ifdef VSWR_METER
  //-- VSWR meter code begins -------------------------------------------
  int n; 
//  n = read_analogue_pin(PIN_SWR_FWD);

  n = analogRead(PIN_SWR_FWD);
//  Serial.print("      fwd=");   Serial.println(n);
  if(n > fwd_max) 
  {
    fwd_max = n;     
//    Serial.print("     F=");   Serial.println(fwd_max);
  }
  n = analogRead(PIN_SWR_REV);
//  Serial.print("      rev=");   Serial.println(n);
  if(n > rev_max) 
  {
    rev_max = n;
//    Serial.print("     R=");   Serial.println(rev_max);
  }
  //-- VSWR meter code ends ---------------------------------------------
#endif


} // end of loop


