/*
ROSSOPOMODORO -- ATTINY85

led? -- |  U  | +V
rot1 -- |     |  -- SCL
rot2 -- |     | -- btn, pb1
    gnd |_____| -- SDA


SSD1306  128x32 I^2C oled displ.

DISP:
|----------             ------|
|  STUDY      PAUSE      GO!  |
|  25 min     5 min      GO!  |
|_____________________________|

Uses bitflags on byte variable "state" to keep track of the program state
http://www.learncpp.com/cpp-tutorial/3-8a-bit-flags-and-bit-masks/

Uses internal 1mhz osc and 8192 clock divider to time interrupt 999.42ms. 
Off by one sec per ~1724 sec.

*/

#include "SSD1306_minimal.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>


#define shortPress 40
#define longPress 300

SSD1306_Mini oled;
volatile byte encoded = 0;  // rotary encoder tracker
volatile int teljari = 0;   // timateljari

byte study = 25;   // sel if state & 64
byte pause = 5;    // sel if state & 32. if state & (64+32) => sel GO

int lastEncoded = 0;
int value = 0;

//  1=countdown enable, 2=updateScreen, 4=buttonState, 8=buttonDown, 16=pinChangeIntrupt
// 32= pause hover, 64=study hover, 128= hover is selected
volatile byte state = 0; 

unsigned long int buttonHoldTime, buttonPressTime;
  
static inline void initTimer1(void)  // set to trigger per ~1 sec
{
  TCCR1 |= (1 << CTC1);  // clear timer on compare match
  TCCR1 |= (1 << CS13) | (1 << CS12) | (1 << CS11); //clock prescaler 8192
  OCR1C = 122; // compare match value: 122*8.192ms = 999.424 ms
  TIMSK |= (1 << OCIE1A); // enable compare match interrupt
}


ISR(TIMER1_COMPA_vect) // timer interrupt. gets called per sec (999.424ms) 
{
  if((state & 1) && teljari > 0){
      teljari--;
      state |= 2;   // update displ.
    }
  /* if(teljari < 1){
      state 
    }
*/
}

// pin change interrupt -> pinChangeHandler -> buttonHandler

inline void decrVal()
{
  state &= 2; // update displ flag
  
  if (!(state & 128)){ // if no sel, scroll between hovering over sel's
    if(state & 96){    // GO option hover (32+64) -> study 
      state &= ~32;
      }
    else if(state & 64) {  // study option hover -> pause
      state &= ~64;
      state |= 32;
      }
    else if(state & 32){   // pause option hover -> GO (32+64)
      state |= 64;
      }
    }

  else if(state & 128){ // if SEL, change sel values
    if(state & 64) {  // change study time
      study--;
      }
    else if(state & 32){   // change pause time
      pause--;
      }
    }
}      


inline void incrVal()
{
  state &= 2; // update displ flag
  
  if (!(state & 128)){ // if no sel, scroll between hovering over sel's
    if(state & 96){    // GO option hover (32+64) -> pause 
      state &= ~64;
      }
    else if(state & 64) {  // study option hover -> GO
      state |= 32;
      }
    else if(state & 32){   // pause option hover -> study option hover
      state &= ~32;
      state |= 64;
      }
    }
    
  else if(state & 128){ // if SEL, change sel values
    if(state & 64) {  // change study time
      study++;
      }
    else if(state & 32){   // change pause time
      pause++;
      }
    }
}

/*
long press. 
if 64 ^ 32 => save times to eeprom
if 64&32 => load times

*/

inline void longPress()
{     // study=>20, pause=> 21
  state &= 2;  // update displ

  if(state & (64 | 32)){  // GO
    study = EEPROM_read(20);
    pause = EEPROM_read(21);
  }
  else if(state & 64) 
    EEPROM_write(20, study);
  else if(state & 32) 
    EEPROM_write(21, pause);
  
 }



inline void shortpress()
{
  state &= 2;  // update displ

  if(state & (64 | 32)){  // GO
    state ^= 1;
    }

  state ^= 128;  // toggle selected/unselect on hovered 
  }

inline void pinChangeHandler()
{
  state &= digitalRead(1) << 2;               // state 4
    
  if (!(state & 8) && (state & 4)){           // has not been pressed and is down
    state &= 8 | ~4;
    buttonPressTime = millis();

  }
  else if (!(state & 8) && !(state & 4)){     // has been pressed and is up
    buttonHoldTime = millis() - buttonPressTime;
    state &= ~12;                             // clear flags 4 and 8
    delay(10);                                // software debounce, experimental
  }
  else{  
    int MSB = digitalRead(3);                 //MSB = most significant bit
    int LSB = digitalRead(4);                 //LSB = least significant bit
 
    int encoded = (MSB << 1) | LSB;           //converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded;  //adding it to the previous encoded value

     //  ++  11->01, 01->00, 00->10, 10->11  
     //  --  11->10, 01->11, 00->01, 10->00 
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011){
      incrVal();
    }
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000){
      decrVal();
    }
 
    lastEncoded = encoded;
 
    if (value <= 0) value = 0;
    if (value >= 255) value = 255;
  }
}


ISR(PCINT0_vect) // pin change interrupt, state 32
{
  state |= 16;

  // -- other func.
}
/*
void PlotText(int x, int y, const __FlashStringHelper *s)
{
  int p = (int)s;
  int page = (47 - y - yOrigin)>>3;
  while (1) {
    char c = pgm_read_byte(p++);
    if (c == 0) return;
    for (uint8_t col = 0 ; col < 6; col++) {
      Buffer[page*64 + x + xOrigin] = pgm_read_byte(&CharMap[c-32][col]);
      x++;
    }
  }
}
 */

void updateDisplay()
{
  oled.printString("STUDY");
  oled.printString("PAUSE");
  oled.printString("GO");
}

void EEPROM_write(unsigned char ucAddress, unsigned char ucData)
{                 // study=>20, pause=> 21
  cli();
  while(EECR & (1<<EEPE))  // wait if still writing
  ;
  EECR = (0<<EEPM1)|(0<<EEPM0);
  EEAR = ucAddress;
  EEDR = ucData;
  EECR |=(1<<EEMPE);
  EECR |= (1<<EEPE);
  sei();
  }

unsigned char EEPROM_read(unsigned char ucAddress)
{
  cli();
  while(EECR & (1<<EEPE))
  ;
  EEAR = ucAddress;
  EECR |= (1<<EERE);
  sei();
  return EEDR;
  }
  
/*
void printByte(byte toPrint)
{
  for(int i=0; i<8; i++){
    Serial.print(toPrint & (1<<i));
    Serial.print(" ");
    }
    Serial.println();
  } */

void setup()
{
  pinMode(1, INPUT_PULLUP);
  pinMode(3, INPUT);
  pinMode(4, INPUT); 

  byte testVal = EEPROM_read(20);   
  if((testVal > 0 || testVal < 255) && testVal != study) 
    study = testVal;
  byte testVal = EEPROM_read(21);
  if((testVal > 0 || testVal < 255) && testVal != pause) 
    pause = testVal;



  GIMSK = 0b00100000;       // Enable pin change interrupts
  // PB3 & PB4 : rot.enc, pb1: rot.enc pushB
  PCMSK = 0b00011010;       // Enable pin change interrupt for PB3, PB4 and PB1
  // bitmask for PCMSK |= (1<<PCINT4)|(1<<PCINT3)|(1<<PCINT1);
  sei();   
  

    oled.init(0x3c);
    oled.clear();
    
       
}
 
void loop()
{
  if(state & 16){
    state &= ~16;    // reset pin change flag
    pinChangeHandler();
  }

  if(buttonHoldTime > shortPress && buttonHoldTime < longPress){
      // shortpress();
      buttonHoldTime = 0;
    }
  if(buttonHoldTime >= longPress){   // button push value, execute
      // longPress();
      buttonHoldTime = 0;
    }
    
  if(state & 2){    // update display
   // updateDisplay();
    }
  
} 


/*

  a ? b : c ==  if(a) then b, else c
  

Pin change interrupts.
Short press: select next
Long press: if time selected, edit. If go selected, go.

push button pin change: 
state ^= digitalRead(pushB) << 3;  // state: 4 


if(digitalRead(pushb) && !(state & 8){ 
  pushTime = millis(); 
  state |= 8; 
}
else if

fæ pin change.
Setja status unprocessed pin change, status 32.

loop: ef status & 32
void buttonHandler(){
  status &= ~32;
  status &= digitalRead(1) << 2;  // stat 4
  
  if !(status & 8){
    status &= 8 | ~4;
    buttonPressTime = millis();
  }
  else if (stat & 8){
    buttonReleaseTime = millis();
    status &= ~12;   // clear flags 4 and 8
  }
  buttonHoldTime = buttonReleaseTime - buttonPressTime;
}

*/








