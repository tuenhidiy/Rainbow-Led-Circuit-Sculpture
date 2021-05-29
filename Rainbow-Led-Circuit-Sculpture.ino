/*
 * 
Rainbow Led Circuit Sculpture project descriptions:
This sculpture circuit controls 16 RGB leds using DigiSpark ATtiny85. We can adjust their colors separately according to the colorwheel rule with a touch button.
Tested with Arduino 1.8.13, the ATTinyCore and tinySPI library:

  https://github.com/SpenceKonde/ATTinyCore
  https://github.com/JChristensen/tinySPI

Connections:
  - DigiSpark ATtiny85 P0 (PB0) - LATCH PIN OF TPIC6B595 & 74HC595
  - DigiSpark ATtiny85 P1 (PB1) - DATA PIN OF TPIC6B595
  - DigiSpark ATtiny85 P2 (PB2) - CLOCK PIN OF 74HC595 & TPIC6B595.
  - DigiSpark ATtiny85 P3 (PB3) - BLANK PIN OF 74HC595 & TPIC6B595.
  - DigiSpark ATtiny85 P4 (PB4) - TOUCH BUTTON PIN.
By TUENHIDIY - 2021. May. 29
 * 
*/

#include <tinySPI.h>            // https://github.com/JChristensen/tinySPI
#define HARDWARE_SPI 1          // set to 1 to use hardware SPI, set to 0 to use software SPI

//****************************************** Digispark ATTINY85 pin definitions*******************************************//

const int
    LATCH_PIN(0),               // Storage register clock (Latch pin)
    DATA_PIN(1),                // Data in
    CLOCK_PIN(2),               // Shift register clock (Clock pin)
    BLANK_PIN(3),               // Enable output pin
    BUTTON(4);                  // Touch button pin

//************************************************BAM Variables************************************************************//
byte anode[8]= {B11111110, B11111101, B11111011, B11110111, B11101111, B11011111, B10111111, B01111111};
byte RRGGBB[4][8];
int row=0;
int BAM_Bit, BAM_Counter=0; 
struct Color
{
  unsigned char red, green, blue;
  Color(int r, int g, int b) : red(r), green(g) , blue(b){}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color tealcolor       = Color(0x00, 0x0F, 0x04);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);
const Color purplecolor     = Color(0x0F, 0x00, 0x0F);
const Color whitecolor      = Color(0x0F, 0x0F, 0x0F);
const Color blackcolor      = Color(0x00, 0x00, 0x00);

//***********************************************ColorWheel Variables*****************************************************//

#define BAM_RESOLUTION 4  
#define COLOUR_WHEEL_LENGTH 128

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473

//*********************************************Touch Button Variables***************************************************//

unsigned long samplingtime  = 0;
int buttonPushCounter = 0;   
int buttonState = 0;         
int lastButtonState = 0; 
int touchCounter = 0;

void setup()
{
  row = 0;  
  #if HARDWARE_SPI == 1
    SPI.begin();                   // Start hardware SPI.
  #else
    pinMode(CLOCK_PIN, OUTPUT);    // Set up the pins for software SPI
    pinMode(DATA_PIN, OUTPUT);
  #endif
  pinMode(LATCH_PIN, OUTPUT);     // Latch pin needs to be set up for hardware or software SPI
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(LATCH_PIN, HIGH);
  noInterrupts();
  // Clear registers
  TCNT1 = 0;
  TCCR1 = 0;
  // 15 x 3.636 = 54.54us
  OCR1C = 15;
  // interrupt COMPA
  OCR1A = OCR1C;
  // CTC
  TCCR1 |= (1 << CTC1);
  // Prescaler 64 - 16.5MHz/64 = 275Kz or 3,636us
  TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK |= (1 << OCIE1A);
  interrupts();
  clearfast();
  // Start ColorWheel function
  fill_colour_wheel();
}

void loop()
{
  buttonState = digitalRead(BUTTON);
  if (buttonState != lastButtonState)
  {
    if (buttonState == HIGH)
    {
      buttonPushCounter++;
    }
    else 
    {

    }
  }
  lastButtonState = buttonState;
  touchCounter = buttonPushCounter % 127;
  for (byte x=0; x<2; x++)
    {      
      for (byte z=0; z<8; z++)
        {

          get_colour(7*(touchCounter + z + 3*x), &R, &G, &B);
          LED(x, z, R, G, B);
        }
      }    
}
void LED(int X, int Z, int R, int G, int B)
{
  X = constrain(X, 0, 1);
  Z = constrain(Z, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);
 
  byte WhichBitRed[2]  = {1, 6};
  byte WhichBitGreen[2]= {2, 5};
  byte WhichBitBlue[2] = {3, 4};

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(RRGGBB[BAM][Z], WhichBitRed[X], bitRead(R, BAM));   
    bitWrite(RRGGBB[BAM][Z], WhichBitGreen[X], bitRead(G, BAM));
    bitWrite(RRGGBB[BAM][Z], WhichBitBlue[X], bitRead(B, BAM));
  } 
}

void clearfast()
{
   memset(RRGGBB, 0, sizeof(RRGGBB[0][0]) * 4 * 8);     
}

ISR(TIMER1_COMPA_vect)
{ 
PORTB |= ((1<<BLANK_PIN));  // Set BLANK PIN (OE) high - TPIC6B595N
if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

DIY_SPI(anode[row]);    // Send out the anode level to 74HC595

switch (BAM_Bit)
{
    case 0:
      DIY_SPI(RRGGBB[0][row]); 
      break;
    case 1: 
      DIY_SPI(RRGGBB[1][row]);                
      break;
    case 2:       
      DIY_SPI(RRGGBB[2][row]);    
      break;
    case 3:
      DIY_SPI(RRGGBB[3][row]);          
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}
 
PORTB &= ~(1<<LATCH_PIN);   // Set LATCH PIN low - TPIC6B595N           
delayMicroseconds(3);  
PORTB |= 1<<LATCH_PIN;      // Set LATCH PIN high - TPIC6B595N
delayMicroseconds(3);
PORTB &= ~(1<<BLANK_PIN);   // Set BLANK PIN low - TPIC6B595N   

row++;
if(row==8) row=0;
pinMode(BLANK_PIN, OUTPUT);
}

void DIY_SPI(uint8_t DATA)
{     
    uint8_t i;
    #if HARDWARE_SPI == 1
        SPI.transfer(DATA);
    #else
    for (i = 0; i<8; i++)  
    {
      digitalWrite(DATA_PIN, !!(DATA & (1 << (7-i))));
      PORTB |= 1<<CLOCK_PIN;
      PORTB &= ~(1<<CLOCK_PIN);                
    }
    #endif
}

//************************************************************************************************************//

//  FAST SINE APPROX
float mySin(float x)
{
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI)
  {
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI))
  {
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//  FAST COSINE APPROX
float myCos(float x)
{
  return mySin(x+myPI/2);
}

float myTan(float x)
{
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in)
{
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1)
  {
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++)
  {
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++)
  {
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//ABSOLUTE VALUE
float myAbs(float in)
{
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}
