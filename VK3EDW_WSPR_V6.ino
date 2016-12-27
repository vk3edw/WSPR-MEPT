/*
This code allows the transmission of a Type 1 WSPR message on a programmed freuency.

I have used the Si5351 library developed by Jason Milldrum NT7S as it provides
simple control of the Si5351 sig gen. I have used the adafruit si5351 breakout board.

The JT Encode Library is also written by Jason Milldrum NT7S and I have cut  down the sketch 
to use WSPR only.

I have used the V.Kel VK16E gps module for accurate timing reference and used the
Tinygps++ library written by Mikal Hart (sundial.com).

The PCD8544 library is written by Carlos Rodrigues <cefrodrigues@gmail.com> and 
out of many libraries tested gives the best performance with minimal memory usage.

I have written the code to be timed by the gps module and the selected transmission
times are programmed within the code. You can modify transmission frequency
in the code, the calibration factor for your si5351, your Callsign,
TX output power and transmission times.

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject
 to the following conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>
#include <Wire.h>
#include <PCD8544.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


#define WSPR_TONE_SPACING       146           // ~1.46 Hz

#define WSPR_CTC                10672         // CTC value for WSPR

#define WSPR_DEFAULT_FREQ       7040164ULL   //this is wspr transmit frequency ENTER THE FREQUENCY YOU WANT
                                             //TO TRANSMIT ON


#define DEFAULT_MODE            MODE_WSPR

// Hardware defines

#define LED_PIN                 13

unsigned long long int frequencystandby = 000010000 ; // here you set the MEPT standby frequency, 
                                                      //this gives a smooth start to transmit sequence.


// Enumerations
enum mode { MODE_WSPR };


Si5351 si5351;
JTEncode jtencode;
static PCD8544 lcd;// The lcd display

SoftwareSerial ss(9, 8);// The serial connection to GPS
int calvalue = 4700;// here you set the cal value for the SI5351 This is determined using the
                    //calibration sketch


// The TinyGPS++
TinyGPSPlus gps;


// Global variables
unsigned long freq;
char message[] = "VK3EDW QF22";//enter your call and locator
char call[] = "VK3EDW";//enter your callsign
char loc[] = "QF22";//enter your locator
uint8_t dbm = 30;//enter your transmit power in db
uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t ctc, tone_spacing;

// Global variables used in ISRs
volatile bool proceed = false;

// Timer interrupt vector.  This toggles the variable we use to gate
// each column of output to ensure accurate timing.  Called whenever
// Timer1 hits the count set below in setup().
ISR(TIMER1_COMPA_vect)
{
  proceed = true;
}

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  // Clear out the old transmit buffer
  memset(tx_buffer, 0, 255);
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
 
  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(LED_PIN, HIGH);
  
    for (i = 0; i < symbol_count; i++)
    {
      si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), 0, SI5351_CLK0);
      proceed = false;
      while (!proceed);
    }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(LED_PIN, LOW);
  si5351.set_freq(frequencystandby, 0, SI5351_CLK0);    //set si5351 to standby frequency
  si5351.output_enable(SI5351_CLK0, 1);                //Turn on si5351 at standby frequency

  (gps.encode(ss.read()));// do a GPS read
}


void setup()
{
  lcd.begin(84, 48);    //establish LCD display size

  ss.begin(9600);// Begin the gps connection

  
  pinMode(LED_PIN, OUTPUT);// Use the Arduino's on-board LED as a keying indicator.
  digitalWrite(LED_PIN, LOW);//set it low
  
  cur_mode = MODE_WSPR;// Set the mode to use
  freq = WSPR_DEFAULT_FREQ;
  ctc = WSPR_CTC;
  symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
  tone_spacing = 146;
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired this is 8MA
  si5351.set_correction(calvalue);//set the si5351 calvalue
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // Set up Timer1 for interrupts every symbol period.
  (gps.encode(ss.read()));

  noInterrupts();          // Turn off interrupts.
  TCCR1A = 0;              // Set entire TCCR1A register to 0; disconnects
                           //   interrupt output pins, sets normal waveform
                           //   mode.  We're just using Timer1 as a counter.
  TCNT1  = 0;              // Initialize counter value to 0.
  TCCR1B = (1 << CS12) |   // Set CS12 and CS10 bit to set prescale
           (1 << CS10) |   //to /1024
           (1 << WGM12);   //turn on CTC
                           //   which gives, 64 us ticks
  TIMSK1 = (1 << OCIE1A);  // Enable timer compare interrupt.
  OCR1A = ctc;             // Set up interrupt trigger count;
  interrupts();            // Re-enable interrupts.
}

void displayInfo()        //LCD display of GPS UTC time and number of satellites being used
{
  if (gps.time.isValid())
    lcd.setCursor(0, 0);
  if (gps.time.hour() < 10)lcd.print (F("0"));
  lcd.print(gps.time.hour());
  lcd.print(F(":"));
  if (gps.time.minute() < 10) lcd.print (F("0"));
  lcd.print(gps.time.minute());
  lcd.print(F(":"));
  if (gps.time.second() < 10) lcd.print (F("0"));
  lcd.print(gps.time.second());
  lcd.print(F(" UT "));
  lcd.setCursor(70, 0);
  lcd.print(gps.satellites.value());
  gpsgatetime ();
}

void gpsgatetime ()        //setup of transmission times at 0,7,14,21,24,32,35
                           //44,46,50 and 55 minutes of each hour at exact zero seconds
{

  if (gps.time.minute() == 0 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 7 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 14 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 21 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 24 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 32 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 35 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 44 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 46 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 50 && gps.time.second() == 0)
    encode();
  if (gps.time.minute() == 55 && gps.time.second() == 0)
    encode();
}


void loop()             //loop through checking GPS and goto 'displayinfo()' to do transmissions.
{

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  
}
