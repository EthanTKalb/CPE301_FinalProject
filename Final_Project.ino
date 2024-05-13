// Libraries
#include <LiquidCrystal.h>
#include <dht.h>
#include <Stepper.h>
#include "RTClib.h"
#include <Wire.h>


LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

//Water Monitor
volatile unsigned char *port_K = (unsigned char *)0x108;
volatile unsigned char *ddr_K = (unsigned char *)0x107;
volatile unsigned char *pin_K = (unsigned char *)0x106;

//DHT Sensor

//Analog Read
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

//buttons (on/off)
volatile unsigned char* port_j = (unsigned char*) 0x105; 
volatile unsigned char* ddr_j  = (unsigned char*) 0x104; 
volatile unsigned char* pin_j  = (unsigned char*) 0x103;

volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29;

volatile unsigned char* my_PCMSK1 = (unsigned char*) 0x6C; 
volatile unsigned char* my_PCICR  = (unsigned char*) 0x68; 

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23;


//LED Outputs 22,24,26,28
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20;

#define RDA 0x80
#define TBE 0x20
#define DHT11_ANA 12
#define STEPS 32


int WaterMonitor = 0;
dht DHT;
int previousPot = 0;


bool start = false;

  Stepper stepper = Stepper(STEPS, 8, 10, 9, 11);

RTC_DS1307 rtc;
unsigned int minute = 0;
DateTime now;
String state = "disabled";



void setup() {
     U0init(9600);

  // put your setup code here, to run once:
  *ddr_K &= 0b0001000;

//Input for buttons
  *ddr_j &= 0b111111101;
  *port_j |= 0b00000010;

  *ddr_j &= 0b111111110;
  *port_j |= 0b00000001;


  *ddr_d &= 0b111111011;
  *port_d |= 0b00000100;

  *ddr_d &= 0b111110111;
  *port_d |= 0b00001000;

  *ddr_b &= 0b111111101;
  *port_b |= 0b00000010;



  //Outputs for LEDS
  *ddr_a |= 0b00000001;
  *ddr_a |= 0b00000100;
  *ddr_a |= 0b00010000;
  *ddr_a |= 0b01000000;

  //Other

  attachInterrupt(digitalPinToInterrupt(18), toggleStart, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), reset, FALLING);

  lcd.begin(15, 2);

  adc_init();

  stepper.setSpeed(200);

    while (! rtc.begin()) 
    {
        U0putchar("Couldn't find RTC");
    }
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


//LEDS
    *port_a |= 0b00000001; //YELLOW ON

    printing("  State = Disabled");

}

void loop() {
  //RTC
      now = rtc.now();

    //DHT Sensor

    int DHTchk = DHT.read11(DHT11_ANA);


  //On-Off buttons
  if (!start)
  {
        return;
  }

  if (state != "disabled")
  {

//LCD screen for temp/humidity
if (state == "error")
{
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ERROR");

}
else
{
  //if (true)                 //For video demonstration purposes RTC module recorded vent & state every second instead of every minute.
if (minute != now.minute())
      {
        minute = now.minute();
        
      
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Air Temp:       ");
lcd.setCursor(0,1);
lcd.print("Air hum:   ");

lcd.setCursor(14, 0);
lcd.print(DHT.temperature);
lcd.setCursor(14, 1);
lcd.print(DHT.humidity);
      }
if (DHT.temperature > 28.0f)
{
  printing("  State Running");
  state = "running";
}
}

//Vent / stepper motor
int pot = map(adc_read(6),0,1024,0,500);
if (abs(pot - previousPot) >= 30)
{
    stepper.step(pot - previousPot);
    previousPot = pot;
    printing("    Vent changed");
}

}

  if (state == "idle")
  {
    //Water Monitor
    WaterMonitor = adc_read(0);
     analogWrite(A4, 0);


    if (WaterMonitor <= 200) 
    {
     // printing("     Water is too low");

     printing("  State = Error");

     state = "error";
    *port_a &= 0b10101010; //ALL OFF
    *port_a |= 0b00010000; //RED ON
    }
  }

  if (state == "error")
  {
      analogWrite(A4, 0);
  }


  if (state == "running")
  {
    analogWrite(A4, 150);

    *port_a &= 0b10101010; //ALL OFF
    *port_a |= 0b01000000; //BLUE ON
    if (DHT.temperature <= 28.0f)
    {
        printing("  State = Idle");
      state = "idle";
      analogWrite(A4, 0);
      *port_a &= 0b10111111; //BLUE OFF
      *port_a |= 0b00000100; //GREEN ON
    }
    WaterMonitor = adc_read(0);

    if (WaterMonitor <= 200) 
    {
      analogWrite(A4, 0);
        printing("  State = Error");

     state = "error";
    *port_a &= 0b10101010; //ALL OFF
    *port_a |= 0b00010000; //RED ON
    }
  }
}

void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000;  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111;  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111;  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000;  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111;  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000;  // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX &= 0b01111111;  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000;  // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111;  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000;  // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7) {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0)
    ;
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}
unsigned char U0kbhit() {
  return *myUCSR0A & RDA;
}
unsigned char U0getchar() {
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE) == 0)
    ;
  *myUDR0 = U0pdata;
}

void toggleStart()
{
  start = !start;
  if (start && state == "disabled")
  {
      printing("  State = Idle");

    state = "idle";
    
    *port_a &= 0b11111110; //YELLOW OFF
    *port_a |= 0b00000100; //GREEN ON
  }
  else if (!start)
  {
      printing("  State = Disabled");

    state = "disabled";
    *port_a &= 0b10101010; //ALL OFF
    *port_a |= 0b00000001; //YELLOW ON

    analogWrite(A4, 0);
  }
}

void reset()
{
  if (state == "error")
  {
    WaterMonitor = adc_read(0);

    if (WaterMonitor > 200) 
    {
        printing("  State = Idle");

      state = "idle";
      *port_a &= 0b11101111; //RED OFF
      *port_a |= 0b00000100; //GREEN ON

    }
  }
}

void printing(String s)
{
 String Printing = String(now.year());
    for(int i = 0; i < Printing.length(); i++)
    {
      U0putchar(Printing[i]);
    }
    U0putchar('-');
    Printing = now.month();
    for(int i = 0; i < Printing.length(); i++)
    {
      U0putchar(Printing[i]);
    }
    U0putchar('-');
    Printing = now.day();
    for(int i = 0; i < Printing.length(); i++)
    {
      U0putchar(Printing[i]);
    }
    U0putchar(' ');
    Printing = now.hour();
    for(int i = 0; i < Printing.length(); i++)
    {
      U0putchar(Printing[i]);
    }
    U0putchar(':');
    Printing = now.minute();
    for(int i = 0; i < Printing.length(); i++)
    {
      U0putchar(Printing[i]);
    }
    U0putchar(':');
    Printing = now.second();
    for(int i = 0; i < Printing.length(); i++)
    {
      U0putchar(Printing[i]);
    }
    s += "\n";    
    for(int i = 0; i < s.length(); i++)
    {
        U0putchar(s[i]);
    }

}
