/*
  CPE 301 - Final Project
  Authors: Nicholas Finne, Domminic Mayer, Jake Herweg
*/

#include <LiquidCrystal.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Stepper.h>
#include "RTClib.h"

#define DHTPIN 2
#define BUTTON_PIN 3
#define LCD_RS 4
#define LCD_EN 5
#define LCD_D4 44
#define LCD_D5 45
#define LCD_D6 46
#define LCD_D7 47
#define STEPPER_PIN1 6
#define STEPPER_PIN2 7
#define STEPPER_PIN3 8
#define STEPPER_PIN4 9
#define FAN_PIN 10
#define LIGHT_R 24
#define LIGHT_G 25
#define LIGHT_B 26
#define POT_PIN A3

#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHTPIN, 11);
Stepper stepper(32, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

RTC_DS1307 DS1307_RTC;

const int revSteps = 100;
char Week_days[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
int waterLevel, state = 0;
float tempLevel, humiLevel;
bool error, TWMU, prevState;
volatile int buttonState = LOW;

const int STEPS_PER_REV = 2048;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;
const float DEGREES_PER_STEP = 360.0 / STEPS_PER_REV;
const float STEPS_PER_INPUT_VALUE = 1.0;

void buttonPressed() {
  Serial.println("Button Pressed");
  buttonState = !buttonState;  // toggle the button state
  if (!(buttonState)) {
    stated();
  }
  //Serial.println(buttonState); // print the button state
}

void setup() {
  // put your setup code here, to run once:
  setupPins();
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);
  LCD();
  U0init(9600); //Same as Serial.begin()
  adc_init();

  stepper.setSpeed(200);

  error = false; // Can't start in error
  buttonState = 0;
  //Setting up RTC
  if(! DS1307_RTC.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  DS1307_RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  TWMU = true;
  TWM();
  state = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  printTime();
  //TWM();
  digitalWrite(FAN_PIN, HIGH);
  if(buttonState == 1) {
    if (waterLevel < 100) {
      Serial.println("System: Water Level Too Low");
      lcdE();
      state = 0;
      disabled();
      char str[] = {'E','r','r','o','r', '-', 'W', 'a', 't', 'e', 'r', ' ', 'L', 'o', 'w', '\n'};
      for (int i = 0; str[i] != '\0'; i++) {
        U0putchar(str[i]);
      }
      //digitalWrite(LIGHT_R, HIGH);
      //digitalWrite(LIGHT_B, LOW);
      //digitalWrite(LIGHT_G, LOW);
      error = true;
    } else {
      error = false;
    }


      if (tempLevel > 80) {
        running();
      } else {
        idled();
      }
  }  

  if(analogRead(POT_PIN) > 100){
    // Rotate CW slowly at 5 RPM
    stepper.setSpeed(10);
    stepper.step(2038);
    delay(1000);
  }
}

void disabled(){
  state = 3;
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_R, HIGH);
  digitalWrite(LIGHT_B, LOW);
  digitalWrite(LIGHT_G, LOW);
  TWMU = false;
}

void idled(){
  state = 2;
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_R, LOW);
  digitalWrite(LIGHT_B, HIGH);
  digitalWrite(LIGHT_G, HIGH);
  TWMU = true;
}

void printTime(){
  DateTime now = DS1307_RTC.now();  //Gets current time
  if (TWMU) {
    if (now.second() == 0) {
        Serial.print("Updating Time and Water levels.");
        TWM();
    }
  }
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(" - ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(" - ");
  Serial.print(" state: ");
  Serial.print(state);
  Serial.print(" - ");
  Serial.print(" temp: ");
  Serial.print(tempLevel);
  Serial.print(" - ");
  Serial.print(" humi: ");
  Serial.print(humiLevel);
  Serial.print(" - ");
  Serial.print(" water: ");
  Serial.print(waterLevel);
  Serial.println();
}
void TWM(){ // Temperature and Water Monitoring Function
  waterLevel = adc_read(2);
  humiLevel = dht.readHumidity();
  tempLevel = dht.readTemperature(true);

  if (isnan(humiLevel) && (tempLevel)) {  
    lcdE();
  }
  //Prints out the temperature and humidity 
  lcd.setCursor(7,0);
  lcd.print(tempLevel);
  lcd.setCursor(7,1);
  lcd.print(humiLevel);
}
void setupPins() {
  // Sets all the pins to the correct mode
  DDRD &= ~(1 << DHTPIN); // DHTPIN as input
  DDRD |= (1 << FAN_PIN); // FAN as output
  DDRD |= (1 << LCD_RS); // LCD_RS as output
  DDRD |= (1 << LCD_EN); // LCD_EN as output
  DDRB |= (1 << LCD_D4); // LCD_D4 as output
  DDRB |= (1 << LCD_D5); // LCD_D5 as output
  DDRB |= (1 << LCD_D6); // LCD_D6 as output
  DDRB |= (1 << LCD_D7); // LCD_D7 as output
  DDRD |= (1 << STEPPER_PIN1); // STEPPER_PIN1 as output
  DDRD |= (1 << STEPPER_PIN2); // STEPPER_PIN2 as output
  DDRD |= (1 << STEPPER_PIN3); // STEPPER_PIN3 as output
  DDRB |= (1 << STEPPER_PIN4); // STEPPER_PIN4 as output
  DDRC &= ~(1 << BUTTON_PIN); // BUTTON_PIN as input
  DDRB |= (1 << LIGHT_R); // LIGHT_R as output
  DDRB |= (1 << LIGHT_G); // LIGHT_G as output
  DDRB |= (1 << LIGHT_B); // LIGHT_B as output
}
void LCD() {
    lcd.begin(16, 2);
    lcd.setCursor(7,1);
    lcd.print("xxx");
    dht.begin();
    DDRB |= (1 << 5);
    lcd.setCursor(0,0);
    lcd.print("Temp:");
    lcd.setCursor(0,1);
    lcd.print("Humi:");
}
void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
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
 *myUBRR0  = tbaud;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
/*
void stepperButton() {
	lastButtonState = buttonState;
	buttonState = digitalRead(BUTTON_PIN);
	
	if(lastButtonState == LOW && buttonState == HIGH) {
		stepperOpen();
    Serial.println("Button Pressed");
	} else if(lastButtonState == HIGH && buttonState == LOW) {
		stepperClose();
    Serial.println("Button Pressed");
	}
}
*/

void running() {
  state = 1;
  digitalWrite(LIGHT_R, LOW);
  digitalWrite(LIGHT_B, HIGH);
  digitalWrite(LIGHT_G, LOW);
  digitalWrite(FAN_PIN, HIGH);
  TWMU = true;
}

void stated() {
  if(state == 0) {
      //errored
      disabled();
    } else if (state == 1) {
        //running
        disabled();
    } else if (state == 2) {
      //idling
      disabled();
    } else if (state == 3) {
      //disabled
      idled();
    } else {
      disabled();
    }
}

void lcdE(){
  //Creates an error message if the sensor aren't responding
    lcd.clear();
    lcd.setCursor(4,1);
    lcd.print("|SENSOR|");
    lcd.setCursor(4,0);
    lcd.print("|ERROR |");  
    error = true;
    return;
}
