/*
  CPE 301 - Final Project
  Authors: Nicholas Finne, Domminic Mayer, Jake Herweg
*/

#include <LiquidCrystal.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Stepper.h>

#define DHTPIN 2
#define FAN_PIN 3
#define LCD_RS 4
#define LCD_EN 5
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 13
#define STEPPER_PIN1 6
#define STEPPER_PIN2 7
#define STEPPER_PIN3 8
#define STEPPER_PIN4 9
#define BUTTON_PIN 23
#define LIGHT_R 24
#define LIGHT_G 25
#define LIGHT_B 26

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHTPIN, 11);
Stepper myStepper(32, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
void setupPins(){
  // Sets all the pins to the correct mode
  pinMode(2, INPUT); // DHTPIN as input
  pinMode(3, OUTPUT); //FAN as output
  pinMode(4, OUTPUT); // LCD_RS as output
  pinMode(5, OUTPUT); // LCD_EN as output
  pinMode(10, OUTPUT); // LCD_D4 as output
  pinMode(11, OUTPUT); // LCD_D5 as output
  pinMode(12, OUTPUT); // LCD_D6 as output
  pinMode(13, OUTPUT); // LCD_D7 as output
  pinMode(6, OUTPUT); // STEPPER_PIN1 as output
  pinMode(7, OUTPUT); // STEPPER_PIN2 as output
  pinMode(8, OUTPUT); // STEPPER_PIN3 as output
  pinMode(9, OUTPUT); // STEPPER_PIN4 as output
  pinMode(22, INPUT); // LIGHT_R as output
  pinMode(23, INPUT); // BUTTON_PIN as input
  pinMode(24, OUTPUT);  //LIGHT_R as output
  pinMode(25, OUTPUT);  //LIGHT_G as output
  pinMode(26, OUTPUT);  //LIGHT_B as output
}


const int revSteps = 100;
Stepper myStepper(revSteps, 6, 7, 8, 9);

void stepperOpen() {
	myStepper.setSpeed(50);
	myStepper.step(revSteps);
}
void stepperClose() {
	myStepper.setSpeed(50);
	myStepper.step(-revSteps);
}
