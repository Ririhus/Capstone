#include <ESP32Servo.h>  // Use the ESP32-specific Servo library
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusMaster.h>

#ifndef DEVICE_DSICONNECRED_C
#define DEVICE_DSICONNECRED_C -127
#endif
#define ESC_MIN 1000 // Minimum throttle (stopped)
#define ESC_MAX 2000 // Maximum throttle
#define ESC_NEUTRAL 1480 // Neutral position (stopped)
#define ONE_WIRE_BUS 19 // Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0X27, 16, 2);

Servo esc1;  
Servo esc2; 

const int relayPinA1 = 14;
const int relayPinA2 = 15;
const int relayPinA3 = 12;
const int relayPinA4 = 13;
const int B2joystick = 32;
const int B3joystick = 33;
const int B4joystick = 34;
const int B5joystick = 35;

//Sink and float
// Right Motor connections (BTS7960)
int r_en = 4;   // Right motor EN pin
int r_pwm = 25;  // Right motor PWM pin

// Left Motor connections (BTS7960)
int l_en = 5;   // Left motor EN pin
int l_pwm = 26;  // Left motor PWM pin

// Button pins
int button1 = 27; // Sink button
int button2 = 36; // Float button
int button3 = 18; // Stop motors button

// Function to initialize ESCs
void initializeESCs() {
  esc1.attach(16);
  esc2.attach(17);

  // Set ESCs to neutral position
  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(ESC_NEUTRAL);

  //delay(2000); // Wait for ESCs to arm
}

// Function to stop all ESCs
void stopMotors() {
  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(ESC_NEUTRAL);
}

// Movement functions
void moveForward(int speed) {
  int pwm = map(speed, 0, 100, ESC_NEUTRAL, ESC_MAX);
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
}

void moveBackward(int speed) {
  int pwm = map(speed, 0, 100, ESC_NEUTRAL, ESC_MAX);
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
}

void turnLeft(int speed) {
  int pwm = map(speed, 0, 100, ESC_NEUTRAL, ESC_MAX);
  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(pwm);
}

void turnRight(int speed) {
  int pwm = map(speed, 0, 100, ESC_NEUTRAL, ESC_MAX);
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(ESC_NEUTRAL);
}
void Forward(){
  digitalWrite(relayPinA1, LOW);
  digitalWrite(relayPinA2, LOW);
  digitalWrite(relayPinA3, LOW);
  digitalWrite(relayPinA4, LOW);
}
void Backward(){
  digitalWrite(relayPinA1, HIGH);
  digitalWrite(relayPinA2, HIGH);
  digitalWrite(relayPinA3, HIGH);
  digitalWrite(relayPinA4, HIGH);
}

void LT(){
  digitalWrite(relayPinA1, LOW);
  digitalWrite(relayPinA2, LOW);
  digitalWrite(relayPinA3, HIGH);
  digitalWrite(relayPinA4, HIGH);
}

void RT(){
  digitalWrite(relayPinA1, HIGH);
  digitalWrite(relayPinA2, HIGH);
  digitalWrite(relayPinA3, LOW);
  digitalWrite(relayPinA4, LOW);
}

// Sinking function
void Sink() {
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);
  // Set motors to maximum speed (PWM to 255)
  analogWrite(r_pwm, 100);  // Right motor speed
  analogWrite(l_pwm, 0);  // Left motor speed
  // Move right motor forward
 
}

// Floating function
void Float() {
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH);
  // Set motors to maximum speed (PWM to 255)
  analogWrite(r_pwm, 0);  // Right motor speed
  analogWrite(l_pwm, 100);  // Left motor speed
  // Move right motor backward
  
}

// Stop vertical movement function
void stopVertical() {
  // Stop both motors by setting PWM to 0
  analogWrite(r_pwm, 0);  // Right motor speed (stop)
  analogWrite(l_pwm, 0);  // Left motor speed (stop)
}

void setup() 
{
  Serial.begin(9600);  // Start the Serial communication at 9600 baud
  sensors.begin();
  lcd.begin(16,2);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("   UnI Robot");
  lcd.setCursor(0, 1);
  lcd.print("TempSensor Ready");
  delay (2000);
  lcd.clear();
  initializeESCs();

  // Set all motor control pins to outputs
  pinMode(r_en, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  
  pinMode(l_en, OUTPUT);
  pinMode(l_pwm, OUTPUT);

  // Set button pins to inputs with pull-up resistors
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  // Turn off motors - Initial state
  analogWrite(r_pwm, 0);  // Set PWM to 0 (no movement)
  analogWrite(l_pwm, 0);  // Set PWM to 0 (no movement)

  pinMode(B2joystick, INPUT);
  pinMode(B3joystick, INPUT);
  pinMode(B4joystick, INPUT); 
  pinMode(B5joystick, INPUT);
  pinMode(relayPinA1, OUTPUT);
  pinMode(relayPinA2, OUTPUT);
  pinMode(relayPinA3, OUTPUT);
  pinMode(relayPinA4, OUTPUT);
  digitalWrite(relayPinA1, HIGH);
  digitalWrite(relayPinA2, HIGH);
  digitalWrite(relayPinA3, HIGH);
  digitalWrite(relayPinA4, HIGH);
}

void loop() 
{
  if (digitalRead(B2joystick) == LOW)
    {
      Forward();
      moveForward(50); // Move forward at 50% speed
    }
    else if (digitalRead(B5joystick) == LOW)
    {
      Forward();
      turnLeft(50); // Turn left at 50% speed
    }
    else if (digitalRead(B3joystick) == LOW)
    {
      Forward();
      turnRight(50); // Turn right at 50% speed
    }
    else if (digitalRead(B4joystick) == LOW)
    {
      Backward();
      moveBackward(50); // Move backward at 50% speed
    }
    else
    {
      stopMotors(); // Stop all motors when no button is pressed
    }

  if (digitalRead(button1) == LOW) {  // Button 1 pressed (sink)
    Sink();
  }
  else if (digitalRead(button2) == LOW) {  // Button 2 pressed (float)
    Float();
  }
  else  {  // Button 3 pressed (stop motors)
    stopVertical();
  }
  
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex (0);
    if (tempC != DEVICE_DSICONNECRED_C){
      lcd.setCursor(0, 0);
      lcd.print("   UnI Robot");
      lcd.setCursor(0, 1);
      lcd.print(" Temp : ");
      lcd.print(tempC);
      lcd.print(" C");
    }
}
