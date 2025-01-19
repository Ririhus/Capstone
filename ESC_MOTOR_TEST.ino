#include <ESP32Servo.h>  // Use the ESP32-specific Servo library

Servo myservo;  // Create a Servo object

void setup() {
  Serial.begin(9600);  // Start the Serial communication at 9600 baud
  myservo.attach(14);  // Attach the servo to GPIO pin 14 (change this pin if needed)
  delay(1);  // Short delay to ensure everything initializes
  myservo.write(10);  // Move the servo to 10 degrees
  //delay(1000);  // Wait for 5 seconds
}

void loop() {
  while (Serial.available() > 0) 
  {
    int val = Serial.parseInt();  // Read the integer value from Serial input
    Serial.println(val);  // Print the value to the Serial Monitor
    myservo.write(val);  // Move the servo to the specified angle
    delay(10000);
  }
}
