#include <SoftwareSerial.h>

// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Re-re-written by Justin Turcotte
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define MIN_DIST 50 //minimum distance (cm) hand can be from sensor before activating ultrasonic

#define echoPin 10 // attach pin 10 Arduino to pin Echo of HC-SR04
#define trigPin 9  //attach pin 9 Arduino to pin Trig of HC-SR04

#define VIB_PIN 5   //vibration actuator connected to pin 5

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  pinMode(VIB_PIN, OUTPUT); //set vibration pin as output
  digitalWrite(VIB_PIN, LOW); //ensure pin starts low
  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}
void loop() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if(distance < MIN_DIST)
  {
    digitalWrite(VIB_PIN, HIGH);
  }
  else
  {
    digitalWrite(VIB_PIN, LOW);
  }

  delay(100);
}
