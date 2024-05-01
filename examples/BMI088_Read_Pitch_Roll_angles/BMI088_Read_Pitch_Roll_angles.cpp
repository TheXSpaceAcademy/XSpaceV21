/**
 This example interfaces with the XSpaceV21 hardware platform, specifically utilizing 
 the BMI088 accelerometer to compute orientation angles (pitch and roll).
*/
#include <Arduino.h>
#include <XSpaceV21.h>

// Instantiate an object of the XSpaceV21Board class to interface with the hardware.
XSpaceV21Board XSBoard;

// Define constants for motor control settings.
#define PWM_FREQUENCY 20000 // Set PWM frequency for motor control to 20 kHz to avoid audible noise.
#define ENCODER_RESOLUTION 960 // Resolution of the encoder, typically the number of steps per revolution.
#define DRV8837_POWER_SUPPLY 5 // Operating voltage in volts for the DRV8837 motor driver.

// Variables to store accelerometer data.
float ax, ay, az;

void setup() {
  Serial.begin(1000000); // Initialize serial communication at 1 Mbps for fast data transmission.
  
  // Initialize the hardware of XSpace v2.1 including BMI088 sensor
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, DRV8837_POWER_SUPPLY);
}

void loop() {
  // Retrieve the latest accelerometer data from the BMI088 sensor.
  XSBoard.BMI088_GetAccelData(&ax, &ay, &az);
  
  // Calculate the pitch angle based on the X and Z accelerometer readings.
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; // Convert radians to degrees.
  
  // Calculate the roll angle based on the Y and Z accelerometer readings.
  float roll = atan2(ay, az) * 180.0 / PI; // Convert radians to degrees.

  // Output the calculated pitch and roll angles to the serial monitor.
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  // Delay for 20 milliseconds
  delay(20);
}