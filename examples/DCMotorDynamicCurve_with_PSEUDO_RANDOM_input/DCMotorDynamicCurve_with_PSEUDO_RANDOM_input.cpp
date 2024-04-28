/**
 This program enables obtaining the dynamic response of a DC motor considering PSEUDO-RANDOM voltage inputs.
 The example uses the XSpace V2.1 board and one of the two DRV8837 drivers, marked as DRVx1, and the encoder connector E2.
 */

#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSControl.h>

// Instantiate the board and filter for motor control operations.
XSpaceV21Board XSBoard;
XSFilter Filter;

// Define constants for motor control settings.
#define PWM_FREQUENCY 20000 // PWM frequency for the motor driver in Hz.
#define ENCODER_RESOLUTION 960 // Resolution of the motor encoder.
#define DRV8837_POWER_SUPPLY 5 // Operating voltage for the DRV8837 motor driver.

// Variables for storing real-time and filtered speed measurements.
double speed;
double filtered_speed;

// RTOS task to continuously filter the motor's speed based on encoder readings.
void SpeedFilter(void *pv){
  while(1){
    speed = XSBoard.GetEncoderSpeed(E2, DEGREES_PER_SECOND); // Obtain current speed in degrees per second.
    filtered_speed = Filter.SecondOrderLPF(speed, 20, 0.001); // Apply a second-order low-pass filter to the speed.
    vTaskDelay(1); // Pause the task for 1 millisecond between speed measurements.
  }
  vTaskDelete(NULL); // Safely delete the task if it ever exits the while loop.
}

void setup() {
  Serial.begin(1000000); // Initialize serial communication at 1 Mbps.
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, DRV8837_POWER_SUPPLY); // Set up the motor board with the defined settings.

  randomSeed(esp_random()); // Initialize the random number generator with a hardware-based seed.
  xTaskCreate(SpeedFilter, "Filter", 2000, NULL, 1, NULL); // Create a FreeRTOS task for speed filtering.
}

void loop() {
  XSBoard.DRV8837_Wake(DRVx1); // Activate the DRV8837 motor driver from sleep mode.

  // Prepare MATLAB environment for receiving and plotting data.
  Serial.println("");
  Serial.println("clc; clear; close all;");
  Serial.println("time = 0:0.001:4.999;");
  Serial.print("data = [");
  
  unsigned long lastChangeTime = 0;
  float currentVoltage = random(-5.0, 5.0); // Generate an initial random voltage.
  float randomDelay = random(200, 800); // Random delay for voltage change between 200ms and 800ms.

  // Loop to apply pseudo-random voltages to the motor and collect speed data for plotting.
  for(int i = 0; i < 5000; i++){
    if (millis() - lastChangeTime >= randomDelay) {
      currentVoltage = random(-5.0, 5.0); // Update voltage after the random interval.
      randomDelay = random(200, 800); // Calculate next random interval for voltage change.
      lastChangeTime = millis();
    }
    XSBoard.DRV8837_Voltage(DRVx1, currentVoltage); // Apply the voltage to the motor.
    
    Serial.print(currentVoltage);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(";");
    delay(1);
  }

  // Format the data and send MATLAB plotting commands.
  Serial.println("];");
  Serial.println("subplot(2,1,1);");
  Serial.println("plot(time, data(:,1));");
  Serial.println("xlabel('Time (seconds)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("ylabel('Voltage (volts)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("title('Pseudo-random Input Voltage', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("");
  Serial.println("subplot(2,1,2);");
  Serial.println("plot(time, data(:,2));");
  Serial.println("xlabel('Time (seconds)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("ylabel('Speed (degrees/sec)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("title('Dynamic Curve of DC Motor', 'Interpreter', 'latex', 'FontSize', 14);");
  
  XSBoard.DRV8837_Sleep(DRVx1); // Return the motor driver to low power mode.

  while(1); // Enter an infinite loop to prevent further execution.
}