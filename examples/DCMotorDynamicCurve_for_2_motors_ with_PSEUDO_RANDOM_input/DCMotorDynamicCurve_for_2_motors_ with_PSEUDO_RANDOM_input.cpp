/**
 * This program acquires the dynamic response of a DC motor with pseudo-random voltage inputs.
 * The example utilizes the XSpace V2.1 board, DRV8837 motor drivers (referred to as DRVx1 and DRVx2),
 * and encoder connectors E1 and E2.
 */

#include <Arduino.h>
#include <XSpaceV21.h> // Required for XSpace V2.1 board
#include <XSControl.h> // Required for motor control functions

// Instantiate the board and filters for motor control operations.
XSpaceV21Board XSBoard;
XSFilter Filter1;
XSFilter Filter2;

// Define constants for motor control settings.
#define PWM_FREQUENCY 20000 // PWM frequency for the motor driver in Hz
#define ENCODER_RESOLUTION 960 // Resolution of the motor encoder
#define MAX_MOTOR_VOLTAGE 6 // Maximum voltage to be applied to the motor

// Variables for real-time and filtered speed measurements.
double speed_m1;
double speed_m2;
double filtered_speed_m1;
double filtered_speed_m2;

/**
 * Task function to filter the speed readings.
 * This function runs indefinitely, sampling the motor speed and applying a second-order low-pass filter.
 */
void SpeedFilter(void *pv){
  while(1){
    // Get current speed in degrees per second from encoder 1 and 2.
    speed_m1 = XSBoard.GetEncoderSpeed(E1, DEGREES_PER_SECOND);
    speed_m2 = XSBoard.GetEncoderSpeed(E2, DEGREES_PER_SECOND);

    // Apply a second-order low-pass filter to the speed readings.
    filtered_speed_m1 = Filter1.SecondOrderLPF(speed_m1, 20, 0.001);
    filtered_speed_m2 = Filter2.SecondOrderLPF(speed_m2, 20, 0.001);

    // Delay the task for 1 millisecond between speed measurements.
    vTaskDelay(1);
  }
  vTaskDelete(NULL); // Safely delete the task if the loop exits (though it never should).
}

void setup() {
  Serial.begin(1000000); // Initialize serial communication at 1 Mbps.

  // Calculate VM from connected 8.4V Li-ion batteries (example setup).
  double VM = (double)analogRead(36) / 4096.0 * 3.3 * 4.0;

  // Initialize the motor board with the defined settings.
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, VM);

  // Seed the random number generator using a hardware-based value.
  randomSeed(esp_random());

  // Create a FreeRTOS task for speed filtering.
  xTaskCreate(SpeedFilter, "Filter", 2000, NULL, 1, NULL);
}

void loop() {
  // Wake up the DRV8837 motor driver for the first motor (DRVx1).
  XSBoard.DRV8837_Wake(DRVx1);

  // Prepare MATLAB environment for receiving and plotting data.
  Serial.println("");
  Serial.println("clc; clear; close all;");
  Serial.println("time = 0:0.001:4.999;");
  Serial.print("data1 = [");

  // Variables for controlling the voltage changes.
  unsigned long lastChangeTime = 0;
  float currentVoltage = random(-MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE); // Initial random voltage
  float randomDelay = random(200, 800); // Delay for voltage change in milliseconds

  // Loop to apply pseudo-random voltages to the motor and collect speed data for plotting.
  for(int i = 0; i < 5000; i++){
    // Change voltage after a random interval.
    if (millis() - lastChangeTime >= randomDelay) {
      currentVoltage = random(-MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
      randomDelay = random(200, 800);
      lastChangeTime = millis();
    }
    // Apply the current voltage to the motor driver.
    XSBoard.DRV8837_Voltage(DRVx1, currentVoltage);

    // Output the voltage and speed measurements to the serial monitor.
    Serial.print(currentVoltage);
    Serial.print(" ");
    Serial.print(speed_m1);
    Serial.print(";");

    delay(1); // Sampling interval
  }

  // Format the collected data for MATLAB plotting.
  Serial.println("];");
  Serial.println("figure;");
  Serial.println("subplot(2,1,1);");
  Serial.println("plot(time, data1(:,1));");
  Serial.println("xlabel('Time (seconds)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("ylabel('Voltage (volts)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("title('Pseudo-random Input Voltage', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("");
  Serial.println("subplot(2,1,2);");
  Serial.println("plot(time, data1(:,2));");
  Serial.println("xlabel('Time (seconds)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("ylabel('Speed (degrees/sec)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("title('Dynamic Response of DC Motor 1', 'Interpreter', 'latex', 'FontSize', 14);");

  // Put the first motor driver back to low power mode.
  XSBoard.DRV8837_Sleep(DRVx1);

  // Repeat the same process for the second motor driver (DRVx2).
  XSBoard.DRV8837_Wake(DRVx2);

  Serial.println("");
  Serial.println("time = 0:0.001:4.999;");
  Serial.print("data2 = [");

  lastChangeTime = 0;
  currentVoltage = random(-MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE); // Initial random voltage
  randomDelay = random(200, 800); // Delay for voltage change in milliseconds

  for(int i = 0; i < 5000; i++){
    if (millis() - lastChangeTime >= randomDelay) {
      currentVoltage = random(-MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
      randomDelay = random(200, 800);
      lastChangeTime = millis();
    }
    XSBoard.DRV8837_Voltage(DRVx2, currentVoltage);

    Serial.print(currentVoltage);
    Serial.print(" ");
    Serial.print(speed_m2);
    Serial.print(";");

    delay(1); // Sampling interval
  }

  // Format the collected data for MATLAB plotting.
  Serial.println("];");
  Serial.println("figure;");
  Serial.println("subplot(2,1,1);");
  Serial.println("plot(time, data2(:,1));");
  Serial.println("xlabel('Time (seconds)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("ylabel('Voltage (volts)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("title('Pseudo-random Input Voltage', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("");
  Serial.println("subplot(2,1,2);");
  Serial.println("plot(time, data2(:,2));");
  Serial.println("xlabel('Time (seconds)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("ylabel('Speed (degrees/sec)', 'Interpreter', 'latex', 'FontSize', 14);");
  Serial.println("title('Dynamic Response of DC Motor 2', 'Interpreter', 'latex', 'FontSize', 14);");

  // Put the second motor driver back to low power mode.
  XSBoard.DRV8837_Sleep(DRVx2);

  // Enter an infinite loop to prevent further execution.
  while(1);
}
