# XSpaceV21 Library

Welcome to the `XSpaceV21` library, designed to simplify the development of projects using the XSpace v2.1 Boards. This library provides an easy-to-use interface for controlling motors, reading sensors, and managing connectivity, making it ideal for education, hobby projects, and rapid prototyping.

## Features

- Motor control support with the DRV8837 driver
- Support for ESP32-based Boards and XSpace v2.1 Board

## Installation

### PlatformIO

To use `XSpaceV21` with PlatformIO, add the following line to your `platformio.ini` file under the `lib_deps` section:

```ini
lib_deps =
  https://github.com/TheXSpaceAcademy/XSpaceV21.git#main
```

### Arduino IDE

For Arduino IDE, you can clone this repository and place it in your Arduino libraries folder, or you can download the zip and import it directly into the Arduino IDE.

## Quick Start

Here's a simple example to get you started with the `XSpaceV21` library. This example initializes the board and blinks an onboard LED:

```cpp
#include <XSpaceV21.h>

XSpaceV20Board XSBoard; // Create an instance of the XSpaceV20Board for motor control.

// Constants for motor control
#define PWM_FREQUENCY 20000 // PWM frequency in Hz
#define ENCODER_RESOLUTION 960 // Encoder resolution
#define DRV8837_POWER_SUPPLY 5 // Power supply voltage for DRV8837 motor driver

void setup() {
  // Initialize the motor board with specified PWM frequency, encoder resolution, and power supply voltage
  XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, DRV8837_POWER_SUPPLY);
}

void loop() {
  XSBoard.DRV8837_Wake(DRVx1); // Activate one of the DRV8837 motor driver called DRVx1
  XSBoard.DRV8837_Voltage(DRVx1,2.5); //Apply 2.5 volts to DC Motor
}
```

## Documentation

For detailed documentation, including setup instructions, API reference, and examples, please visit our [GitHub repository](https://github.com/TheXSpaceAcademy/XSpaceV21).

## Contributing

We welcome contributions to the `XSpaceV21` library! If you have suggestions, bug reports, or contributions, please submit them as issues or pull requests on GitHub.

## License

`XSpaceV21` is licensed under the MIT License. See the [LICENSE](https://github.com/TheXSpaceAcademy/XSpaceV21/blob/main/LICENSE) file for more details.

## Acknowledgments

A big thank you to everyone who has contributed to this project. Your support helps make `XSpaceV21` better for everyone.
