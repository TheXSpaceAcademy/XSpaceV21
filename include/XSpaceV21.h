/*
	XSpaceV21 Library
	Author: Pablo Cardenas
	Description: An open-source library designed for advanced robotics projects. Please remember to reference this library in your projects!
*/

#ifndef XSPACEV21_H
#define XSPACEV21_H

#include <Arduino.h>
#include <SPI.h>
#include <XSpaceBmi088.h>

#define DRVx1_IN1 33
#define DRVx1_IN2 25
#define DRVx1_nSLEEP 15
#define DRVx1 0

#define DRVx2_IN1 14
#define DRVx2_IN2 12
#define DRVx2_nSLEEP 4
#define DRVx2 1

#define encoder1_CHA 34
#define encoder1_CHB 35
#define E1 0

#define encoder2_CHA 26
#define encoder2_CHB 27
#define E2 1

#define DEGREES_PER_SECOND 1
#define RADS_PER_SECOND 2

#define DEGREES 1
#define RADS 2


struct XSEncoder{
	int resolution;
	int channelA;
	int channelB;

	double speed_ant = 0;
};

struct XSDRV88xx{
	int IN1;
	int IN2;
	int PWM_CH_IN1;
	int PWM_CH_IN2;
	int nSLEEP;
	double Vm;
};

class XSpaceV21Board{
	private:
			double _vel_ant = 0;
			XSDRV88xx DRV88xx[2];
      XSBMI088 *bmi;

	public:
		/** Initializes the board and motor driver
			@param frequency PWM frequency for the DRV8837 driver (in Hz)
			@param resolution Encoder pulses per revolution * gearmotor reduction
			@param VM DRV8837 driver power supply voltage (range 0v to 11v)
		*/
		void init(int frequency, double resolution, double VM);

		/** Initializes encoder1 settings
			@param enc_CHAx Channel A pin number
			@param enc_CHBx Channel B pin number
			@param resolutionx Encoder resolution
			@param mode Encoding mode (RISING, FALLING)
		*/
		void encoder1_init(int enc_CHAx, int enc_CHBx, int resolutionx, int mode);

		/** Initializes encoder2 settings
			@param enc_CHAx Channel A pin number
			@param enc_CHBx Channel B pin number
			@param resolutionx Encoder resolution
			@param mode Encoding mode (RISING, FALLING)
		*/
		void encoder2_init(int enc_CHAx, int enc_CHBx, int resolutionx, int mode);

		/** Initializes DRV8837 motor driver
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
			@param IN1x Pin connected to IN1 on DRV8837
			@param CH_IN1x PWM channel for IN1
			@param IN2x Pin connected to IN2 on DRV8837
			@param CH_IN2x PWM channel for IN2
			@param nSLEEPx Pin connected to nSLEEP on DRV8837
			@param frequencyx PWM frequency
			@param Vmx Operating voltage for the DRV8837
		*/
		void DRV8837_init(int DRVx, int IN1x, int CH_IN1x, int IN2x, int CH_IN2x, int nSLEEPx,int frequencyx, int Vmx);

		/** Puts the DRV8837 motor driver into sleep mode, reducing power consumption.
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
		*/
		void DRV8837_Sleep(int DRVx);

		/** Wakes the DRV8837 motor driver from sleep mode, enabling normal operation.
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
		*/
		void DRV8837_Wake(int DRVx);

		/** Sets the operating voltage for the DRV8837 motor driver.
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
		   	@param vp The voltage to be set, which controls the motor speed and power (range 0v to 11v).
		*/
		void DRV8837_Voltage(int DRVx,double vp);
				
		/** Retrieves the speed of the specified encoder
			@param encoder Encoder number (E1 or E2)
			@param mode Unit of measurement (DEGREES_PER_SECOND, RADS_PER_SECOND)
			@return Current speed of the encoder
		*/
		double GetEncoderSpeed(int encoder, int modo);

		/** Retrieves the position of the specified encoder
			@param encoder Encoder number (E1 or E2)
			@param mode Unit of measurement (DEGREES, RADS)
			@return Current position of the encoder
		*/
		double GetEncoderPosition(int encoder, int modo);


		/** Initializes the BMI088 sensor with specific chip select pins for accelerometer and gyroscope.
		 *  This function must be called before attempting to read sensor data to ensure the sensor is
		 *  properly configured and ready to transmit data.
		 *  @param accel_cs The chip select pin number used for the accelerometer part of the sensor.
		 *  @param gyro_cs The chip select pin number used for the gyroscope part of the sensor.
		 */
		void BMI088_init(int accel_cs, int gyro_cs);
		
		/**  Fetches the latest sensor data from the BMI088 sensor for both the accelerometer and gyroscope.
		*   This function assumes that the sensor has been initialized with BMI088_init.
		*   @param ax Pointer to a float where the accelerometer's X-axis data will be stored.
		*   @param ay Pointer to a float where the accelerometer's Y-axis data will be stored.
		*   @param az Pointer to a float where the accelerometer's Z-axis data will be stored.
		*   @param gx Pointer to a float where the gyroscope's X-axis data will be stored.
		*   @param gy Pointer to a float where the gyroscope's Y-axis data will be stored.
		*   @param gz Pointer to a float where the gyroscope's Z-axis data will be stored.
		*/
		void BMI088_GetData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

		/**
		 * Retrieves the latest accelerometer data from the BMI088 sensor. This function
		 * fetches the most recent X, Y, and Z-axis accelerometer readings and stores them
		 * in the provided floating-point variables.
		 *
		 * @param ax Pointer to a float where the accelerometer's X-axis data will be stored.
		 *        This location will be overwritten with the new data from the sensor.
		 * @param ay Pointer to a float where the accelerometer's Y-axis data will be stored.
		 *        The existing content will be replaced with the updated reading.
		 * @param az Pointer to a float where the accelerometer's Z-axis data will be stored.
		 *        The value pointed to by this parameter will be updated with fresh sensor data.
		 */
		void BMI088_GetAccelData(float *ax, float *ay, float *az);

		/**
		 * Fetches the latest gyroscope data from the BMI088 sensor. This function
		 * collects the current rotational velocity readings around the X, Y, and Z axes
		 * and outputs them through the provided pointers.
		 *
		 * @param gx Pointer to a float that will receive the gyroscope's X-axis rotational
		 *        velocity in radians per second. The existing content will be replaced with
		 *        the new data.
		 * @param gy Pointer to a float that will store the gyroscope's Y-axis rotational
		 *        velocity. The previous value will be overwritten by the updated data.
		 * @param gz Pointer to a float to be updated with the gyroscope's Z-axis data.
		 *        The provided memory location will be overwritten with the new sensor output.
		 */
		void BMI088_GetGyroData(float *gx, float *gy, float *gz);
};

#endif