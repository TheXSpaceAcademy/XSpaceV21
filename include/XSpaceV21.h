/*
	XSpaceV21 Library
	Author: Pablo Cardenas
	Description: An open-source library designed for advanced robotics projects. Please remember to reference this library in your projects!
*/

#ifndef XSPACEV21_H
#define XSPACEV21_H

#include <Arduino.h>
#include <SPI.h>

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
		/* Initializes the board and motor driver
			@param frequency PWM frequency for the DRV8837 driver (in Hz)
			@param resolution Encoder pulses per revolution * gearmotor reduction
			@param VM DRV8837 driver power supply voltage (range 0v to 11v)
		*/
		void init(int frequency, double resolution, double VM);

		/* Initializes encoder1 settings
			@param enc_CHAx Channel A pin number
			@param enc_CHBx Channel B pin number
			@param resolutionx Encoder resolution
			@param mode Encoding mode (RISING, FALLING)
		*/
		void encoder1_init(int enc_CHAx, int enc_CHBx, int resolutionx, int mode);

		/* Initializes encoder2 settings
			@param enc_CHAx Channel A pin number
			@param enc_CHBx Channel B pin number
			@param resolutionx Encoder resolution
			@param mode Encoding mode (RISING, FALLING)
		*/
		void encoder2_init(int enc_CHAx, int enc_CHBx, int resolutionx, int mode);

		/* Initializes DRV8837 motor driver
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

		/* Puts the DRV8837 motor driver into sleep mode, reducing power consumption.
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
		*/
		void DRV8837_Sleep(int DRVx);

		/* Wakes the DRV8837 motor driver from sleep mode, enabling normal operation.
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
		*/
		void DRV8837_Wake(int DRVx);

		/* Sets the operating voltage for the DRV8837 motor driver.
			@param DRVx index of DRV8837 (DRVx1, DRVx2)
		   	@param vp The voltage to be set, which controls the motor speed and power (range 0v to 11v).
		*/
		void DRV8837_Voltage(int DRVx,double vp);
				
		/* Retrieves the speed of the specified encoder
			@param encoder Encoder number (E1 or E2)
			@param mode Unit of measurement (DEGREES_PER_SECOND, RADS_PER_SECOND)
			@return Current speed of the encoder
		*/
		double GetEncoderSpeed(int encoder, int modo);

		/* Retrieves the position of the specified encoder
			@param encoder Encoder number (E1 or E2)
			@param mode Unit of measurement (DEGREES, RADS)
			@return Current position of the encoder
		*/
		double GetEncoderPosition(int encoder, int modo);


    void Bmi088_init(int accel_cs, int gyro_cs);
    void Bmi088_readSensor(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
};





class Bmi088Accel {
  public:
    enum Range {
      RANGE_3G = 0x00,
      RANGE_6G = 0x01,
      RANGE_12G = 0x02,
      RANGE_24G = 0x03
    };
    enum Odr {
      ODR_1600HZ_BW_280HZ,
      ODR_1600HZ_BW_234HZ,
      ODR_1600HZ_BW_145HZ,
      ODR_800HZ_BW_230HZ,
      ODR_800HZ_BW_140HZ,
      ODR_800HZ_BW_80HZ,
      ODR_400HZ_BW_145HZ,
      ODR_400HZ_BW_75HZ,
      ODR_400HZ_BW_40HZ,
      ODR_200HZ_BW_80HZ,
      ODR_200HZ_BW_38HZ,
      ODR_200HZ_BW_20HZ,
      ODR_100HZ_BW_40HZ,
      ODR_100HZ_BW_19HZ,
      ODR_100HZ_BW_10HZ,
      ODR_50HZ_BW_20HZ,
      ODR_50HZ_BW_9HZ,
      ODR_50HZ_BW_5HZ,
      ODR_25HZ_BW_10HZ,
      ODR_25HZ_BW_5HZ,
      ODR_25HZ_BW_3HZ,
      ODR_12_5HZ_BW_5HZ,
      ODR_12_5HZ_BW_2HZ,
      ODR_12_5HZ_BW_1HZ
    };
    Bmi088Accel(SPIClass &bus,uint8_t csPin);
    int begin();
    bool setOdr(Odr odr);
    bool setRange(Range range);
    bool getDrdyStatus();
    void readSensor();
    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getTemperature_C();
    uint64_t getTime_ps();
  private:
    // allow class Bmi088 access to private members 
    friend class XSBMI088;
    // spi
    uint8_t _csPin;
    SPIClass *_spi;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_CLOCK = 10000000; // 10 MHz
    // buffer for reading from sensor
    uint8_t _buffer[9];
    // constants
    static const uint8_t ACC_CHIP_ID = 0x1E;
    static const uint8_t ACC_RESET_CMD = 0xB6;
    static const uint8_t ACC_ENABLE_CMD = 0x04;
    static const uint8_t ACC_DISABLE_CMD = 0x00;
    static const uint8_t ACC_SUSPEND_MODE_CMD = 0x03;
    static const uint8_t ACC_ACTIVE_MODE_CMD = 0x00;
    static const uint8_t ACC_INT_INPUT = 0x11;
    static const uint8_t ACC_INT_OUTPUT = 0x08;
    static const uint8_t ACC_INT_OPENDRAIN = 0x04;
    static const uint8_t ACC_INT_PUSHPULL = 0x00;
    static const uint8_t ACC_INT_LVL_HIGH = 0x02;
    static const uint8_t ACC_INT_LVL_LOW = 0x00;
    static const uint8_t ACC_POS_SELF_TEST = 0x0D;
    static const uint8_t ACC_NEG_SELF_TEST = 0x09;
    static const uint8_t ACC_DIS_SELF_TEST = 0x00;
    // registers
    static const uint8_t ACC_CHIP_ID_ADDR = 0x00;
    static const uint8_t ACC_CHIP_ID_MASK = 0xFF;
    static const uint8_t ACC_CHIP_ID_POS = 0;
    static const uint8_t ACC_FATAL_ERR_ADDR = 0x02;
    static const uint8_t ACC_FATAL_ERR_MASK = 0x01;
    static const uint8_t ACC_FATAL_ERR_POS = 0;
    static const uint8_t ACC_ERR_CODE_ADDR = 0x02;
    static const uint8_t ACC_ERR_CODE_MASK = 0x1C;
    static const uint8_t ACC_ERR_CODE_POS = 2;
    static const uint8_t ACC_DRDY_ADDR = 0x03;
    static const uint8_t ACC_DRDY_MASK = 0x80;
    static const uint8_t ACC_DRDY_POS = 7;
    static const uint8_t ACC_ODR_ADDR = 0x40;
    static const uint8_t ACC_ODR_MASK = 0xFF;
    static const uint8_t ACC_ODR_POS = 0;
    static const uint8_t ACC_RANGE_ADDR = 0x41;
    static const uint8_t ACC_RANGE_MASK = 0x03;
    static const uint8_t ACC_RANGE_POS = 0;
    static const uint8_t ACC_INT1_IO_CTRL_ADDR = 0x53;
    static const uint8_t ACC_INT1_IO_CTRL_MASK = 0x1F;
    static const uint8_t ACC_INT1_IO_CTRL_POS = 0;
    static const uint8_t ACC_INT2_IO_CTRL_ADDR = 0x54;
    static const uint8_t ACC_INT2_IO_CTRL_MASK = 0x1F;
    static const uint8_t ACC_INT2_IO_CTRL_POS = 0;
    static const uint8_t ACC_INT1_DRDY_ADDR = 0x58;
    static const uint8_t ACC_INT1_DRDY_MASK = 0x04;
    static const uint8_t ACC_INT1_DRDY_POS = 2;
    static const uint8_t ACC_INT2_DRDY_ADDR = 0x58;
    static const uint8_t ACC_INT2_DRDY_MASK = 0x40;
    static const uint8_t ACC_INT2_DRDY_POS = 6;
    static const uint8_t ACC_SELF_TEST_ADDR = 0x6D;
    static const uint8_t ACC_SELF_TEST_MASK = 0xFF;
    static const uint8_t ACC_SELF_TEST_POS = 0;
    static const uint8_t ACC_PWR_CONF_ADDR = 0x7C;
    static const uint8_t ACC_PWR_CONF_MASK = 0xFF;
    static const uint8_t ACC_PWR_CONF_POS = 0;
    static const uint8_t ACC_PWR_CNTRL_ADDR = 0x7D;
    static const uint8_t ACC_PWR_CNTRL_MASK = 0xFF;
    static const uint8_t ACC_PWR_CNTRL_POS = 0;
    static const uint8_t ACC_SOFT_RESET_ADDR = 0x7E;
    static const uint8_t ACC_SOFT_RESET_MASK = 0xFF;
    static const uint8_t ACC_SOFT_RESET_POS = 0;
    static const uint8_t ACC_ACCEL_DATA_ADDR = 0x12;
    static const uint8_t ACC_TEMP_DATA_ADDR = 0x22;
    // transformation from sensor frame to right hand coordinate system
    const int16_t tX[3] = {1, 0, 0};
    const int16_t tY[3] = {0, -1, 0};
    const int16_t tZ[3] = {0, 0, -1};
    // convert G to m/s/s
    const float G = 9.807f;
    // accel full scale range
    float accel_range_mss;
    // accel data
    float accel_mss[3];
    // temperature data
    float temp_c;
    // sensor time
    uint32_t current_time_counter, prev_time_counter = 0;
    uint64_t time_counter;
    // self test
    bool selfTest();
    // power and mode settings
    bool setMode(bool active);
    bool setPower(bool enable);
    // command soft reset
    void softReset();
    // error checking
    bool isConfigErr();
    bool isFatalErr();
    // check id
    bool isCorrectId();
    // write / read registers
    void writeRegister(uint8_t subAddress, uint8_t data);
    void writeRegisters(uint8_t subAddress, uint8_t count, const uint8_t* data);
    void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
};

class Bmi088Gyro {
  public:
    enum Range {
      RANGE_2000DPS = 0x00,
      RANGE_1000DPS = 0x01,
      RANGE_500DPS = 0x02,
      RANGE_250DPS = 0x03,
      RANGE_125DPS = 0x04
    };
    enum Odr {
      ODR_2000HZ_BW_532HZ = 0x80,
      ODR_2000HZ_BW_230HZ = 0x81,
      ODR_1000HZ_BW_116HZ = 0x82,
      ODR_400HZ_BW_47HZ = 0x83,
      ODR_200HZ_BW_23HZ = 0x84,
      ODR_100HZ_BW_12HZ = 0x85,
      ODR_200HZ_BW_64HZ = 0x86,
      ODR_100HZ_BW_32HZ = 0x87
    };
    Bmi088Gyro(SPIClass &bus,uint8_t csPin);
    int begin();
    bool setOdr(Odr odr);
    bool setRange(Range range);
    bool getDrdyStatus();
    void readSensor();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
  private:
    // available power settings
    enum PowerMode {
      PWR_NORMAL = 0x00,
      PWR_SUSPEND = 0x80,
      PWR_DEEP_SUSPEND = 0x20
    };
    // spi
    uint8_t _csPin;
    SPIClass *_spi;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_CLOCK = 10000000; // 10 MHz
    // buffer for reading from sensor
    uint8_t _buffer[9];
    // constants
    static const uint8_t GYRO_CHIP_ID = 0x0F;
    static const uint8_t GYRO_RESET_CMD = 0xB6;
    static const uint8_t GYRO_ENABLE_DRDY_INT = 0x80;
    static const uint8_t GYRO_DIS_DRDY_INT = 0x00;
    static const uint8_t GYRO_INT_OPENDRAIN = 0x02;
    static const uint8_t GYRO_INT_PUSHPULL = 0x00;
    static const uint8_t GYRO_INT_LVL_HIGH = 0x01;
    static const uint8_t GYRO_INT_LVL_LOW = 0x00;
    // registers
    static const uint8_t GYRO_CHIP_ID_ADDR = 0x00;
    static const uint8_t GYRO_CHIP_ID_MASK = 0xFF;
    static const uint8_t GYRO_CHIP_ID_POS = 0;
    static const uint8_t GYRO_DRDY_ADDR = 0x0A;
    static const uint8_t GYRO_DRDY_MASK = 0x80;
    static const uint8_t GYRO_DRDY_POS = 7;
    static const uint8_t GYRO_RANGE_ADDR = 0x0F;
    static const uint8_t GYRO_RANGE_MASK = 0xFF;
    static const uint8_t GYRO_RANGE_POS = 0;
    static const uint8_t GYRO_ODR_ADDR = 0x10;
    static const uint8_t GYRO_ODR_MASK = 0xFF;
    static const uint8_t GYRO_ODR_POS = 0;
    static const uint8_t GYRO_SOFT_RESET_ADDR = 0x14;
    static const uint8_t GYRO_SOFT_RESET_MASK = 0xFF;
    static const uint8_t GYRO_SOFT_RESET_POS = 0;
    static const uint8_t GYRO_INT_CNTRL_ADDR = 0x15;
    static const uint8_t GYRO_INT_CNTRL_MASK = 0xFF;
    static const uint8_t GYRO_INT_CNTRL_POS = 0;
    static const uint8_t GYRO_INT3_IO_CTRL_ADDR = 0x16;
    static const uint8_t GYRO_INT3_IO_CTRL_MASK = 0x03;
    static const uint8_t GYRO_INT3_IO_CTRL_POS = 0;
    static const uint8_t GYRO_INT4_IO_CTRL_ADDR = 0x16;
    static const uint8_t GYRO_INT4_IO_CTRL_MASK = 0x0C;
    static const uint8_t GYRO_INT4_IO_CTRL_POS = 2;
    static const uint8_t GYRO_INT3_DRDY_ADDR = 0x18;
    static const uint8_t GYRO_INT3_DRDY_MASK = 0x01;
    static const uint8_t GYRO_INT3_DRDY_POS = 0;
    static const uint8_t GYRO_INT4_DRDY_ADDR = 0x18;
    static const uint8_t GYRO_INT4_DRDY_MASK = 0x80;
    static const uint8_t GYRO_INT4_DRDY_POS = 7;
    static const uint8_t GYRO_DATA_ADDR = 0x02;
    // transformation from sensor frame to right hand coordinate system
    const int16_t tX[3] = {1, 0, 0};
    const int16_t tY[3] = {0, -1, 0};
    const int16_t tZ[3] = {0, 0, -1};
    // convert deg/s to rad/s
    const float D2R = M_PI / 180.0f;
    // gyro full scale range
    float gyro_range_rads;
    // gyro data
    float gyro_rads[3];
    // command soft reset
    void softReset();
    // check id
    bool isCorrectId();
    // write / read registers
    void writeRegister(uint8_t subAddress, uint8_t data);
    void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
};

class XSBMI088 {
  public:
    enum AccelRange {
      ACCEL_RANGE_3G,
      ACCEL_RANGE_6G,
      ACCEL_RANGE_12G,
      ACCEL_RANGE_24G
    };
    enum GyroRange {
      GYRO_RANGE_2000DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_250DPS,
      GYRO_RANGE_125DPS
    };
    enum Odr {
      ODR_2000HZ,
      ODR_1000HZ,
      ODR_400HZ
    };
    XSBMI088(SPIClass &bus,uint8_t accel_cs,uint8_t gyro_cs);
    int begin();
    bool setOdr(Odr odr);
    bool setRange(AccelRange accel_range,GyroRange gyro_range);
    void readSensor(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getTemperature_C();
    uint64_t getTime_ps();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();  
  private:
    Bmi088Accel *accel;
    Bmi088Gyro *gyro;
    // constants
    static const uint8_t ACC_DISABLE = 0;
    static const uint8_t ACC_ENABLE = 1;
    static const uint8_t ACC_DATA_SYNC_LEN = 1;
    static const uint16_t ACC_DATA_SYNC_MODE_MASK = 0x0003;
    static const uint16_t ACC_DATA_SYNC_MODE_OFF = 0x00;
    static const uint16_t ACC_DATA_SYNC_MODE_400HZ = 0x01;
    static const uint16_t ACC_DATA_SYNC_MODE_1000HZ = 0x02;
    static const uint16_t ACC_DATA_SYNC_MODE_2000HZ = 0x03;
    static const uint8_t ACC_INTA_DISABLE = 0x00;
    static const uint8_t ACC_INTA_ENABLE  = 0x01;
    static const uint8_t ACC_INTB_DISABLE = 0x00;
    static const uint8_t ACC_INTB_ENABLE  = 0x02;
    // registers
    static const uint8_t ACC_INIT_CTRL_ADDR = 0x59;
    static const uint8_t ACC_FEATURE_LSB_ADDR = 0x5B;
    static const uint8_t ACC_FEATURE_MSB_ADDR = 0x5C;
    static const uint8_t ACC_FEATURE_CFG_ADDR = 0x5E;
    static const uint8_t ACC_INTERNAL_STATUS_ADDR = 0x2A;
    static const uint8_t ACC_DATA_SYNC_ADDR = 0x02;
    static const uint8_t ACC_INT1_MAP_ADDR = 0x56;
    static const uint8_t ACC_INT2_MAP_ADDR = 0x57;
    bool writeFeatureConfig();  
    void updateFeatureConfig(uint8_t addr, uint8_t count, const uint16_t *data);    
};




#endif