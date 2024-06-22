/*
  XSpace V21 Library
  Author     :  Pablo Cardenas
  This is an open source library but dont remeber to reference!
*/

#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSpaceBmi088.h>

volatile double TimerValue[2] = {0, 0};
volatile double Periodo[2] = {100000000, 100000000};
volatile double Tant[2] = {0, 0};
volatile double counter[2] = {0, 0};
volatile double Xval[2];
XSEncoder encoder[2];

void IRAM_ATTR ISR_encoder1(){
	TimerValue[E1] = micros();
	Xval[E1] = (-1+digitalRead(encoder[E1].channelB)*2);
	Periodo[E1] = (TimerValue[E1] - Tant[E1])*Xval[E1];
	Tant[E1] = TimerValue[E1];
	counter[E1] = counter[E1]+Xval[E1];
}

void IRAM_ATTR ISR_encoder2(){
	TimerValue[E2] = micros();
	Xval[E2] = (-1+digitalRead(encoder[E2].channelB)*2);
	Periodo[E2] = (TimerValue[E2] - Tant[E2])*Xval[E2];
	Tant[E2] = TimerValue[E2];
	counter[E2] = counter[E2]+Xval[E2];
}

/*****************************XSpaceV21Board******************************************/

void XSpaceV21Board::init(int frequency, double resolution, double Vm){
	this->DRV8837_init(0,DRVx1_IN1, 2, DRVx1_IN2, 3, DRVx1_nSLEEP, 20000, Vm);
	this->DRV8837_init(1,DRVx2_IN1, 0, DRVx2_IN2, 1, DRVx2_nSLEEP, 20000, Vm);
	this->BMI088_init(CS_Accel,CS_Gyro);
	this->encoder1_init(encoder1_CHA, encoder1_CHB, resolution, RISING);
	this->encoder2_init(encoder2_CHA, encoder2_CHB, resolution, RISING);
}

void XSpaceV21Board::encoder1_init(int enc_CHAx, int enc_CHBx, int resolutionx, int mode){
	
	encoder[E1].channelA = enc_CHAx;
	encoder[E1].channelB = enc_CHBx;
	encoder[E1].resolution = resolutionx;

	pinMode(encoder[E1].channelA, INPUT_PULLUP);
	pinMode(encoder[E1].channelB, INPUT_PULLUP);
	attachInterrupt(encoder[E1].channelA, ISR_encoder1, mode);
}
void XSpaceV21Board::encoder2_init(int enc_CHAx, int enc_CHBx, int resolutionx, int mode){
	encoder[E2].channelA = enc_CHAx;
	encoder[E2].channelB = enc_CHBx;
	encoder[E2].resolution = resolutionx;

	pinMode(encoder[E2].channelA, INPUT_PULLUP);
	pinMode(encoder[E2].channelB, INPUT_PULLUP);
	attachInterrupt(encoder[E2].channelA, ISR_encoder2, mode);
}

void XSpaceV21Board::DRV8837_init(int DRVx, int IN1x, int CH_IN1x, int IN2x, int CH_IN2x, int nSLEEPx,int frequencyx, int Vmx){
	DRV88xx[DRVx].IN1 = IN1x;
	DRV88xx[DRVx].IN2 = IN2x;
	DRV88xx[DRVx].nSLEEP = nSLEEPx;
	DRV88xx[DRVx].PWM_CH_IN1 = CH_IN1x;
	DRV88xx[DRVx].PWM_CH_IN2 = CH_IN2x;
	DRV88xx[DRVx].Vm = Vmx;

	pinMode(DRV88xx[DRVx].nSLEEP,OUTPUT);
	this->DRV8837_Sleep(DRVx);

	ledcSetup(DRV88xx[DRVx].PWM_CH_IN1, frequencyx, 10);
	ledcSetup(DRV88xx[DRVx].PWM_CH_IN2, frequencyx, 10);

	ledcAttachPin(DRV88xx[DRVx].IN1, DRV88xx[DRVx].PWM_CH_IN1);
	ledcAttachPin(DRV88xx[DRVx].IN2, DRV88xx[DRVx].PWM_CH_IN2);

}

void XSpaceV21Board::DRV8837_Sleep(){
	digitalWrite(DRV88xx[DRVx1].nSLEEP,LOW);
	digitalWrite(DRV88xx[DRVx2].nSLEEP,LOW);
}

void XSpaceV21Board::DRV8837_Sleep(int DRVx){
	digitalWrite(DRV88xx[DRVx].nSLEEP,LOW);
}

void XSpaceV21Board::DRV8837_Wake(){
	digitalWrite(DRV88xx[DRVx1].nSLEEP,HIGH);
	digitalWrite(DRV88xx[DRVx2].nSLEEP,HIGH);
}

void XSpaceV21Board::DRV8837_Wake(int DRVx){
	digitalWrite(DRV88xx[DRVx].nSLEEP,HIGH);
}

void XSpaceV21Board::DRV8837_Voltage(int DRVx, double Vp){

	if(Vp>DRV88xx[DRVx].Vm) Vp = DRV88xx[DRVx].Vm;
	if(Vp<-DRV88xx[DRVx].Vm) Vp = -DRV88xx[DRVx].Vm;
	uint32_t Duty = (uint32_t) ( (1 - abs(Vp)/DRV88xx[DRVx].Vm) * 1023.0);

	if(Vp<0){
		ledcWrite(DRV88xx[DRVx].PWM_CH_IN1, Duty);
		ledcWrite(DRV88xx[DRVx].PWM_CH_IN2, 1023);
	}else{
		ledcWrite(DRV88xx[DRVx].PWM_CH_IN1, 1023);
		ledcWrite(DRV88xx[DRVx].PWM_CH_IN2, Duty);
	}
}

double XSpaceV21Board::GetEncoderSpeed(int encx, int modo){
	double speed=0;
	double deltaX;
	speed = 360000000.0/(encoder[encx].resolution*Periodo[encx]);
	deltaX = abs(speed - encoder[encx].speed_ant);

	if(deltaX>200) speed = encoder[encx].speed_ant;
	encoder[encx].speed_ant = speed;

	switch (modo){
		case DEGREES_PER_SECOND:
			return speed;
			break;
		case RADS_PER_SECOND:
			return speed*PI/180;
			break;
		default:
			break;
	}

	return speed;
}
double XSpaceV21Board::GetEncoderPosition(int encx, int modo){
	double pos=0;

	switch (modo){
		
	case DEGREES:
		pos = counter[encx]/encoder[encx].resolution*360.0;
		break;
	case RADS:
		pos = counter[encx]/encoder[encx].resolution*2*PI;
		break;
	
	default:
		break;
	}

	return pos;
}

void XSpaceV21Board::BMI088_init(int accel_cs, int gyro_cs){
	bmi = new XSBMI088(SPI,accel_cs,gyro_cs);
	bmi->begin();
}

void XSpaceV21Board::BMI088_GetData(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
	bmi->readSensor(ax,ay,az,gx,gy,gz);
}

void XSpaceV21Board::BMI088_GetAccelData(float *ax, float *ay, float *az){
	bmi->readAccel(ax,ay,az);
}

void XSpaceV21Board::BMI088_GetGyroData(float *gx, float *gy, float *gz){
	bmi->readGyro(gx,gy,gz);
}

void XSpaceV21Board::BMI088_calibrateGyro(float *gx_offset, float *gy_offset, float *gz_offset){
	int numReadings = 1000;
	float gx,gy,gz;
	float gx_sum = 0;
	float gy_sum = 0;
	float gz_sum = 0;

	for (int i = 0; i < numReadings; i++) {
	// Retrieve the latest gyroscope data from the BMI088 sensor.
	this->BMI088_GetGyroData(&gx, &gy, &gz);

	gx_sum += gx;
	gy_sum += gy;
	gz_sum += gz;

	delay(2); // Small delay between readings
	}

	*gx_offset = gx_sum / numReadings;
	*gy_offset = gy_sum / numReadings;
	*gz_offset = gz_sum / numReadings;
}