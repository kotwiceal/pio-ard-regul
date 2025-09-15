#include <Arduino.h>

#define BOARDRATE 115200

// define FSM
#define DIGITAL_OUT_WRITE 1
#define DIGITAL_OUT_READ 2
#define ANALOG_READ 3
#define DIGITAL_IN_READ 4
#define REGULATOR_ENABLE 5
#define REGULATOR_COEF_WRITE 6
#define REGULATOR_COEF_READ 7
#define REGULATOR_THRESHOLD_X_WRITE 8
#define REGULATOR_THRESHOLD_X_READ 9
#define REGULATOR_DISCREPANCY_X_WRITE 10
#define REGULATOR_DISCREPANCY_X_READ 11
#define REGULATOR_THRESHOLD_Y_WRITE 12
#define REGULATOR_THRESHOLD_Y_READ 13
#define REGULATOR_X_READ 14
#define REGULATOR_Y_READ 15
#define REGULATOR_FIR_READ 16
#define REGULATOR_NUMBER 17
#define REGULATOR_BUFFER_LENGTH 18
#define REGULATOR_DEADTIME_READ 19
#define REGULATOR_DEADTIME_WRITE 20
char buffer[4];
int bytesRead = 0;
int8_t command = 0, address = 0;
int16_t data = 0;

// define general purpose input digital pins
const int8_t gpin = 3;
int8_t gpinMask[gpin] = {8, 9, 10};
bool gpinValue[gpin] = {0, 0, 0};

// define general purpose output digital pins
const int8_t gpout = 4;
int8_t gpoutMask[gpout] = {4, 5, 6, 7};
bool gpoutValue[gpout] = {0, 0, 0, 0};

// define regulator parameters
const uint8_t reg_num = 3;
const uint8_t reg_buf_len = 4;
uint8_t reg_ptr = 0;
uint16_t reg_buf[reg_num][reg_buf_len];
int16_t reg_coef[reg_num][reg_buf_len];
uint8_t reg_mask[reg_buf_len];
uint16_t reg_thresh_x[reg_num];
int16_t reg_thresh_y[reg_num];
int16_t reg_fir_res[reg_num];
uint16_t reg_disc_x[reg_num];
bool reg_en[reg_num];

// define digital pins of regulator
int8_t pinControlMask[reg_num] = {2, 12, 13};
bool pinControlValue[reg_num] = {0, 0, 0};

// define analog pins of regulator
int8_t pinSignalMask[reg_num] = {A0, A1, A2};
uint16_t pinSignalValue[reg_num] = {0, 0, 0};

// define timer of trigger schmitt
uint32_t time[reg_num], deadtime[reg_num];
bool trigger[reg_num];

float fir(uint16_t x[], int16_t coef[], uint8_t mask[], const uint8_t len) {
	float y = 0;
	for (uint8_t i = 0; i < len; i++) {
		y = y + x[i]*coef[mask[i]];
	}
	return y;
}

void regulator() {
	// circular shift
	for (uint8_t i = 0; i < reg_buf_len; i++) {
    if ((reg_ptr+i) < reg_buf_len) {
      reg_mask[reg_ptr+i] = i;
    } else {
      reg_mask[reg_ptr+i-reg_buf_len] = i;
    }
	}
	// consequence ADC acquisition
	for (uint8_t i = 0; i < reg_num; i++) {
		pinSignalValue[i] = analogRead(pinSignalMask[i]);
		// accumulate buffer
		reg_buf[i][reg_ptr] = pinSignalValue[i];
		delay(1);
	};
	// circular increment
	if (reg_ptr == reg_buf_len - 1) {
		reg_ptr = 0;
	} else {
		reg_ptr = reg_ptr + 1;
	}
	for (uint8_t i = 0; i < reg_num; i++) {
		if (!trigger[i]) {
			pinControlValue[i] = !(int16_t(pinSignalValue[i] - reg_thresh_x[i]) > int16_t(reg_disc_x[i])) & reg_en[i];
			digitalWrite(pinControlMask[i], pinControlValue[i]);
			trigger[i] = true;
			time[i] = 0;
		} else {
			if (time[i] > deadtime[i]) {
				trigger[i] = false;
			}
		}
	};
}

void com() {
	// serial port
	if (Serial.available() > 0) {
		// read buffer
		bytesRead = Serial.readBytes(buffer, sizeof(buffer)); 

		// parse packet
		command = buffer[0];
		address = buffer[1];
		data = ((buffer[2] & 255) << 8) | (buffer[3] & 255);

		// FSM
		switch (command) {
			case DIGITAL_IN_READ:
				gpinValue[address] = digitalRead(gpinMask[address]);
				data = gpinValue[address];
				break;
			case DIGITAL_OUT_WRITE:
				gpoutValue[address] = bool(data);
				digitalWrite(address, gpoutValue[address]);
				break;
			case DIGITAL_OUT_READ:
				data = gpoutValue[address];
				break;
			case ANALOG_READ:
				data = pinSignalValue[address];
				break;
			case REGULATOR_ENABLE:
				reg_en[address] = bool(data);
				break;
			case REGULATOR_COEF_WRITE:
				reg_coef[address >> 4][address & 15] = data;
				break;
			case REGULATOR_COEF_READ:
				data = reg_coef[address >> 4][address & 15];
				break;
			case REGULATOR_THRESHOLD_X_WRITE:
				reg_thresh_x[address] = data;
				break;
			case REGULATOR_THRESHOLD_X_READ:
				data = reg_thresh_x[address];
				break;
			case REGULATOR_DISCREPANCY_X_WRITE:
				reg_disc_x[address] = data;
				break;
			case REGULATOR_DISCREPANCY_X_READ:
				data = reg_disc_x[address];
				break;
			case REGULATOR_THRESHOLD_Y_WRITE:
				reg_thresh_y[address] = data;
				break;
			case REGULATOR_THRESHOLD_Y_READ:
				data = reg_thresh_y[address];
				break;
			case REGULATOR_X_READ:
				data = pinSignalValue[address];
				break;
			case REGULATOR_Y_READ:
				data = pinControlValue[address];
				break;
			case REGULATOR_FIR_READ:
				data = reg_fir_res[address];
				break;
			case REGULATOR_NUMBER:
				data = int16_t(reg_num);
				break;
			case REGULATOR_BUFFER_LENGTH:
				data = int16_t(reg_buf_len);
				break;
			case REGULATOR_DEADTIME_READ:
				deadtime[address] = data;
				break;
			case REGULATOR_DEADTIME_WRITE:
				data = deadtime[address];
				break;
			default:
				break;
		}

		// split bits
		buffer[2] = (uint8_t) (data >> 8);
		buffer[3] = (uint8_t) (data);

		// transmit response
		Serial.write(buffer, sizeof(buffer));
	}
}

void setup() {
	Serial.begin(BOARDRATE);
	// initialize general purpose input digital pins
	for (int i = 0; i < gpin; i++) {pinMode(gpinMask[i], INPUT);}
	// initialize general purpose output digital pins
	for (int i = 0; i < gpout; i++) 
		{pinMode(gpoutMask[i], OUTPUT); digitalWrite(gpoutMask[i], gpoutValue[i]);}
	// iniitalize control output digital pins
	for (int i = 0; i < reg_num; i++) 
		{pinMode(pinControlMask[i], OUTPUT); digitalWrite(pinControlMask[i], pinControlValue[i]);}
	// initialize regulator
	for (uint8_t i = 0; i < reg_num; i++) {
		reg_thresh_x[i] = 0;
    	reg_thresh_y[i] = 0;
		reg_disc_x[i] = 0;
		reg_en[i] = false;
    	reg_fir_res[i] = 0;
		for (uint8_t j = 0; j < reg_buf_len; j++) {
      		reg_mask[j] = j;
			reg_buf[i][j] = 0;
			reg_coef[i][j] = 0;
		}
	}
	// initialize timer
	for (uint8_t i = 0; i < reg_num; i++) {
		time[i] = 0;
		deadtime[i] = 500;
		trigger[i] = false;
	}
}

void loop() {
	regulator();
	com();
	// update timers
	delay(1);
	for (uint8_t i = 0; i < reg_num; i++) {
		time[i] = time[i] + 1;
	}
}