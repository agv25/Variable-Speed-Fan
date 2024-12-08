/*
 * systemStructs.h
 *
 *  Created on: 29 Nov 2021
 *      Author: Alex Varney
 */

#ifndef SYSTEMSTRUCTS_H_
#define SYSTEMSTRUCTS_H_

#define key0 0xE
#define key1 0xD
#define key2 0xB
#define key3 0x7

#define HIGH 1
#define LOW 0

#define OFF 0
#define ON 1
#define OPEN 2
#define CLOSED 3

//Variables related to the FPGA
struct fpgaStuff {
	int state; // ON or OFF state of system
	int loop; // selected loop method of control
	volatile int * hex3to0; // right most 7-segment displays
	volatile int * hex5to4; // left most 7-segment displays
	volatile int * switches; // FPGA switches
	volatile int * pressButt; // FPGA buttons
	volatile int * LED; // FPGA LEDs
	volatile unsigned int * counter; // FPGA inbuilt counter (50MHz)
};

//Variables related to the fan
struct fanStuff {
	volatile int * GPIO; // GPIO reference to fan
	volatile int * GPIOdir; // GPIO fan direction of information
	int setDutyCycle; // set speed in percentage duty cycle
	int actualDutyCycle; // actualDutyCycle running the fan
	int rpmSpeed; // actual fan speed in rpm
	float pidOutput; // pid output
	int PWMstate; // state of pwm signal high or low
};

//Variables related to the rotary encoder
struct encoderStuff {
	volatile int * GPIO; //GPIO reference to encoder pin
	int pinA; // pin 17
	int pinB; // pin 19
	int GrayCode; // gray codes of pinA & pinB
	int oldGrayCode; // previous gray code
	int direction; // direction that encoder has turned
	int encSens; // selected sensitivity of the encoder
};


#endif /* SYSTEMSTRUCTS_H_ */
