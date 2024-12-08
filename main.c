/*
 * main.c
 *
 *	main program for
 *	EE30186 Coursework - Variable Speed Fan Control
 *
 *  Created on: 29 Nov 2021
 *      Author: Alex Varney
 */

#include "EE30186.h"
#include "system.h"
#include "socal/socal.h"
#include <inttypes.h>
#include <stdio.h>
#include "systemStructs.h"


//Define functions before main() so no declarations have to be made
//******************************************************************************************************//
//******************************************************************************************************//

//Starting function to initialise all structs and variables
void startSystem(struct fpgaStuff *fpga, struct fanStuff *fan, struct encoderStuff *enc)
{
	//Initialise fpga structs variables
	fpga->state = OFF;
	fpga->loop = OPEN;
	fpga->hex3to0 = (volatile int *)(ALT_LWFPGA_HEXA_BASE);
	fpga->hex5to4 = (volatile int *)(ALT_LWFPGA_HEXB_BASE);
	fpga->pressButt = (volatile int *)(ALT_LWFPGA_KEY_BASE);
	fpga->switches = (volatile int *)(ALT_LWFPGA_SWITCH_BASE);
	fpga->LED = (volatile int *)(ALT_LWFPGA_LED_BASE);
	fpga->counter = (volatile unsigned int *)(ALT_LWFPGA_COUNTER_BASE);

	//Initialise fan structs variables
	fan->GPIO = (volatile int *)(ALT_LWFPGA_GPIO_0A_BASE);
	fan->GPIOdir = (fan->GPIO) + 1;
	*(fan->GPIOdir) = 8;
	fan->setDutyCycle = 0;
	fan->actualDutyCycle = 0;
	fan->rpmSpeed = 0;
	fan->pidOutput = 0;

	//Initialise encoder struct variables
	enc->GPIO = (volatile int *)(ALT_LWFPGA_GPIO_0A_BASE);
	enc->direction = 0;
	enc->encSens = 1;
}

//******************************************************************************************************//

//Function that checks if system is ON/OFF and closed-loop/open-loop and method of speed control
void checkSystem(volatile int switches, struct fpgaStuff *fpga, struct fanStuff *fan)
{
	//Initialise functions variables
	volatile int switchOnOff = (switches >> 9) & 0b1; //SW9
	volatile int switchLoop = (switches >> 8) & 0b1; //SW8
	volatile int switchMode = (switches >> 7) & 0b1; //SW7
	volatile int switchSpeed1 = (switches >> 6) & 0b1; //SW6
	volatile int switchSpeed2 = (switches >> 5) & 0b1; //SW5


	//If SW9 is high, system is on
	if(switchOnOff == HIGH)
	{
		fpga->state = ON;

		//Fan on & SW8 high, closed-loop
		if(switchLoop == HIGH)
		{
			fpga->loop = CLOSED;
		}
		//Fan on & SW8 low, open-loop
		else if(switchOnOff == HIGH)
		{
			fpga->loop = OPEN;
			fan->pidOutput = 0;
		}

		//Switch mode of control - using switches rather than encoder
		if(switchMode == HIGH)
		{
			//Both speed switches SW6 & SW5 high, max speed
			if(switchSpeed1 == HIGH && switchSpeed2 == HIGH)
			{
				fan->setDutyCycle = 100;
				fan->actualDutyCycle = 100;
			}
			//Either speed switch on and the other off, 50% speed
			else if(switchSpeed1 != switchSpeed2)
			{
				fan->setDutyCycle = 50;
				fan->actualDutyCycle = 50;
			}
			//Both speed switches low, stop fan
			else
			{
				fan->setDutyCycle = 0;
				fan->actualDutyCycle = 0;
				fan->pidOutput = 0;
			}
		}
	}
	//SW9 low overrides system and turns off
	else
	{
		fan->setDutyCycle = 0;
		fan->pidOutput = 0;
		fan->actualDutyCycle = 0;
		fan->rpmSpeed = 0;
		fpga->state = OFF;
	}
}

//******************************************************************************************************//

//Function to read rotary encoder and return its changing direction
void readEncoder(struct encoderStuff *enc)
{
	//Read encoder inputs on pin A & B
	enc->pinA = 0x2 & *(enc->GPIO) >> 16;
	enc->pinB = 0x1 & *(enc->GPIO) >> 19;

	//Link pinA and pinB to get gray code
	enc->GrayCode = enc->pinA | enc->pinB;

	//3-1-0-2-3 -> clockwise turn = increase speed, direction = 1
	//3-2-0-1-3 -> anti-clockwise turn = decrease speed, direction = -1
	//No change in encoder, direction = 0

	//Use old and current grayCodes to check for clockwise or anti-clockwise
	if(enc->oldGrayCode == 0)
	{
		if(enc->GrayCode == 0)
		{
			enc->direction = 0;       //0->0 , direction = 0
		}
		else if(enc->GrayCode == 1)
		{
			enc->direction = -1;      //0->1 , direction = -1
		}
		else if(enc->GrayCode == 2)
		{
			enc->direction = 1;       //0->2 , direction = 1
		}
	}
	else if(enc->oldGrayCode == 1)
	{
		if(enc->GrayCode == 1)
		{
			enc->direction = 0;       //1->1 , direction = 0
		}
		else if(enc->GrayCode == 0)
		{
			enc->direction = 1;       //1->0 , direction = 1
		}
		else if(enc->GrayCode == 3)
		{
			enc->direction = -1;      //1->3 , direction = -1
		}
	}
	else if(enc->oldGrayCode == 2)
	{
		if(enc->GrayCode == 2)
		{
			enc->direction = 0;       //2->2 , direction = 0
		}
		else if(enc->GrayCode == 0)
		{
			enc->direction = -1;      //2->0 , direction = -1
		}
		else if(enc->GrayCode == 3)
		{
			enc->direction = 1;       //2->3 , direction = 1
		}
	}
	else if(enc->oldGrayCode == 3)
	{
		if(enc->GrayCode == 3)
		{
			enc->direction = 0;       //3->3 , direction = 0
		}
		else if(enc->GrayCode == 2)
		{
			enc->direction = -1;      //3->2 , direction = -1
		}
		else if(enc->GrayCode == 1)
		{
			enc->direction = 1;       //3->1 , direction = 1
		}
	}
	//Change the gray code to the old gray code for next iteration
	enc->oldGrayCode = enc->GrayCode;
}

//******************************************************************************************************//

//Function to check selected sensitivity of encoder
void checkSens(volatile int **pressButt, int *encSens)
{
	//Switch cases based on last key pressed
	switch (**pressButt)
	{
	//Press key0 for +/-1% sensitivity
	case key0:
		*encSens = 1;
		break;

	//Press key1 for +/-3% sensitivity
	case key1:
		*encSens = 3;
		break;

	//Press key2 for +/-5% sensitivity
	case key2:
		*encSens = 5;
		break;

	//Press key3 for +/-10% sensitivity
	case key3:
		*encSens = 10;
		break;
	}
}

//******************************************************************************************************//

//Function to create the PWM signal to drive the fan
void createPWM(struct fpgaStuff *fpga, struct fanStuff *fan)
{
	//Set cycle of value 0-100 based on frequency (1000Hz)
	volatile int cycle = (*(fpga->counter)/50000)%101; // remainder for time on/off relative to period

	//If dutyCycle less than cycle time
	if(fan->actualDutyCycle < cycle)
	{
		//Output signal low
		*(fan->GPIO) = 0x0;
		fan->PWMstate = LOW;
	}
	else
	{
		//Output signal high
		*(fan->GPIO) = 0x8;
		fan->PWMstate = HIGH;
	}
}

//******************************************************************************************************//

//Function to drive fan using open-loop method
void driveFan(struct fanStuff *fan, struct encoderStuff *enc)
{
	static int oldDutyCycle; //previous duty cycle

	//Drive fan with respect to duty cycle
	fan->setDutyCycle = fan->setDutyCycle + ((enc->direction) * (enc->encSens));

	//Prevent duty cycle greater than 100 and less than 0
	fan->setDutyCycle = (fan->setDutyCycle > 100.0) ? 100.0 : fan->setDutyCycle;
	fan->setDutyCycle = (fan->setDutyCycle < 0.0) ? 0.0 : fan->setDutyCycle;


	//Statements to avoid running Duty Cycles too little for fan to run
	//Bearing in mind threshold is lower for moving fan than stationary fan
	if(fan->setDutyCycle == 0)
	{
		fan->actualDutyCycle = 0;
		fan->pidOutput = 0;
		fan->rpmSpeed = 0;
	}
	else if((fan->setDutyCycle > 0) && (fan->setDutyCycle < 12) && (fan->setDutyCycle > oldDutyCycle))
	{
		fan->setDutyCycle = 30; // bounce up to 30% duty cycle so fan does not stall to begin with
		fan->actualDutyCycle = 30;
	}
	else if((fan->setDutyCycle < 14) && (fan->setDutyCycle < oldDutyCycle))
	{
		fan->setDutyCycle = 0; // bounce down to 0% duty cycle if below 14 to avoid decrementing to a stall condition
		fan->actualDutyCycle = 0;
		fan->rpmSpeed = 0;
	}

	//Shift to old duty cycle
	oldDutyCycle = fan->setDutyCycle;
}

//******************************************************************************************************//

//Function to implement closed-loop control
void controlPID(struct fpgaStuff *fpga, struct fanStuff *fan)
{
	//Initialise PID variables
	float p = 0.000000000000009; // proportional constant
	float i = 0.0000000003; // integral constant
	float d = 0.000000003; // derivative constant
	static int oldCount;
	int abs();

	// Desired rpm from PWM input
	int rpmDesired = ((fan->setDutyCycle)*1800)/100;

	//Calculate iteration time for pid using counter
	int newCount = *(fpga->counter);
	int dt = abs(newCount - oldCount);
	oldCount = newCount;

	//Calculate error term
	int error = (rpmDesired) - (fan->rpmSpeed);


	//Calculate output for pid computation
	fan->pidOutput += ((p*error) + (i*error)*dt + (d*error)/dt);

}

//******************************************************************************************************//

//Function to read tachometer for feedback rpm
void readTacho(struct fpgaStuff *fpga, struct fanStuff *fan)
{
	//Declare variables
	int newTacho, newCount;
	static int oldTacho, oldCount, pulses, pulsePerSec;
	const int clock = 50000000;

	//Read tachometer signal
	newTacho = (0x1 & (*(fan->GPIO) >> 1));

	//Read counter from FPGA clock
	newCount = *(fpga->counter)/clock;

	//Find pulses detected in one second
	if(newCount != oldCount)
	{
		pulsePerSec = pulses;
		pulses = 0;
	}
	else
	{
		//Falling edge detected 1->0
		if((oldTacho == 1) && (newTacho == 0))
		{
			++pulses; //accumulate pulse count
		}
	}

	//Shift new values to old for next iteration
	oldCount = newCount;
	oldTacho = newTacho;

	//Calculate rpm, divide by 2 as 2 pulses per revolution
	fan->rpmSpeed = (pulsePerSec*60)/2;
}

//******************************************************************************************************//

//Function to decode seven segment display
int sevenSegDec(int digit)
{
	//Array with segment values for digits 0-9 & blank
	int hexVal[11] = {0x40,0xF9,0x24,0x30,0x19,0x12,0x02,0xF8,0x00,0x10,0xFF};

	//Return blank if invalid digit used
	if ((digit < 0) || (digit > 9)) return hexVal[10];

	//Use digit as index in segments array
	return hexVal[digit];
}

//******************************************************************************************************//

//Function that takes more than one digit and uses sevenSegDec to decode a multidigit
int multiDigitDec(int value)
{
	//Initialise returnVal as blank
	int returnVal = 0xFFFFFFFF;

	//Current digit to keep track
	int currentDigit = 0;

	//Temporary variable to store extracted values
	int singleDigitDisp;

	//Loop through digits
	do
	{
		//Extract bottom digit
		singleDigitDisp = sevenSegDec(value%10);

		//Adjust the input value to reflect the extraction of the bottom right
		value /= 10;

		//Clear the space for decoder result
		returnVal = returnVal & ~(0xFF << (currentDigit * 8));

		//Shift single decoded digit to the right place
		returnVal = returnVal | (singleDigitDisp << (currentDigit * 8));

		//Update digit position so if value is non-zero
		// next digit is 8 bits to the left
		currentDigit++;

	} while(value > 0);

	//Return multidigit result
	return returnVal;
}

//******************************************************************************************************//

//Function that sets all 7 segment displays correctly
void finalDisplay(volatile int switches, struct fpgaStuff *fpga, struct fanStuff *fan)
{
	//Initialise the switch variable rpm
	volatile int showRPM = (switches) & 0b1; //get switch SW0
	volatile int expectedRPM = (switches >> 1) & 0b1; //get switch SW1

	//Calculate desired rpm for display
	int rpmDesired = (fan->setDutyCycle)*1800/100;

	//If state is off display "OFF" on hex3to0
	if(fpga->state == OFF)
	{
		*(fpga->hex5to4) = (0xFF << 8) | (0xFF); //display blank
		*(fpga->hex3to0) = (0xFF << 24) | (0x40 << 16) | (0xE << 8) | (0xE); //display "OFF"
	}

	else
	{
		//If showRPM switch high and expectedRPM switch high, display measured RPM on hex3to0
		if((fpga->state == ON && showRPM == HIGH && expectedRPM == LOW))
		{
			*(fpga->hex3to0) = multiDigitDec(fan->rpmSpeed); //show measured RPM multi digit
		}

		//If both showRPM and expectedRPM switch high, show desired rpm on hex3to0
		else if(fpga->state == ON && showRPM == HIGH && expectedRPM == HIGH)
		{
			*(fpga->hex3to0) = multiDigitDec(rpmDesired); //show expected RPM multi digit
		}

		//If showRPM switch off, display percentage speed on hex3to0
		else if(fpga->state == ON && showRPM == LOW)
		{
			*(fpga->hex3to0) = multiDigitDec(fan->setDutyCycle); //show fan speed duty cycle in %
		}

		//Display closed loop or open loop on hex5to4
		if(fpga->loop == OPEN)
		{
			*(fpga->hex5to4) = (0xA3 << 8) | (0x47); //display "oL" open loop
		}
		else if(fpga->loop == CLOSED)
		{
			*(fpga->hex5to4) = (0xA7 << 8) | (0x47); //display "cL" closed loop
		}
	}
}

//******************************************************************************************************//

//Function to light up LEDs depending on set Duty Cycle
//0% -> no LEDs on, to  100% -> all LEDs on
// left to right
void lightUP(volatile int **LED, int *setDutyCycle)
{
	// Light up
	if(*setDutyCycle == 100)
	{
		**(LED) = 0x3FF;    // all 10 LEDs on
	}
	else if((*setDutyCycle <= 99) && (*setDutyCycle >= 90))
	{
		**(LED) = 0x3FE;    // 9 LEDs on
	}
	else if((*setDutyCycle <= 89) && (*setDutyCycle >= 80))
	{
		**(LED) = 0x3FC;    // 8 LEDs on
	}
	else if((*setDutyCycle <= 79) && (*setDutyCycle >= 70))
	{
		**(LED) = 0x3F8;    // 7 LEDs on
	}
	else if((*setDutyCycle <= 69) && (*setDutyCycle >= 60))
	{
		**(LED) = 0x3F0;    // 6 LEDs on
	}
	else if((*setDutyCycle <= 59) && (*setDutyCycle >= 50))
	{
		**(LED) = 0x3E0;    // 5 LEDs on
	}
	else if((*setDutyCycle <= 49) && (*setDutyCycle >= 40))
	{
		**(LED) = 0x3C0;    // 4 LEDs on
	}
	else if((*setDutyCycle <= 39) && (*setDutyCycle >= 30))
	{
		**(LED) = 0x380;    // 3 LEDs on
	}
	else if((*setDutyCycle <= 29) && (*setDutyCycle >= 20))
	{
		**(LED) = 0x300;    // 2 LEDs on
	}
	else if((*setDutyCycle <= 19) && (*setDutyCycle >= 10))
	{
		**(LED) = 0x200;    // 1 LED on
	}
	else if(*setDutyCycle < 10)
	{
		**(LED) = 0x0;      // all LEDs off
	}
}


//******************************************************************************************************//
//******************************************************************************************************//
//**************************				Main Function				********************************//
//******************************************************************************************************//
//******************************************************************************************************//


//Main function running all functions through the FPGA
int main(int argc, char** argv)
{
	//Function call to initialise FPGA configuration
	EE30186_Start();
	printf("Fan Initialised...");

	//Declare structs
	struct fpgaStuff fpga;
	struct fanStuff fan;
	struct encoderStuff enc;

	//Function to start-up the system
	startSystem(&fpga, &fan, &enc);

	//While loop running functions continuously
	while(1)
	{
		//Check ON/0FF/closed-loop/open-loop
		checkSystem(*(fpga.switches), &fpga, &fan);

		//Check encoder sensitivity
		checkSens(&(fpga.pressButt), &(enc.encSens));

		//Read direction of rotary encoder change
		readEncoder(&enc);

		//Read Tachometer for feedback
		//Only if PWMstate is high for more accurate feedback
		if(fan.PWMstate == HIGH)
		{
			readTacho(&fpga, &fan);
		}
		else{}

		//Decide on method of fan control
		if(fpga.loop == OPEN)
		{
			//Drive fan open loop
			driveFan(&fan, &enc);
			fan.actualDutyCycle = fan.setDutyCycle;
		}
		else
		{
			//Drive the fan closed loop
			driveFan(&fan, &enc);
			controlPID(&fpga, &fan);
			fan.actualDutyCycle = fan.pidOutput;
		}

		//Create a PWM signal and apply to fan
		createPWM(&fpga, &fan);

		//Set final displays
		finalDisplay(*(fpga.switches), &fpga, &fan);

		//LED light up
		lightUP(&(fpga.LED), &(fan.setDutyCycle));

	}

	//Function call to close FPGA configuration
	EE30186_End();

	return 0;
}


