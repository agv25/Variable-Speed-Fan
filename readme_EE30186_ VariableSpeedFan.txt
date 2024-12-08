

						EE30186 - Integrated Engineering - Variable Fan Speed Coursework

									Name:	Alex Varney
									agv25
									Date:	14/12/21


	-------------------------------------------------------------------------------------------------------------------------------------------------------


									The Setup.


			Plug in the Altera DE1-SOC to a power socket and connect it to the computer via a USB cable.
			Connect the extension board using the 40way ribbon cable via the GPIO_0 socket on the FPGA board.
			A 12V power supply should be connected to the extension board's terminals.
			The 3-pin fan should be connected to the extension board.


	-------------------------------------------------------------------------------------------------------------------------------------------------------


									Starting the System.


			Compile the C code in the program Eclipse and connect to the FPGA board.
			Ensure all switches are switched off (downwards) and the 12V power supply is off before running.
			Turn the power supply on once the code has ran, the fan will remain stationary.
			The 7-segment display HEX2, HEX1, HEX0, will display "OFF", all other displays will be blank. 


	-------------------------------------------------------------------------------------------------------------------------------------------------------


									Switches.


		SW9 will allow you to toggle the system ON and OFF, this overrides all other switches.

		SW8 will allow you to toggle between an open-loop and closed-loop method of controlling fan speed.
		Whilst LOW, the open-loop method is selected, "oL" is displayed on HEX5 & HEX4.
		Note, during open-loop control the inaccuracy of measured rpm will range from 0rpm at high duty cycles to 300rpm at low duty cycles.
		Whilst HIGH, the closed-loop method is selected, "cL" is displayed on HEX5 & HEX4.
		Note, during closed-loop control the rpm measured will oscillate around the desired/expected rpm. 

		SW7 will allow you to change the mode of how the fan's speed is controlled.
		Whilst LOW only the rotary encoder can be used to control the fan's speed.
		Whilst HIGH only SW6 and SW5 can be used to control the fan's speed.

		SW6 and SW5 will not work if SW7 is LOW. 
		SW6 and SW5 will each provide 50% power to the fan.
		For example, if either one of the switches are HIGH and the other is LOW, the fan will run at 50% speed.
		If both are HIGH, the fan will run at 100% speed and if both are LOW, the fan will stop.

		SW0 will allow you to toggle between what speed is displayed on HEX3, HEX2, HEX1, HEX0.
		Whilst LOW, the duty cycle percentage of the fans set speed will be displayed.
		Whilst HIGH, an rpm speed will be displayed depending on the state of SW1.

		SW1 only works when SW0 is HIGH to display the rpm speeds. 
		Whilst HIGH the expected rpm speed (calculated from the applied duty cycle) will be shown.
		Whilst LOW the measured rpm speed from the tachometer will be shown.


	-------------------------------------------------------------------------------------------------------------------------------------------------------


							Buttons.


		Pressing buttons will allow you to select different rotary encoder sensitivities.
		KEY0 will select a sensitivity of +/-1% of the duty cycle (default).
		KEY1 will select a sensitivity of +/-3% of the duty cycle.
		KEY2 will select a sensitivity of +/-5% of the duty cycle.
		KEY3 will select a sensitivity of +/-10% of the duty cycle.


	-------------------------------------------------------------------------------------------------------------------------------------------------------


							Rotary Encoder.


		The rotary encoder on the extension board will allow you to adjust the fan's speed linearly with respect to its duty cycle.
		Rotating it clockwise will increase the speed at the selected encoder sensitivity.
		Rotating it anti-clockwise will decrease the speed at the selected encoder sensitivity.
		In open-loop, when you increase the speed by any amount from 0%, the duty cycle will jump to 30% to avoid any stalling at low duty cycles.
		In open-loop when you decrease the speed by any amount below 14%, the duty cycle will jump to 0% to also avoid any stalling.
		Note, this method of fan control will only work whilst SW7 is LOW.


	-------------------------------------------------------------------------------------------------------------------------------------------------------

							LEDs.

		The LEDs will light up from left to right depending on the set duty cycle.
		No LEDS lit at 0%.
		From left (LEDR9) to right (LEDR0), one extra LED will light up every +10%.
		All LEDs lit at 100%.

