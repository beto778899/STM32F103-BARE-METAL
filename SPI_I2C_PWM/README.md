This example Sends a value to the SPI, PWM, I2C, LED_SPEED Every 250ms

A Counter Loop From 0-255 is implemented using an IF statement. 

Tested with the PicSimlab and with Proteus. using a Virtual oscilloscope.


-- Functions that Sends the values --
send_spi_data(counter);   											// INCR_SPI_DATA  Every 250ms
v[0] = counter; i2c_write(0x27, v, 1);   							// INCR_I2C_DATA  Every 250ms
TIM2->CCR2     = (counter * 100);									// INCR_PWM_DUTY  Every 250ms
LED_DELAY_PC13 = counter;  											// INCR_LED_DELAY Every 250ms