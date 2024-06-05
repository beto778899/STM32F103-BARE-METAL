The examples in this Folder Initialize The I2C_PCF8574 Module. Using the I2C Protocol

* PCF8574_I2C_Binary_Counter 		An I2C Binary_Counter On the PCF8574 Port Expander	
* PCF8574_I2C_Read				      Example that Reads One Byte From the PCF8574(Address 0x27)
* PCF8574_I2C_Lcd_Init			    Initializes the I2C_LCD Module And Displays Some Chars
* PCF8574_I2C_Lcd_Putstr			  Displays Scrolling Text on the I2C_LCD Module

-- Read and Write To the I2C is done with this Functions. --

* i2c_init();									// Initializes the I2C   	PB6=CLK 	PB7=DATA
* i2c_write(0x27, v, 1); 						// Writes One Byte  to Address 0x27  uint8_t v[1] = {0x55};
* i2c_write(0x27, v, 2); 						// Writes Two Bytes to Address 0x27  uint8_t v[2] = {0x55,0xAF};

* int mybyte = i2c_read(0x27, 1) 				// Reads One Byte From the PCF8574(Address 0x27) 
* int mybyte = i2c_read(0x27, 1) & 1;			// Reads One Byte From the PCF8574(Address 0x27) With Masks BIT_0
