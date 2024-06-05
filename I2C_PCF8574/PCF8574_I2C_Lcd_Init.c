/********************************************************************************
* 		 	Inits the I2C_LCD Module And Display Some Chars	    				*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

void Write_Lcd(int data);
void Lcd_Init(void);

volatile uint16_t 	counter  = 0;			// Var Used For Counter Loop, Goes To The Value Of spi
uint8_t v[1] = { 0x00 };					// Var Used For Holding i2c Value
int i,j;

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 50) == 0 ) { 
	gpio_Toggle(GPIOC, 13); 												// Toggles PC13 Every 500ms    		
	counter++; Write_Lcd(0x49);Write_Lcd(0x39);
	send_spi_data_16(1, counter/4);						
	}
}

void Write_Lcd(int data){
	data |= 0x0C;							// E. Latch = P2 = 0100		Light = P3 = 1000   P2+P3 = 0000 1100
	v[0] = data; i2c_write(0x27, v, 1);	
	for (j = 0; j < 1670; j++) {}			// Delay 0.4ms
	data &= 0xFB;							// falling edge of E. Latch = 0	= 0000  				1111 1011
	v[0] = data; i2c_write(0x27, v, 1);	
	for (j = 0; j < 1670; j++) {}			// Delay 0.4ms
}

void Lcd_Init(void){
	for (i = 0; i < 90000; i++) {}			// Allow LCD time to powerup 20ms
	uint8_t lcd[14] = { 0x30,0x30,0x30,0x20,0x20,0x80,0x00,0xC0,0x00,0x10,0x00,0x60,0x00,0x20 };
	for (i = 0; i < 14; i++) { 
		Write_Lcd(lcd[i]); 	
	}	
}	

void MAX7219_Init(void){
	spi_init();																// Initializes 		SPI 	PA4=LATCH 	PA5=CLK   PA7=MOSI	
	send_spi_data_16(0x0F, 0x00); 											// MAX7219 INIT		Test_Off
	send_spi_data_16(0x0A, 0x07); 											// MAX7219 INIT		Med_Intensity (00-0F)
	send_spi_data_16(0x0B, 0x07); 											// MAX7219 INIT 	Scan_Limit = 8_Columns
	send_spi_data_16(0x0C, 0x01); 											// MAX7219 INIT		Power_On
	for (j = 0; j < 1040; j++) {}											// Wait 0.25ms
	for (i = 1; i < 9; i++) {  send_spi_data_16(i, 0x00);  }				// CLRS ALL LEDS
}

void Hardware_Init(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	MAX7219_Init();
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	i2c_init();																// Initializes 	I2C   	PB6=CLK 	PB7=DATA
	Lcd_Init();		
	Write_Lcd(0x41);Write_Lcd(0x81);
	Write_Lcd(0x41);Write_Lcd(0x51);
	Write_Lcd(0x49);Write_Lcd(0x89);	
	Write_Lcd(0x49);Write_Lcd(0x59);
	systick_init(720000, 1);													// Initializes 	systick WITH 10ms interrupt 	720000/72_MHZ=10ms	
}	

int main(void){
	Hardware_Init();
	for(;;) {	
	}
}


