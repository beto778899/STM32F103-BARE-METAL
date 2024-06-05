/********************************************************************************
* 		 	Show Fonts(Numbers 0-9)  On 	MAX7219	8x8 Led Matrix		    	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

volatile uint16_t 	counter  = 0;			// Var Used For Counter Loop
int i,j;

// Font Numbers 0-9 	8x8 Format
const uint8_t font[][8] = 
{
	{ 0x7C,0xCE,0xDE,0xF6,0xE6,0xC6,0x7C,0x00, },		// 0			
	{ 0x30,0x70,0x30,0x30,0x30,0x30,0xFC,0x00, },		// 1		
	{ 0x7C,0xC6,0x06,0x1C,0x70,0xC0,0xFE,0x00, },		// 2		
	{ 0x7C,0xC6,0x06,0x3C,0x06,0xC6,0x7C,0x00, },		// 3	
	{ 0x0C,0xCC,0xCC,0xCC,0xFE,0x0C,0x0C,0x00, },		// 4
	{ 0xFE,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00, },		// 5
	{ 0x3C,0x60,0xC0,0xFC,0xC6,0xC6,0x7C,0x00, },		// 6
	{ 0xFE,0x04,0x0C,0x18,0x30,0x30,0x30,0x00, },		// 7
	{ 0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0x7C,0x00, },		// 8
	{ 0x7C,0xC6,0xC6,0x7E,0x06,0x0C,0x78,0x00, },		// 9	
};

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 500) == 0 ) { 
	gpio_Toggle(GPIOC, 13); 												// Toggles PC13 Every 500ms    	
	
	for(i = 0; i < 8; i++) { send_spi_data_16(i+1, font[counter][i]); }		// Sends Font(Numbers 0-9) To MAX7219	
	counter++; if( counter > 9 ) { counter = 0; }							// Incrementes counter And Caps It
	
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

int main(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 	72000/72_MHZ=1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	MAX7219_Init();															// Inits MAX7219 8x8 Led Matrix
	
	for(;;) {	
	}
}


