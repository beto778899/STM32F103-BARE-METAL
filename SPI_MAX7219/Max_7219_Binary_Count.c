/********************************************************************************
* 		 	Binary_Count_All_Leds On the MAX7219 8x8 Led Matrix			    	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

volatile uint16_t 	counter  = 0;			// Var Used For Counter Loop, Goes To The Value Of spi
int i,j;

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 100) == 0 ) { 
	gpio_Toggle(GPIOC, 13); 												// Toggles PC13 Every 100ms
	counter++;
	for(i = 0; i < 8; i++) { send_spi_data_16(i+1, counter); }				// Binary Count All_Leds On MAX7219	
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
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 	72000/72_MHZ = 1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	MAX7219_Init();															// Inits MAX7219 8x8 Led Matrix
	
	for(;;) {	
	}
}


