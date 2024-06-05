/********************************************************************************
* 		 	Test The SPI_INPUT By Making an LoopBack Connection MOSI=MISO    	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

volatile uint16_t 	spi_in  = 0;			// Var Used For Holding the SPI_INPUT Value

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 10) == 0 ) { 						// Do This Every 10 ms

	spi_in = send_spi_data(0x55);						// Reads SPI periph  PA6=PA7=LoopBack Connection
	if (spi_in == 0x55) { gpio_out(GPIOC, 13, 0); } 	// If LoopBack Connection Then Led_On   PA6=PA7
	else { gpio_out(GPIOC, 13, 1); }					// NO LoopBack Connection Then Led_Off	

	}
}

void Hardware_Init(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 	72000/72_MHZ = 1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	spi_init();																// Initializes 	SPI PA4=LATCH, PA5=CLK, PA6=MISO, PA7=MOSI	
}	


int main(void){
	Hardware_Init();	
	for(;;) {	
	}
}


