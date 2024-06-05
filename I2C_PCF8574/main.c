/********************************************************************************
* 		 	I2C Binary_Counter On the PCF8574 Port Expander				    	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

volatile uint16_t 	counter  = 0;			// Var Used For the Counter Loop, Counter is the spi Data
uint8_t v[1] = { 0x00 };					// Var Used For Holding the i2c Value

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 250) == 0 ) { 
	gpio_Toggle(GPIOC, 13); 												// Toggles PC13 Every 250ms    		
	
	v[0] = counter; i2c_write(0x27, v, 1);   								// INCR_I2C_DATA  Every 250ms
	counter++; if( counter > 255 ) { counter = 0; }							// Incrementes counter And Caps It	
	}
}


int main(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 	72000/72_MHZ=1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	i2c_init();																// Initializes 	I2C   	PB6=CLK 	PB7=DATA
	
	for(;;) {	
	}
}


