/********************************************************************************
* 		 			I2C Read From the PCF8574							    	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

int mybyte = 0;

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		

	if ( (millis % 100) == 0 ) { 
		mybyte = i2c_read(0x27, 1) & 1;					// Reads PCF8574 BIT_0
		if (mybyte) { gpio_out(GPIOC, 13, 0); }			// If BIT_0 = 1 Then Led_On
		else { gpio_out(GPIOC, 13, 1); }				// If BIT_0 = 0 Then Led_Off
	}	
}

void Hardware_Init(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 	72000/72_MHZ=1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	i2c_init();																// Initializes 	I2C   	PB6=CLK 	PB7=DATA
}	

int main(void){	
	Hardware_Init();	
	for(;;) {	
	}
}


