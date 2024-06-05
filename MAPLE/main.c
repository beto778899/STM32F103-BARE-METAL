
/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 100) == 0 ) { gpio_Toggle(GPIOC, 13); }	// Toggles PC13 Every 100ms
	if ( (millis % 200) == 0 ) { gpio_Toggle(GPIOC, 14); }	// Toggles PC14 Every 200ms
}

int main(void){
	rcc_init();								// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);					// Initializes 	systick WITH 1ms interrupt 	72000/72MHZ=1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	gpio_setup(GPIOC, 14, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC14 Output
	
	for(;;) {	 							// EveryThing Is Done Inside The Systick_Loop Function  
	}
}
