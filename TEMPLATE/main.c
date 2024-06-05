/********************************************************************************
*	Blinks Two Leds. PC13 And PC14 at different speeds.		   	*
********************************************************************************/

#include "rcc_gpio_tick.c"				// Definitions And Functions For rcc_init, gpio_init, systick_init

/********************************************************************************
*   Sort Of MultiTasking Using The Systick Interrupt	When OS Is NA 	 	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 100) == 0 ) { gpio_Toggle(GPIOC, 13); }		// Toggles PC13 Every 100ms
	if ( (millis % 200) == 0 ) { gpio_Toggle(GPIOC, 14); }		// Toggles PC14 Every 200ms
}

int main(void){
	rcc_init();								// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);							// Initializes 	systick WITH 1ms interrupt 	72000/72MHZ=1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// Makes PC13 Output
	gpio_setup(GPIOC, 14, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// Makes PC14 Output
	
	for(;;) {	 			// Nothing Here. EveryThing Is Done Inside The Systick_Loop Function  
	}
}
