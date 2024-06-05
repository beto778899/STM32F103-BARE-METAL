/****************************************************************************************
*	If Switch on PC15 is ON Then Send USART '0' 	 If it's OFF Then Send USART '1'   	*
*****************************************************************************************/

#include "rcc_gpio_tick.c"					// Definitions And Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

/********************************************************************************
*	Sort Of MultiTasking Using The Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 250) == 0 ) 								// Do This Every 250ms 
	{ 
	gpio_Toggle(GPIOC, 13);									// Toggles On Board LED
	gpio_out(GPIOC, 14, gpio_in(GPIOC, 15) );				// PC14 = PC15
	}	
	
	if ( (millis % 300) == 0 ) 								// Do This Every 300ms 
	{ 	
		if ( gpio_in(GPIOC, 15) ) { uart_putchar('1'); } 	// USART = PC15
		else { uart_putchar('0'); } 
	}	
	
}

void Hardware_Init(void){
	rcc_init();								// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);					// Initializes 	systick WITH 1ms interrupt 72000/72MHZ=1ms	 	
	uart_init(9600);						// Initializes  USART1 PA9=TX, PA10=RX, 9600, 19200, 38400, 57600, 115200  	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// Makes PC13 Output
	gpio_setup(GPIOC, 14, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// Makes PC14 Output
	gpio_setup(GPIOC, 15, GPIO_MODE_IN | GPIO_CNF_IN_PULL);					// Makes PC15 Input
}

int main(void){
	Hardware_Init();	
	for(;;) {	 							// Nothing Here. EveryThing Is Done Inside The Systick_Loop Function    
	}
}
