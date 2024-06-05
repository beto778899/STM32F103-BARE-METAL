/********************************************************************************
*  Blinks The PC13_LED And Sets The PWM_Duty. Proportional to ADC Reading   	*
********************************************************************************/

#include "rcc_gpio_tick.c"				// Definitions And Functions For rcc_init, gpio_init, systick_init
#include "periph.c"					// Definitions And Functions For pwm, adc1, usart, i2c, spi  

volatile uint16_t 	LED_DELAY_PC13  = 100;		// Var Used For Toggling PC13 Every 100ms

// Function That Maps A Value to a Different Value.
uint16_t map_16bit(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
		{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

/********************************************************************************
* 	Sort Of MultiTasking Using The Systick Interrupt	 When OS Is NA  *
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % LED_DELAY_PC13) == 0 ) { gpio_Toggle(GPIOC, 13); }	// Toggles PC13 Every (LED_DELAY_PC13_ms)
	
	
	if ( (millis % 250) == 0 ) { 						// Do This Every 250ms		
		uint16_t adc_value = ADC1->DR;					// Gets the ADC1 Value 
		LED_DELAY_PC13 = map_16bit(adc_value, 0, 4096, 10, 256);	// ADC1 mapping value Is The BLINK_LED FREQ 
		TIM2->CCR2     = map_16bit(adc_value, 0, 4096, 1, 20256);	// ADC1 mapping value Is The PWM_DUTY 
	}				
}

void Hardware_Init(void){
	rcc_init();								// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);							// Initializes 	Systick WITH 1ms interrupt 72000/72MHZ=1ms		
	adc_1_init();								// Initializes 	ADC1 = PA0
	pwm_tim2_init();							// Initializes 	PWM  = PA1
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// Makes	PC13 = Output
}	

int main(void){	
	Hardware_Init();
	for(;;) {	 			// Nothing Here. EveryThing Is Done Inside The Systick_Loop Function   
	}
}
