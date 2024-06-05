/********************************************************************************
*		 		Send a value to SPI, PWM, I2C, LED_SPEED	 				   	*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Definitions And Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

/********************************************************************************
* 		Sort Of MultiTasking Using The Systick Interrupt	When OS Is NA    	*
********************************************************************************/

volatile uint16_t 	LED_DELAY_PC13  = 100;	// Var Used For Toggling the PC13 LED Initialy at 100ms
volatile uint16_t 	counter  = 0;			// Var Used For Counting, The value is used for spi_i2c_pwm_blink 
uint8_t v[1] = { 0x00 };					// Var Used For Holding the i2c Value

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % LED_DELAY_PC13) == 0 ) { gpio_Toggle(GPIOC, 13); }	// Toggles PC13 Every (LED_DELAY_PC13_ms)
		
	if ( (millis % 250) == 0 ) { 										// Do This Every 250ms	
	counter++; if (counter > 255){ counter = 0; }						// Incrementes the counter And Caps it
	
	send_spi_data(counter);   											// INCR_SPI_DATA  Every 250ms
	v[0] = counter; i2c_write(0x27, v, 1);   							// INCR_I2C_DATA  Every 250ms
	TIM2->CCR2     = (counter * 100);									// INCR_PWM_DUTY  Every 250ms
	LED_DELAY_PC13 = counter;  											// INCR_LED_DELAY Every 250ms
	}
	
}

void Hardware_Init(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 72000/72MHZ=1ms		
	spi_init();																// Initializes 	SPI 	PA4=LATCH 	PA5=CLK   PA7=MOSI			
	i2c_init();																// Initializes 	I2C   	PB6=CLK 	PB7=DATA
	pwm_tim2_init();														// Initializes 	PWM  = PA1
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// Initializes  PC13 = Output
}	

int main(void){	
	Hardware_Init();
	for(;;) {	 							// Nothing Here. EveryThing Is Done Inside The Systick_Loop Function   
	}
}
