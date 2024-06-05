/********************************************************************************
* 		 Simple PID Control. 	Using PA0 = AN0, PA1 = PWM						* 
* 		 Also Debug Some PID Data To the MAX7219 8x8 Led Matrix					*
********************************************************************************/

/********************************************************************************
 		 Tested With An RC Filter.	Needs Some Tuning Though
		 Tested With Two Pots For Disturbing The process    		

	PA0 -- R1K -- 100uF -- R1K -- PA1
					|
				   GND	
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  

uint8_t v[8] = {0,0,0,0,0,0,0,0};			// Var Used For Holding the SPI_Registers_Alike
int i,j;

// *** PID Variables *** //
int setpoint = 90; 							// desired output  
int processVariable = 100 ; 				// current output  
int error = 0; 								// difference between setpoint and processVariable  
int previousError = 0; 						// error in previous iteration  
int integral = 0; 							// integral of error  
int derivative = 0; 						// derivative of error  
int kp = 1; 								// proportional gain  
int ki = 10; 								// integral gain  
int kd = 10; 								// derivative gain  
int output; 								// output of the controller  
int Valvin = 30; 							// Input Valv 
int Valvout = 40 ;							// Onput Valv 	

// *** PID FUNCTION *** //
int calculateOutput(int processVariable) {   
    error = setpoint - processVariable;  
    integral += error;  
    derivative = error - previousError;  
    output = kp * error + integral/ki + derivative/kd;  
    previousError = error;  
    return output;  
}

// *** FUNCTION For mapping Values *** //
uint16_t map_16bit(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
		{ return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 10) == 0 ) { 											// Do This Every 10ms
	gpio_Toggle(GPIOC, 13); 												// Toggles PC13 Every 10ms	

	v[0]=processVariable; v[1]=setpoint;									// Puts PID Values in the SPI Regs
	for(i = 0; i < 8; i++) { send_spi_data_16(i+1, v[i]); }					// Shows_Regs On MAX7219 For Debugging PID Data
	}
	
	if ( (millis % 100) == 0 ) { 											// Do This Every 100ms	
		uint16_t adc_value = ADC1->DR;										// Gets ADC1 Value 
		processVariable = map_16bit(adc_value, 0, 4096, 0, 100);			// ADC1 mapping Goes To the processVariable	
		Valvin = calculateOutput(processVariable) ; 						// Calculates the PID Output
		if (Valvin < 0) {Valvin = 0;}										// Caps The PID Output
		if (Valvin > 100) {Valvin = 100;}
		TIM2->CCR2 = Valvin*204 ;											// PWM Values 0-20480
	}	
	
	if ( (millis % 7000) == 0 ) {											// Do This Every 7s		
	if (setpoint > 80) { setpoint=20; }										// Caps the setpoint
	else{setpoint+=10;}														// Incrementes the setpoint by 10
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

void Hardware_Init(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	systick_init(72000, 1);													// Initializes 	systick WITH 1ms interrupt 	72000/72_MHZ = 1ms	
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	MAX7219_Init();				
	adc_1_init();															// Initializes 	ADC1 = PA0
	pwm_tim2_init();														// Initializes 	PWM = PA1
	
}

int main(void){
	Hardware_Init();
	for(;;) {	
	}
}


