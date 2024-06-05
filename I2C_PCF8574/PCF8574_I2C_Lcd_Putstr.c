/********************************************************************************
* 		 				PCF8574 I2C_LCD Testing					    			*
*				main_lcd() is placed in  main()	Function 		 				*
* 				Sofware_Delay is been used for the LCD Pgm						*
********************************************************************************/

#include "rcc_gpio_tick.c"					// Functions For rcc_init, gpio_init, systick_init
#include "periph.c"							// Definitions And Functions For pwm, adc1, usart, i2c, spi  
#include "LCD_2_Wire.c"						// Functions For LCD Init, Write.   Sofware_Delay msec_dly();

/********************************************************************************
* 		Sort Of MultiTasking Using Systick Interrupt	When OS Is NA	    	*
********************************************************************************/

// Systick_Loop() Is Called Every 1ms. millis++ Is Incremented On Every Call. Both Defined in "rcc_gpio_tick.c" 
void Systick_Loop(void){		
	if ( (millis % 50) == 0 ) { 
	gpio_Toggle(GPIOC, 13); 												// Toggles PC13 Every 500ms    		
	}
}

void show_value(unsigned char value);
void main_lcd(void);

void main_lcd(void)
{	
	unsigned char s = 0;

	char txt1[] = {"PROTOBOARD"};		
	char txt2[] = {"Testing MCU"};
	char txt3[] = {"Blue-Pill ARM uC"};
	char txt4[] = {"STM32F103C8T6"};
	
	LCD_init();
	LCD_clear_home();

	LCD_goto(3, 0);					// Shows txt1 on Line1 
	LCD_putstr(txt1);
	LCD_goto(3, 1);					// Shows txt2 on Line2 
	LCD_putstr(txt2);
	msec_dly(3000);					// Sofware_Delay 3s

	LCD_clear_home();

	for(s = 0; s < 16; s++)
	{
		LCD_goto(s, 0);				// Shows txt3 on Line1 do it char_by_char
		LCD_putchar(txt3[s]);
		msec_dly(100);				// Sofware_Delay 100ms
	}
	
	msec_dly(1000);					// Sofware_Delay 1s
	
	for(s = 0; s < 13; s++)
	{
		LCD_goto((1 + s), 1);		// Shows txt4 on Line2 do it char_by_char
		LCD_putchar(txt4[s]);
		msec_dly(100);				// Sofware_Delay 100ms
	}
	
	msec_dly(1000);					// Sofware_Delay 1s
	
	s = 0;
	LCD_clear_home();
	
	LCD_goto(3, 0);
	LCD_putstr(txt1);				// Shows txt1 on Line1 
	
	for(int i=0;i<160;i++)
	{
		show_value(i);				// Shows Counting Numbers on Line2    
		msec_dly(100);				// Delay 100ms
	}
}

void show_value(unsigned char value)
{
   unsigned char ch = 0x00;

   ch = ((value / 100) + 0x30);
   LCD_goto(6, 1);
   LCD_putchar(ch);

   ch = (((value / 10) % 10) + 0x30);
   LCD_goto(7, 1);
   LCD_putchar(ch);

   ch = ((value % 10) + 0x30);
   LCD_goto(8, 1);
   LCD_putchar(ch);
}

void Hardware_Init(void){
	rcc_init();																// Initializes 	72MHZ_CLK   	PA_PB_PC_CLK
	gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);	// PC13 Output
	i2c_init();																// Initializes 	I2C   	PB6=CLK 	PB7=DATA
	systick_init(720000, 1);												// Initializes 	systick WITH 10ms interrupt 	720000/72_MHZ=10ms	
}	

int main(void){
	Hardware_Init();
	for(;;) {		
		main_lcd();			
		msec_dly(1000);					// Sofware_Delay 1s
	}
}

