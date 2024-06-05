Toggles The Pins PC13, PC14 using the SysTick Timer.

Here are Some Examples To Setup The Pins

gpio_setup(GPIOC, 13, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_GPIO_OPEN);		// Makes PC13 Output
gpio_setup(GPIOA, 6, GPIO_MODE_IN | GPIO_CNF_IN_PULL);						// Makes PA6  Input 
gpio_out(GPIOC, 13, 1);					// Sets PC13
gpio_Toggle(GPIOC, 13);					// Toggles PC13
bool test = gpio_in(GPIOC, 15);			// Reads PC15
	

/********************************************************************************
* GPIO  DEFINITIONS 	                                                		*
********************************************************************************/
#define GPIO_MODE_IN        	0
#define GPIO_MODE_OUT_10MHz 	1
#define GPIO_MODE_OUT_2MHz  	2
#define GPIO_MODE_OUT_50MHz 	3

#define GPIO_CNF_IN_ANALOG		(0 << 2)
#define GPIO_CNF_IN_FLOATING	(1 << 2)
#define GPIO_CNF_IN_PULL		(2 << 2)
#define GPIO_CNF_OUT_GPIO_PULL	(0 << 2)
#define GPIO_CNF_OUT_GPIO_OPEN	(1 << 2)
#define GPIO_CNF_OUT_ALT_PULL	(2 << 2)
#define GPIO_CNF_OUT_ALT_OPEN	(3 << 2)
