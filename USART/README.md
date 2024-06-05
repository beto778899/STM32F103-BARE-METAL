* Sends a Single char using the Serial Port peripheral.
* Connect a Switch between PC15 and GND.

* If Switch is ON Then Send USART '0' 
* If Switch is OFF Then Send USART '1'

-- Here are Some Examples To Setup The USART peripheral.		PA9=TX, PA10=RX --

* uart_init(9600);			  // Initializes  USART1 at 9600   BPS  
* uart_init(38400);				// Initializes  USART1 at 38400  BPS
* uart_init(57600);				// Initializes  USART1 at 57600  BPS
* uart_init(115200);			// Initializes  USART1 at 115200 BPS
* uart_putchar('H');			// Function That Puts The H Char To The Serial Port_1
