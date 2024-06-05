The examples in this Folder Initialize The MAX7219 8x8 LED Matrix Module. Using the SPI Protocol

MAX_7219_Binary_Count	Binary_Count_All_Leds On the MAX7219 8x8 Led Matrix
MAX_7219_Fonts			Display Some Fonts (Numbers 0-9)
MAX_7219_Fonts_Huge		Display Lots of Fonts (780 Different Fonts)
MAX_7219_Show_Regs		Display the Value of Registers (For Debbuging)
SPI_MISO_READ			Test The SPI_INPUT By Making an LoopBack Connection Between MOSI=MISO

Read and Write To the SPI is done with this Functions.

spi_init();												// Initializes the SPI 		PA4=LATCH  PA5=CLK  PA6=MISO  PA7=MOSI	

send_spi_data(0x55);									// Sends One Btye  of Data   8_bits
send_spi_data_16(0x0F, 0x00); 							// Sends Two Bytes of Data  16_bits

uint16_t spi_in = send_spi_data(0x55);					// Reads the SPI periph. Sends dummy data. 



