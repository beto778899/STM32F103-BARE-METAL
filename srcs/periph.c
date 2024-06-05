
/********************************************************************************
* 		DEFINITIONS AND FUNCTIONS FOR THE BLUE_PILL STM32F103C8            		*
********************************************************************************/


/********************************************************************************
* 						TIM2_PWM  DEFINITIONS                           		*
********************************************************************************/

#define PULSE_MIN           480            			// 1000  µs = 1 ms
#define PULSE_CENTER        1340            		// 1500  µs = 1.5 ms
#define PULSE_MAX           2225            		// 2000  µs = 2 ms
#define PULSE_FREQ          50              		// 20000 µs = 20 ms
#define PRESCALE_1MHZ       1000000         		// 1 Mhz prescale

#define GPIO_CRL_CNF1_Pos                    (6U)                              
#define GPIO_CRL_CNF1_Msk                    (0x3U << GPIO_CRL_CNF1_Pos)       /*!< 0x000000C0 */
#define GPIO_CRL_CNF1                        GPIO_CRL_CNF1_Msk                 /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define GPIO_CRL_CNF1_0                      (0x1U << GPIO_CRL_CNF1_Pos)       /*!< 0x00000040 */
#define GPIO_CRL_CNF1_1                      (0x2U << GPIO_CRL_CNF1_Pos)       /*!< 0x00000080 */

#define GPIO_CRL_MODE1_Pos                   (4U)                              
#define GPIO_CRL_MODE1_Msk                   (0x3U << GPIO_CRL_MODE1_Pos)      /*!< 0x00000030 */
#define GPIO_CRL_MODE1                       GPIO_CRL_MODE1_Msk                /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define GPIO_CRL_MODE1_0                     (0x1U << GPIO_CRL_MODE1_Pos)      /*!< 0x00000010 */
#define GPIO_CRL_MODE1_1                     (0x2U << GPIO_CRL_MODE1_Pos)      /*!< 0x00000020 */

struct TIM;
extern struct TIM * const TIM2;

struct TIM
{
  volatile uint32_t CR1;             /*!< TIM control register 1,                      Address offset: 0x00 */
  volatile uint32_t CR2;             /*!< TIM control register 2,                      Address offset: 0x04 */
  volatile uint32_t SMCR;            /*!< TIM slave Mode Control register,             Address offset: 0x08 */
  volatile uint32_t DIER;            /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  volatile uint32_t SR;              /*!< TIM status register,                         Address offset: 0x10 */
  volatile uint32_t EGR;             /*!< TIM event generation register,               Address offset: 0x14 */
  volatile uint32_t CCMR1;           /*!< TIM  capture/compare mode register 1,        Address offset: 0x18 */
  volatile uint32_t CCMR2;           /*!< TIM  capture/compare mode register 2,        Address offset: 0x1C */
  volatile uint32_t CCER;            /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  volatile uint32_t CNT;             /*!< TIM counter register,                        Address offset: 0x24 */
  volatile uint32_t PSC;             /*!< TIM prescaler register,                      Address offset: 0x28 */
  volatile uint32_t ARR;             /*!< TIM auto-reload register,                    Address offset: 0x2C */
  volatile uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
  volatile uint32_t CCR1;            /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  volatile uint32_t CCR2;            /*!< TIM capture/compare register 2,              Address offset: 0x38 */
  volatile uint32_t CCR3;            /*!< TIM capture/compare register 3,              Address offset: 0x3C */
  volatile uint32_t CCR4;            /*!< TIM capture/compare register 4,              Address offset: 0x40 */
  volatile uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
  volatile uint32_t DCR;             /*!< TIM DMA control register,                    Address offset: 0x48 */
  volatile uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
  volatile uint32_t OR;              /*!< TIM option register,                         Address offset: 0x50 */
};

struct TIM    * const TIM2    = ((struct TIM *)    (0x40000000U + 0x00000000U));

/***************  Bit definition for TIM_CCMR1_OC2M register  ****************/
#define TIM_CCMR1_OC2M_Pos                  (12U)                              
#define TIM_CCMR1_OC2M_Msk                  (0x7U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00007000 */
#define TIM_CCMR1_OC2M                      TIM_CCMR1_OC2M_Msk                 /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0                    (0x1U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1                    (0x2U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2                    (0x4U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00004000 */
#define TIM_CCER_CC2E_Pos                   (4U)                               
#define TIM_CCER_CC2E_Msk                   (0x1U << TIM_CCER_CC2E_Pos)        /*!< 0x00000010 */
#define TIM_CCER_CC2E                       TIM_CCER_CC2E_Msk                  /*!<Capture/Compare 2 output enable */
/*******************  Bit definition for TIM_CR1 register  *******************/
#define TIM_CR1_CEN_Pos                     (0U)                               
#define TIM_CR1_CEN_Msk                     (0x1U << TIM_CR1_CEN_Pos)          /*!< 0x00000001 */
#define TIM_CR1_CEN                         TIM_CR1_CEN_Msk                    /*!<Counter enable */


/********************************************************************************
*						TIM2_PWM INIT FUNCTION	                           		*
********************************************************************************/

// Function That Inits 					TIM2_50HZ_Cycle (20ms) 		
// Duty_Val From 1-20700 (1-20ms)		PA1 = PWM

uint32_t SystemCoreClock   	= 72000000U;     		// System Clock Frequency (Core Clock) 
void pwm_tim2_init(void)					
{
    RCC->APB1ENR    |= RCC_APB1ENR_TIM2EN;  						// enable TIM   clock
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;  						// enable GPIOA clock
    GPIOA->CRL &= ~(GPIO_CRL_CNF1  | GPIO_CRL_MODE1);   			// reset PA1
    GPIOA->CRL |=  ( (GPIO_CRL_CNF1_1) | (GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0) );
    uint16_t prescale   = (SystemCoreClock / PRESCALE_1MHZ);
    TIM2->PSC           = prescale - 1;        
    uint16_t period = (SystemCoreClock / prescale / PULSE_FREQ);	// set period to be 50Hz / 20ms
    TIM2->ARR       = period;
    TIM2->CCR2 = PULSE_CENTER;										// set duty cycle
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);			// PWM mode 1    
    TIM2->CCER |= TIM_CCER_CC2E;									// enable output compare on OC2 pin    
    TIM2->CR1 |= TIM_CR1_CEN;										// enable counter    
} 

// TIM2->CCR2 = duty_var; 			set duty cycle
// duty = 2225             2 ms 	de MAX 20 ms
// duty = 4096             4 ms		de MAX 20 ms
// duty = 8200             8 ms		de MAX 20 ms
// duty = 16400           16 ms		de MAX 20 ms
// duty = 20700           20 ms		de MAX 20 ms	


/********************************************************************************
* 							ADC_1  DEFINITIONS   	                     		*
********************************************************************************/

#define RCC_CFGR_ADCPRE_DIV6                 0x00008000U                       /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8                 0x0000C000U                       /*!< PCLK2 divided by 8 */
/*******************  Bit definition for ADC_SMPR2 register  *******************/
#define ADC_SMPR2_SMP0_Pos                  (0U)                               
#define ADC_SMPR2_SMP0_Msk                  (0x7U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000007 */
#define ADC_SMPR2_SMP0                      ADC_SMPR2_SMP0_Msk                 /*!< ADC channel 0 sampling time selection  */
#define ADC_SMPR2_SMP0_0                    (0x1U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000001 */
#define ADC_SMPR2_SMP0_1                    (0x2U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000002 */
#define ADC_SMPR2_SMP0_2                    (0x4U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000004 */
/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_ADON_Pos                    (0U)                               
#define ADC_CR2_ADON_Msk                    (0x1U << ADC_CR2_ADON_Pos)         /*!< 0x00000001 */
#define ADC_CR2_ADON                        ADC_CR2_ADON_Msk                   /*!< ADC enable */
#define ADC_CR2_CONT_Pos                    (1U)                               
#define ADC_CR2_CONT_Msk                    (0x1U << ADC_CR2_CONT_Pos)         /*!< 0x00000002 */
#define ADC_CR2_CONT                        ADC_CR2_CONT_Msk                   /*!< ADC group regular continuous conversion mode */
#define ADC_CR2_CAL_Pos                     (2U)                               
#define ADC_CR2_CAL_Msk                     (0x1U << ADC_CR2_CAL_Pos)          /*!< 0x00000004 */
#define ADC_CR2_CAL                         ADC_CR2_CAL_Msk                    /*!< ADC calibration start */
#define ADC_CR2_RSTCAL_Pos                  (3U)                               
#define ADC_CR2_RSTCAL_Msk                  (0x1U << ADC_CR2_RSTCAL_Pos)       /*!< 0x00000008 */
#define ADC_CR2_RSTCAL                      ADC_CR2_RSTCAL_Msk                 /*!< ADC calibration reset */
#define ADC_CR2_DMA_Pos                     (8U)                               
#define ADC_CR2_DMA_Msk                     (0x1U << ADC_CR2_DMA_Pos)          /*!< 0x00000100 */
#define ADC_CR2_DMA                         ADC_CR2_DMA_Msk                    /*!< ADC DMA transfer enable */
#define ADC_CR2_ALIGN_Pos                   (11U)                              
#define ADC_CR2_ALIGN_Msk                   (0x1U << ADC_CR2_ALIGN_Pos)        /*!< 0x00000800 */
#define ADC_CR2_ALIGN                       ADC_CR2_ALIGN_Msk                  /*!< ADC data alignement */
/*******************  Bit definition for GPIO_CRL register  ********************/
#define GPIO_CRL_CNF0_Pos                    (2U)                              
#define GPIO_CRL_CNF0_Msk                    (0x3U << GPIO_CRL_CNF0_Pos)       /*!< 0x0000000C */
#define GPIO_CRL_CNF0                        GPIO_CRL_CNF0_Msk                 /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define GPIO_CRL_CNF0_0                      (0x1U << GPIO_CRL_CNF0_Pos)       /*!< 0x00000004 */
#define GPIO_CRL_CNF0_1                      (0x2U << GPIO_CRL_CNF0_Pos)       /*!< 0x00000008 */
#define GPIO_CRL_MODE0_Pos                   (0U)                              
#define GPIO_CRL_MODE0_Msk                   (0x3U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000003 */
#define GPIO_CRL_MODE0                       GPIO_CRL_MODE0_Msk                /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define GPIO_CRL_MODE0_0                     (0x1U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000001 */
#define GPIO_CRL_MODE0_1                     (0x2U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000002 */

struct ADC;
extern struct ADC * const ADC1;
struct ADC
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ;

struct ADC    * const ADC1    = ((struct ADC *)    (0x40000000U + 0x00010000U + 0x00002400U));

// Function That Inits ADC1 = PA0		In Continuous Conversion Mode		
void adc_1_init(void)
{
RCC->CFGR       |= RCC_CFGR_ADCPRE_DIV6;
RCC->APB2ENR    |= RCC_APB2ENR_ADC1EN;						// enable ADC clock
GPIOA->CRL &= ~(GPIO_CRL_CNF0  | GPIO_CRL_MODE0);   		// reset PA0
ADC1->SMPR2 |= ( ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0);   
ADC1->CR2 |= (ADC_CR2_CONT | ADC_CR2_ADON);					// put ADC1 into continuous mode and turn on ADC
ADC1->CR2 |= (ADC_CR2_RSTCAL);								// reset calibration registers
while(ADC1->CR2 & ADC_CR2_RSTCAL);							// wait for calibration register initalized
ADC1->CR2 |= (ADC_CR2_CAL);									// enable calibration
while(ADC1->CR2 & ADC_CR2_CAL);				 				// wait for calibration completed
    
    // Not concerned about power consumption, just start the continuous
    // conversions and will read the DR periodically
ADC1->CR2 |= ADC_CR2_ADON;									// start conversions
}

// uint16_t adc_value = ADC1->DR;


/********************************************************************************
* 								USART DEFINITIONS                        		*
********************************************************************************/

void uart_putc(char c);
char uart_getc(void);

#define USART_SR_TXE	(1 << 7)
#define USART_SR_RXNE	(1 << 5)
#define USART_CR_SBK	(1 << 0)
#define USART_CR_RWU	(1 << 1)
#define USART_CR_RE	(1 << 2)
#define USART_CR_TE	(1 << 3)
#define USART_CR_IDLEIE	(1 << 4)
#define USART_CR_RXNEIE	(1 << 5)
#define USART_CR_TCIE	(1 << 6)
#define USART_CR_TXEIE	(1 << 7)
#define USART_CR_PEIE	(1 << 8)
#define USART_CR_PS	(1 << 9)
#define USART_CR_PCE	(1 << 10)
#define USART_CR_WAKE	(1 << 11)
#define USART_CR_M	(1 << 12)
#define USART_CR_UE	(1 << 13)
#define USART1        ((struct USART *) 0x40013800)
#define USART2        ((struct USART *) 0x40004400)
#define USART3        ((struct USART *) 0x40004800)
#define USART1_BB     ((struct USART_BB *) 0x42270000)
#define USART2_BB     ((struct USART_BB *) 0x42088000)
#define USART3_BB     ((struct USART_BB *) 0x42090000)

struct USART {
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
};

struct USART_BB {
	volatile uint32_t SR[32];
	volatile uint32_t DR[32];
	volatile uint32_t BRR[32];
	volatile uint32_t CR1[32];
	volatile uint32_t CR2[32];
	volatile uint32_t CR3[32];
	volatile uint32_t GTPR[32];
};

// Function That Inits USART1.		 BPS Value Is Passed To the Function
void uart_init(int usart_bps)
{	
	gpio_setup(GPIOA,  9, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_ALT_PULL);		// UART1 PA9=TX
	gpio_setup(GPIOA, 10, GPIO_MODE_IN | GPIO_CNF_IN_PULL);					// UART1 PA10=RX
	
	USART1->CR1 = USART_CR_UE | USART_CR_TE | USART_CR_RE;
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	USART1->GTPR = 0;
	USART1->BRR = RCC_APB2_FREQ / usart_bps;		// 4800 9600 19200 38400 57600 115200
}

// Function That Puts One Char To The Serial_1 Port
void uart_putchar(char c)
{
	while (!(USART1->SR & USART_SR_TXE))
		;
	USART1->DR = (uint32_t)c;
}

// Reads One Char From The Serial_1 Port
char uart_getchar(void)
{
	while (!(USART1->SR & USART_SR_RXNE))
		;
	return USART1->DR & 0xff;
}


/********************************************************************************
* 								I2C  DEFINITIONS                        		*
********************************************************************************/

void i2c_init(void);
int i2c_write(uint8_t address, uint8_t *values, unsigned long values_length);

#define RCC_APB1_FREQ 36000000
#define RCC_APB2_FREQ 72000000
#define RCC_CPU_CLK_FREQ 72000000

#define I2C_CR1_PE			(1 << 0)
#define I2C_CR1_SMBUS		(1 << 1)
#define I2C_CR1_SMBTYPE		(1 << 3)
#define I2C_CR1_ENARP		(1 << 4)
#define I2C_CR1_ENPEC		(1 << 5)
#define I2C_CR1_ENGC		(1 << 6)
#define I2C_CR1_NO_STRETCH	(1 << 7)
#define I2C_CR1_START		(1 << 8)
#define I2C_CR1_STOP		(1 << 9)
#define I2C_CR1_ACK			(1 << 10)
#define I2C_CR1_POS			(1 << 11)
#define I2C_CR1_PEC			(1 << 12)
#define I2C_CR1_ALERT		(1 << 13)
#define I2C_CR1_SWRST		(1 << 15)
#define I2C_CR2_FREQ_SHIFT	0
#define I2C_CR2_ITERREN		(1 << 8)
#define I2C_CR2_ITEVTEN		(1 << 9)
#define I2C_CR2_ITBUFEN		(1 << 10)
#define I2C_CR2_DMAEN		(1 << 11)
#define I2C_CR2_LAST		(1 << 12)
#define I2C_CCR_CCR_SHIFT	0
#define I2C_CCR_DUTY		(1 << 14)
#define I2C_CCR_FS			(1 << 15)
#define I2C_SR1_SB			(1 << 0)
#define I2C_SR1_ADDR		(1 << 1)
#define I2C_SR1_BTF			(1 << 2)
#define I2C_SR1_ADD10		(1 << 3)
#define I2C_SR1_STOPF		(1 << 4)
#define I2C_SR1_RxNE		(1 << 6)
#define I2C_SR1_TxE			(1 << 7)
#define I2C_SR1_BERR		(1 << 8)
#define I2C_SR1_ARLO		(1 << 9)
#define I2C_SR1_AF			(1 << 10)
#define I2C_SR1_OVR			(1 << 11)
#define I2C_SR1_PEC_ERR		(1 << 12)
#define I2C_SR1_TIME_OUT	(1 << 14)
#define I2C_SR1_SMB_ALERT	(1 << 15)

#define I2C1    ((struct I2C *)0x40005400)
#define I2C2    ((struct I2C *)0x40005800)
#define I2C1_BB ((struct I2C_BB *)0x420A8000)
#define I2C2_BB ((struct I2C_BB *)0x420B0000)

#define I2C_DUTYCYCLE_2		0
#define I2C_DUTYCYCLE_16_9	I2C_CCR_DUTY

#define I2C_CCR_CCR		0xfff

#define I2C_FREQRANGE(__PCLK__) \
	((__PCLK__)/1000000U)
#define I2C_RISE_TIME(__FREQRANGE__, __SPEED__) \
	(((__SPEED__) <= 100000U) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))
#define I2C_SPEED_STANDARD(__PCLK__, __SPEED__) \
	(((((__PCLK__)/((__SPEED__) << 1U)) & I2C_CCR_CCR) < 4U)? 4U:((__PCLK__) / ((__SPEED__) << 1U)))
#define I2C_SPEED_FAST(__PCLK__, __SPEED__, __DUTYCYCLE__) \
	(((__DUTYCYCLE__) == I2C_DUTYCYCLE_2)? ((__PCLK__) / ((__SPEED__) * 3U)) : (((__PCLK__) / ((__SPEED__) * 25U)) | I2C_DUTYCYCLE_16_9))
#define I2C_SPEED(__PCLK__, __SPEED__, __DUTYCYCLE__) \
	(((__SPEED__) <= 100000U)? (I2C_SPEED_STANDARD((__PCLK__), (__SPEED__))) : \
	((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__)) & I2C_CCR_CCR) == 0U)? 1U : \
	((I2C_SPEED_FAST((__PCLK__), (__SPEED__), (__DUTYCYCLE__))) | I2C_CCR_FS))

#define I2C_FREQ  150000

struct I2C {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
};

struct I2C_BB {
	volatile uint32_t CR1[32];
	volatile uint32_t CR2[32];
	volatile uint32_t OAR1[32];
	volatile uint32_t OAR2[32];
	volatile uint32_t DR[32];
	volatile uint32_t SR1[32];
	volatile uint32_t SR2[32];
	volatile uint32_t CCR[32];
	volatile uint32_t TRISE[32];
};

// Function That INITS I2C_1 at 150000 BPS	Master Mode
void i2c_init(void)
{	
	gpio_setup(GPIOB,  6, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_ALT_OPEN);		// I2C gpios PB6=CLK
	gpio_setup(GPIOB,  7, GPIO_MODE_OUT_50MHz | GPIO_CNF_OUT_ALT_OPEN);		// I2C gpios PB7=DATA
	
	I2C1->CR2 = I2C_FREQRANGE(RCC_APB1_FREQ);
	I2C1->TRISE = I2C_RISE_TIME(I2C_FREQRANGE(RCC_APB1_FREQ), I2C_FREQ);
	I2C1->CCR = I2C_SPEED(RCC_APB1_FREQ, I2C_FREQ, I2C_DUTYCYCLE_2);
	I2C1->CR1 |= I2C_CR1_PE;
}

#define I2C_ADDR_READ(addr)		((addr << 1) | 1)
#define I2C_ADDR_WRITE(addr)	(addr << 1)
#define TIMEOUT 1000								// 100000 = 25ms x 2 = 50ms

//Funtion That Writes I2C Data 		
int i2c_write(uint8_t address, uint8_t *values, unsigned long values_length)
{
	unsigned long i;

	I2C1->CR1 |= I2C_CR1_START;
	for (i = 0; !(I2C1->SR1 & I2C_SR1_SB); i++)
		if (i > TIMEOUT)
			return -1;

	I2C1->DR = I2C_ADDR_WRITE(address);
	for (i = 0; !(I2C1->SR1 & I2C_SR1_ADDR); i++)
		if (i > TIMEOUT)
			return -2;
	
	I2C1->SR2;
	
	for (; values_length; values_length--, values++) {
		for (i = 0; !(I2C1->SR1 & I2C_SR1_TxE); i++) {
			if (i > TIMEOUT)
				return -3;
		}
		I2C1->DR = *values;
	}

	for (i = 0; !((I2C1->SR1 & I2C_SR1_TxE) && (I2C1->SR1 & I2C_SR1_BTF)); i++)
		if (i > TIMEOUT)
			return -4;
		
	I2C1->CR1 |= I2C_CR1_STOP;

	return 0;
}
// uint8_t v[1] = { 0x00 };		v[0] = counter; 	i2c_write(0x27, v, 1); 


//Funtion That READS I2C Data 		
int i2c_read(uint8_t address, int values_length)
{
	unsigned long i; 
	int k;
	int i2c_byte;
	
	// I2C1 -> CR1 &= ~I2C_CR1_POS; 			// ????? Reset the I2C_CR1_POS bit in the I2C1_CR1 register.
	I2C1 -> CR1 |= I2C_CR1_ACK; 			// ????? Set the I2C_CR1_ACK and the I2C_CR1_START bits.

	I2C1->CR1 |= I2C_CR1_START;
	for (i = 0; !(I2C1->SR1 & I2C_SR1_SB); i++)
		if (i > TIMEOUT)
			return -1;

	I2C1->DR = I2C_ADDR_READ(address);
	for (i = 0; !(I2C1->SR1 & I2C_SR1_ADDR); i++)
		if (i > TIMEOUT)
			return -2;
	
	I2C1->SR2;		// I2C.I2C1->SR1;   ?????		
	
	for (k = 0; k <= (values_length-1); k++) {
		for (i = 0; !(I2C1->SR1 & I2C_SR1_RxNE); i++) {
			if (i > TIMEOUT)
				return -3;
		}
		i2c_byte = I2C1->DR;					// buffer[k] = I2C1 -> DR; 
		I2C1 -> CR1 |= I2C_CR1_ACK; 			// Set I2C_CR1_ACK bit after reading.
	}
	k++; 
	i2c_byte = I2C1->DR;			// buffer[k] = I2C1 -> DR; 
	I2C1->CR1 &= ~I2C_CR1_ACK; 		// After the last data is read, send a NACK by resetting the I2C_CR1_ACK bit.
	I2C1->CR1 |= I2C_CR1_STOP;		// Send STOP by setting the I2C_CR1_STOP bit.

	return i2c_byte;
}
//	byte = i2c_read(0x27, 1); 		// FIX the buffer Holding I2C_Read_Value		buffer[k] = I2C1 -> DR; 



/********************************************************************************
* 								SPI  DEFINITIONS                        		*
********************************************************************************/

uint16_t send_spi_data(uint16_t spi_data);
void send_spi_data_16(uint8_t spi_reg, uint8_t spi_data);

#define GPIO_CRL_MODE4_Pos                   (16U)                             
#define GPIO_CRL_MODE4_Msk                   (0x3U << GPIO_CRL_MODE4_Pos)      /*!< 0x00030000 */
#define GPIO_CRL_MODE4                       GPIO_CRL_MODE4_Msk                /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define GPIO_CRL_MODE4_0                     (0x1U << GPIO_CRL_MODE4_Pos)      /*!< 0x00010000 */
#define GPIO_CRL_MODE4_1                     (0x2U << GPIO_CRL_MODE4_Pos)      /*!< 0x00020000 */

#define GPIO_CRL_MODE5_Pos                   (20U)                             
#define GPIO_CRL_MODE5_Msk                   (0x3U << GPIO_CRL_MODE5_Pos)      /*!< 0x00300000 */
#define GPIO_CRL_MODE5                       GPIO_CRL_MODE5_Msk                /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define GPIO_CRL_MODE5_0                     (0x1U << GPIO_CRL_MODE5_Pos)      /*!< 0x00100000 */
#define GPIO_CRL_MODE5_1                     (0x2U << GPIO_CRL_MODE5_Pos)      /*!< 0x00200000 */

#define GPIO_CRL_MODE6_Pos                   (24U)                             
#define GPIO_CRL_MODE6_Msk                   (0x3U << GPIO_CRL_MODE6_Pos)      /*!< 0x03000000 */
#define GPIO_CRL_MODE6                       GPIO_CRL_MODE6_Msk                /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define GPIO_CRL_MODE6_0                     (0x1U << GPIO_CRL_MODE6_Pos)      /*!< 0x01000000 */
#define GPIO_CRL_MODE6_1                     (0x2U << GPIO_CRL_MODE6_Pos)      /*!< 0x02000000 */

#define GPIO_CRL_MODE7_Pos                   (28U)                             
#define GPIO_CRL_MODE7_Msk                   (0x3U << GPIO_CRL_MODE7_Pos)      /*!< 0x30000000 */
#define GPIO_CRL_MODE7                       GPIO_CRL_MODE7_Msk                /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define GPIO_CRL_MODE7_0                     (0x1U << GPIO_CRL_MODE7_Pos)      /*!< 0x10000000 */
#define GPIO_CRL_MODE7_1                     (0x2U << GPIO_CRL_MODE7_Pos)      /*!< 0x20000000 */

#define GPIO_CRL_CNF4_Pos                    (18U)                             
#define GPIO_CRL_CNF4_Msk                    (0x3U << GPIO_CRL_CNF4_Pos)       /*!< 0x000C0000 */
#define GPIO_CRL_CNF4                        GPIO_CRL_CNF4_Msk                 /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define GPIO_CRL_CNF4_0                      (0x1U << GPIO_CRL_CNF4_Pos)       /*!< 0x00040000 */
#define GPIO_CRL_CNF4_1                      (0x2U << GPIO_CRL_CNF4_Pos)       /*!< 0x00080000 */

#define GPIO_CRL_CNF5_Pos                    (22U)                             
#define GPIO_CRL_CNF5_Msk                    (0x3U << GPIO_CRL_CNF5_Pos)       /*!< 0x00C00000 */
#define GPIO_CRL_CNF5                        GPIO_CRL_CNF5_Msk                 /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CRL_CNF5_0                      (0x1U << GPIO_CRL_CNF5_Pos)       /*!< 0x00400000 */
#define GPIO_CRL_CNF5_1                      (0x2U << GPIO_CRL_CNF5_Pos)       /*!< 0x00800000 */

#define GPIO_CRL_CNF6_Pos                    (26U)                             
#define GPIO_CRL_CNF6_Msk                    (0x3U << GPIO_CRL_CNF6_Pos)       /*!< 0x0C000000 */
#define GPIO_CRL_CNF6                        GPIO_CRL_CNF6_Msk                 /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define GPIO_CRL_CNF6_0                      (0x1U << GPIO_CRL_CNF6_Pos)       /*!< 0x04000000 */
#define GPIO_CRL_CNF6_1                      (0x2U << GPIO_CRL_CNF6_Pos)       /*!< 0x08000000 */

#define GPIO_CRL_CNF7_Pos                    (30U)                             
#define GPIO_CRL_CNF7_Msk                    (0x3U << GPIO_CRL_CNF7_Pos)       /*!< 0xC0000000 */
#define GPIO_CRL_CNF7                        GPIO_CRL_CNF7_Msk                 /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CRL_CNF7_0                      (0x1U << GPIO_CRL_CNF7_Pos)       /*!< 0x40000000 */
#define GPIO_CRL_CNF7_1                      (0x2U << GPIO_CRL_CNF7_Pos)       /*!< 0x80000000 */

#define GPIO_BSRR_BS4_Pos                    (4U)                              
#define GPIO_BSRR_BS4_Msk                    (0x1U << GPIO_BSRR_BS4_Pos)       /*!< 0x00000010 */
#define GPIO_BSRR_BS4                        GPIO_BSRR_BS4_Msk                 /*!< Port x Set bit 4 */

#define GPIO_BSRR_BR4_Pos                    (20U)                             
#define GPIO_BSRR_BR4_Msk                    (0x1U << GPIO_BSRR_BR4_Pos)       /*!< 0x00100000 */
#define GPIO_BSRR_BR4                        GPIO_BSRR_BR4_Msk                 /*!< Port x Reset bit 4 */

#define SPI_CR1_SSI_Pos                     (8U)                               
#define SPI_CR1_SSI_Msk                     (0x1U << SPI_CR1_SSI_Pos)          /*!< 0x00000100 */
#define SPI_CR1_SSI                         SPI_CR1_SSI_Msk                    /*!< Internal slave select */
#define SPI_CR1_SSM_Pos                     (9U)                               
#define SPI_CR1_SSM_Msk                     (0x1U << SPI_CR1_SSM_Pos)          /*!< 0x00000200 */
#define SPI_CR1_SSM                         SPI_CR1_SSM_Msk                    /*!< Software slave management */

#define SPI_CR1_MSTR_Pos                    (2U)                               
#define SPI_CR1_MSTR_Msk                    (0x1U << SPI_CR1_MSTR_Pos)         /*!< 0x00000004 */
#define SPI_CR1_MSTR                        SPI_CR1_MSTR_Msk                   /*!< Master Selection */

#define SPI_CR1_BR_Pos                      (3U)                               
#define SPI_CR1_BR_2                        (0x4U << SPI_CR1_BR_Pos)           /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos                     (6U)                               
#define SPI_CR1_SPE_Msk                     (0x1U << SPI_CR1_SPE_Pos)          /*!< 0x00000040 */
#define SPI_CR1_SPE                         SPI_CR1_SPE_Msk                    /*!< SPI Enable */

#define SPI_SR_TXE_Pos                      (1U)                               
#define SPI_SR_TXE_Msk                      (0x1U << SPI_SR_TXE_Pos)           /*!< 0x00000002 */
#define SPI_SR_TXE                          SPI_SR_TXE_Msk                     /*!< Transmit buffer Empty */

#define SPI_SR_BSY_Pos                      (7U)                               
#define SPI_SR_BSY_Msk                      (0x1U << SPI_SR_BSY_Pos)           /*!< 0x00000080 */
#define SPI_SR_BSY                          SPI_SR_BSY_Msk                     /*!< Busy flag */

struct SPI;
extern struct SPI * const SPI1;

struct SPI
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
} ;

struct SPI    * const SPI1    = ((struct SPI *)    (0x40000000U + 0x00010000U + 0x00003000U));

// INITS SPI Periph In Master Mode 		8 Bits 
void spi_init(void)
{
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;  // enable GPIOA clock
    RCC->APB2ENR    |= RCC_APB2ENR_SPI1EN;  // enable SPI1 clock
    GPIOA->CRL &= ~( (GPIO_CRL_MODE4 | GPIO_CRL_CNF4) | (GPIO_CRL_MODE5 | GPIO_CRL_CNF5) |
                     (GPIO_CRL_MODE7 | GPIO_CRL_CNF7)
                     );
    GPIOA->CRL |= GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0;
    GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_1;
    GPIOA->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1;
	
	gpio_setup(GPIOA, 6, GPIO_MODE_IN | GPIO_CNF_IN_PULL);				// SPI PA6 = MISO
	
    GPIOA->BSRR = GPIO_BSRR_BS4;            // set the SS pin high
  		
	// initialize the SPI configuration register
    SPI1->CR1  =  SPI_CR1_SSM   // software slave management enabled
                | SPI_CR1_SSI   // internal slave select
                | SPI_CR1_MSTR  // SPI master mode
                | SPI_CR1_BR_2; // bit rate prescale /32 (72MHz/32 = 2.25MHz)
    
    SPI1->CR1  |= SPI_CR1_SPE;  // enable SPI	
}

// SENDS Data to The SPI Periph.		PA4 is the LATCH Pin.
uint16_t send_spi_data(uint16_t spi_data)
{
	GPIOA->BSRR = GPIO_BSRR_BR4;        	// reset NSS pin 
	SPI1->DR = spi_data;                   	// send data out SPI
	while( !(SPI1->SR & SPI_SR_TXE) );  	// wait until transmit buffer empty
	while( SPI1->SR & SPI_SR_BSY );     	// wait until SPI not busy
	GPIOA->BSRR = GPIO_BSRR_BS4;        	// set NSS pin
	return 	SPI1->DR;
}
// send_spi_data(0x55); 

void send_spi_data_16(uint8_t spi_reg, uint8_t spi_data)
{
	GPIOA->BSRR = GPIO_BSRR_BR4;        	// reset NSS pin 
	
	SPI1->DR = spi_reg;                   	// send data out SPI
	while( !(SPI1->SR & SPI_SR_TXE) );  	// wait until transmit buffer empty
	while( SPI1->SR & SPI_SR_BSY );     	// wait until SPI not busy


	SPI1->DR = spi_data;                   	// send data out SPI
	while( !(SPI1->SR & SPI_SR_TXE) );  	// wait until transmit buffer empty
	while( SPI1->SR & SPI_SR_BSY );     	// wait until SPI not busy
	
	GPIOA->BSRR = GPIO_BSRR_BS4;        	// set NSS pin 	
}
// send_spi_data(0x07, 0x55); 