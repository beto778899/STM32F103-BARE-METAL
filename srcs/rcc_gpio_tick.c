
#include <stdbool.h>
#include <stdint.h>

/* Function declarations */
void Systick_Loop(void);
void systick_handler(void);
void init_systick(uint32_t s, uint8_t en);
void delay_ms(volatile uint32_t ms);
int main(void);

// Define some types for readibility
/*
#define int32_t         int
#define int16_t         short
#define int8_t          char
#define uint32_t        unsigned int
#define uint16_t        unsigned short
#define uint8_t         unsigned char
*/

/********************************************************************************
* 			CLOCK  DEFINITIONS                                         			*
********************************************************************************/

#define RCC_APB1_FREQ 36000000
#define RCC_APB2_FREQ 72000000
#define RCC_CPU_CLK_FREQ 72000000

struct RCC {
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
};

struct RCC_BB {
	volatile uint32_t CR[32];
	volatile uint32_t CFGR[32];
	volatile uint32_t CIR[32];
	volatile uint32_t APB2RSTR[32];
	volatile uint32_t APB1RSTR[32];
	volatile uint32_t AHBENR[32];
	volatile uint32_t APB2ENR[32];
	volatile uint32_t APB1ENR[32];
	volatile uint32_t BDCR[32];
	volatile uint32_t CSR[32];
};

struct FLASH {
	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t AR;
	volatile uint32_t RESERVED_1;
	volatile uint32_t OBR;
	volatile uint32_t WRPR;
};

struct FLASH_BB {
	volatile uint32_t ACR[32];
	volatile uint32_t KEYR[32];
	volatile uint32_t OPTKEYR[32];
	volatile uint32_t SR[32];
	volatile uint32_t CR[32];
	volatile uint32_t AR[32];
	volatile uint32_t RESERVED_1[32];
	volatile uint32_t OBR[32];
	volatile uint32_t WRPR[32];
};

#define RCC		((struct RCC *)0x40021000)
#define RCC_BB		((struct RCC_BB *)0x42420000)
#define FLASH		((struct FLASH *)0x40022000)
#define FLASH_BB	((struct FLASH_BB *)0x42440000)

#define RCC_CR_HSION		(1 << 0)
#define RCC_CR_HSIRDY		(1 << 1)
#define RCC_CR_HSITRIM_SHIFT	3
#define RCC_CR_HSICAL_SHIFT	8
#define RCC_CR_HSEON		(1 << 16)
#define RCC_CR_HSERDY		(1 << 17)
#define RCC_CR_HSEBYP		(1 << 18)
#define RCC_CR_CSSON		(1 << 19)
#define RCC_CR_PLLON		(1 << 24)
#define RCC_CR_PLLRDY		(1 << 25)
#define RCC_CR_PLL2ON		(1 << 26)
#define RCC_CR_PLL2RDY		(1 << 27)
#define RCC_CR_PLL3ON		(1 << 28)
#define RCC_CR_PLL3RDY		(1 << 29)

#define RCC_CFGR_SW_SHIFT	0
#define RCC_CFGR_SW_HSI	(0 << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SW_HSE	(1 << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SW_PLL	(2 << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SWS_SHIFT	2
#define RCC_CFGR_HPRE_SHIFT	4
#define RCC_CFGR_PPRE1_SHIFT	8
#define RCC_CFGR_PPRE1_div2	(4 << RCC_CFGR_PPRE1_SHIFT)
#define RCC_CFGR_PPRE2_SHIFT	11
#define RCC_CFGR_ADC_PRE_SHIFT	14
#define RCC_CFGR_PLL_SRC	(1 << 16)
#define RCC_CFGR_PLL_XTPRE	(1 << 17)
#define RCC_CFGR_PLLMUL_SHIFT	18
#define RCC_CFGR_PLLMUL_x9	(7 << RCC_CFGR_PLLMUL_SHIFT)
#define RCC_CFGR_OTGFS_PRE	(1 << 22)
#define RCC_CFGR_MCO_SHIFT	24

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_AFIOEN 	(1 << 0)
#define RCC_APB2ENR_IOPAEN 	(1 << 2)
#define RCC_APB2ENR_IOPBEN 	(1 << 3)
#define RCC_APB2ENR_IOPCEN 	(1 << 4)
#define RCC_APB2ENR_IOPDEN 	(1 << 5)
#define RCC_APB2ENR_IOPEEN 	(1 << 6)
#define RCC_APB2ENR_ADC1EN 	(1 << 9)
#define RCC_APB2ENR_ADC2EN 	(1 << 10)
#define RCC_APB2ENR_TIM1EN 	(1 << 11)
#define RCC_APB2ENR_SPI1EN 	(1 << 12)
#define RCC_APB2ENR_USART1EN 	(1 << 14)

/******************  Bit definition for RCC_APB1ENR register  *****************/
#define RCC_APB1ENR_TIM2EN 	(1 << 0)
#define RCC_APB1ENR_TIM3EN 	(1 << 1)
#define RCC_APB1ENR_TIM4EN 	(1 << 2)
#define RCC_APB1ENR_TIM5EN 	(1 << 3)
#define RCC_APB1ENR_TIM6EN 	(1 << 4)
#define RCC_APB1ENR_TIM7EN 	(1 << 5)
#define RCC_APB1ENR_WWDGEN 	(1 << 11)
#define RCC_APB1ENR_SPI2EN 	(1 << 14)
#define RCC_APB1ENR_SPI3EN 	(1 << 15)
#define RCC_APB1ENR_USART2EN 	(1 << 17)
#define RCC_APB1ENR_USART3EN 	(1 << 18)
#define RCC_APB1ENR_UART4EN 	(1 << 19)
#define RCC_APB1ENR_UART5EN 	(1 << 20)
#define RCC_APB1ENR_I2C1EN 	(1 << 21)
#define RCC_APB1ENR_I2C2EN 	(1 << 22)
#define RCC_APB1ENR_CAN1EN 	(1 << 25)
#define RCC_APB1ENR_CAN2EN 	(1 << 26)
#define RCC_APB1ENR_BKPEN 	(1 << 27)
#define RCC_APB1ENR_PWREN 	(1 << 28)
#define RCC_APB1ENR_DACEN 	(1 << 29)

#define FLASH_ACR_LATENCY_SHIFT	0
#define FLASH_ACR_HLFCYA	(1 << 3)
#define FLASH_ACR_PRFTBE	(1 << 4)
#define FLASH_ACR_PRFTBS	(1 << 5)

/********************************************************************************
* INITS 	72MHZ_CLK 	PA_PB_PC_CLK	USART1_CLK		TMR2_CLK   	I2C1_CLK	*
********************************************************************************/
void rcc_init(void)
{
	RCC->CFGR = RCC_CFGR_PLL_SRC | RCC_CFGR_PLLMUL_x9 | RCC_CFGR_SW_HSI | RCC_CFGR_PPRE1_div2;
	RCC->CR = RCC_CR_HSION | RCC_CR_HSEON | (0x10 << RCC_CR_HSITRIM_SHIFT) | RCC_CR_PLLON;
	
	while( !(RCC->CR & RCC_CR_HSERDY) );    							// wait for the HSEREADY flag
	while (!(RCC->CR & RCC_CR_PLLRDY))									// wait for the PLLRDY flag
		;
	FLASH->ACR = FLASH_ACR_PRFTBE | (2 << FLASH_ACR_LATENCY_SHIFT);
	RCC->CFGR = RCC_CFGR_PLL_SRC | RCC_CFGR_PLLMUL_x9 | RCC_CFGR_SW_PLL | RCC_CFGR_PPRE1_div2;
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN |			// CLK_EN PA PB PC USART1
			RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
			RCC_APB2ENR_USART1EN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_I2C1EN;			// CLK_EN TMR2 I2C1
}


/********************************************************************************
* GPIO  DEFINITIONS 	                                                		*
********************************************************************************/
#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT_10MHz 1
#define GPIO_MODE_OUT_2MHz  2
#define GPIO_MODE_OUT_50MHz 3

#define GPIO_CNF_IN_ANALOG	(0 << 2)
#define GPIO_CNF_IN_FLOATING	(1 << 2)
#define GPIO_CNF_IN_PULL	(2 << 2)
#define GPIO_CNF_OUT_GPIO_PULL	(0 << 2)
#define GPIO_CNF_OUT_GPIO_OPEN	(1 << 2)
#define GPIO_CNF_OUT_ALT_PULL	(2 << 2)
#define GPIO_CNF_OUT_ALT_OPEN	(3 << 2)

struct GPIO;

extern struct GPIO * const GPIOA;
extern struct GPIO * const GPIOB;
extern struct GPIO * const GPIOC;
extern struct GPIO * const GPIOD;

void gpio_setup(struct GPIO *gpio, unsigned int bit, uint32_t mode);
void gpio_out(struct GPIO *gpio, unsigned int bit, bool value);
void gpio_Toggle(struct GPIO *gpio, unsigned int bit);
bool gpio_in(struct GPIO *gpio, unsigned int bit);

struct GPIO {
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
};

struct GPIO_BB {
	volatile uint32_t CRL[32];
	volatile uint32_t CRH[32];
	volatile uint32_t IDR[32];
	volatile uint32_t ODR[32];
	volatile uint32_t BSRR[32];
	volatile uint32_t BRR[32];
	volatile uint32_t LCKR[32];
};

struct GPIO    * const GPIOA    = ((struct GPIO *)    0x40010800);
struct GPIO    * const GPIOB    = ((struct GPIO *)    0x40010C00);
struct GPIO    * const GPIOC    = ((struct GPIO *)    0x40011000);
struct GPIO    * const GPIOD    = ((struct GPIO *)    0x40011400);
	   
/********************************************************************************
* GPIO FUNCTIONS	gpio_setup()	gpio_out()	gpio_in()	LED_Toggle()		*
********************************************************************************/
void gpio_setup(struct GPIO *gpio, unsigned int bit, uint32_t mode)
{
	volatile uint32_t *cr;
	uint32_t v;

	if (bit >= 8) {
		cr = &gpio->CRH;
		bit -= 8;
	} else {
		cr = &gpio->CRL;
	}

	v = *cr;
	v &= ~(0xf << (bit * 4));
	v |= (0xf & mode) << (bit * 4);
	*cr = v;
}

void gpio_out(struct GPIO *gpio, unsigned int bit, bool value)
{
	uint32_t v;

	v = gpio->ODR;

	if (value)
		v |= (1 << bit);
	else
		v &= ~(1 << bit);

	gpio->ODR = v;
}

bool gpio_in(struct GPIO *gpio, unsigned int bit)
{
	return gpio->IDR & (1 << bit);
}

void gpio_Toggle(struct GPIO *gpio, unsigned int bit)
{
	gpio->ODR ^= (1 << bit);
}



/********************************************************************************
* SYSTICK DEFINITIONS AND INITS                                           			*
********************************************************************************/

// Define the base addresses for peripherals
#define PERIPH_BASE     ((uint32_t) 0x40000000)
#define SYSTICK_BASE    ((uint32_t) 0xE000E010)

#define GPIOC_BASE      (PERIPH_BASE + 0x11000) // GPIOC base address is 0x40011000
#define GPIOD_BASE      (PERIPH_BASE + 0x11400) // GPIOD base address is 0x40011400
#define GPIOE_BASE      (PERIPH_BASE + 0x11800) // GPIOE base address is 0x40011800
#define RCC_BASE        (PERIPH_BASE + 0x21000) //   RCC base address is 0x40021000

#define STACKINIT       0x20008000
#define DELAY           70000

#define GPIOD   ((GPIO_type *)  GPIOD_BASE)
#define GPIOE   ((GPIO_type *)  GPIOE_BASE)
//#define RCC     ((RCC_type *)     RCC_BASE)
#define SYSTICK ((STK_type *) SYSTICK_BASE)

/*
 * Register Addresses
 */
typedef struct
{
	uint32_t CRL;      /* GPIO port configuration register low,      Address offset: 0x00 */
	uint32_t CRH;      /* GPIO port configuration register high,     Address offset: 0x04 */
	uint32_t IDR;      /* GPIO port input data register,             Address offset: 0x08 */
	uint32_t ODR;      /* GPIO port output data register,            Address offset: 0x0C */
	uint32_t BSRR;     /* GPIO port bit set/reset register,          Address offset: 0x10 */
	uint32_t BRR;      /* GPIO port bit reset register,              Address offset: 0x14 */
	uint32_t LCKR;     /* GPIO port configuration lock register,     Address offset: 0x18 */
} GPIO_type;


typedef struct
{
	uint32_t CR;       /* RCC clock control register,                Address offset: 0x00 */
	uint32_t CFGR;     /* RCC clock configuration register,          Address offset: 0x04 */
	uint32_t CIR;      /* RCC clock interrupt register,              Address offset: 0x08 */
	uint32_t APB2RSTR; /* RCC APB2 peripheral reset register,        Address offset: 0x0C */
	uint32_t APB1RSTR; /* RCC APB1 peripheral reset register,        Address offset: 0x10 */
	uint32_t AHBENR;   /* RCC AHB peripheral clock enable register,  Address offset: 0x14 */
	uint32_t APB2ENR;  /* RCC APB2 peripheral clock enable register, Address offset: 0x18 */
	uint32_t APB1ENR;  /* RCC APB1 peripheral clock enable register, Address offset: 0x1C */
	uint32_t BDCR;     /* RCC backup domain control register,        Address offset: 0x20 */
	uint32_t CSR;      /* RCC control/status register,               Address offset: 0x24 */
	uint32_t AHBRSTR;  /* RCC AHB peripheral clock reset register,   Address offset: 0x28 */
	uint32_t CFGR2;    /* RCC clock configuration register2,         Address offset: 0x2C */
} RCC_type;

typedef struct
{
	uint32_t CSR;      /* SYSTICK control and status register,       Address offset: 0x00 */
	uint32_t RVR;      /* SYSTICK reload value register,             Address offset: 0x04 */
	uint32_t CVR;      /* SYSTICK current value register,            Address offset: 0x08 */
	uint32_t CALIB;    /* SYSTICK calibration value register,        Address offset: 0x0C */
} STK_type;

/*
 * SysTick interrupt handler function
 *
 * If systick is enabled with interrupt, this function is used
 */

volatile uint32_t millis;

void systick_handler(void){
	millis++;
	Systick_Loop();	
}

void delay_ms(volatile uint32_t ms)
{
    uint32_t lm = millis;
    while ((millis - lm) < ms);
}

// ***  If systick is enabled with NO_interrupt, this function Could Be Used  *** //
void delay_ms_no_int(volatile uint32_t s)
{
	for(s; s>0; s--){
		while(!(SYSTICK->CSR & (1 << 16))); // Wait until COUNTFLAG is 1	systick without interrupt
	}
}


/********************************************************************************
* SYSTICK INIT				                                           			*
********************************************************************************/
void systick_init(uint32_t s, uint8_t en)
{
	SYSTICK->CSR |= 0x00000; 
	SYSTICK->CSR |= (en << 1);
	SYSTICK->RVR = s;
	SYSTICK->CVR = 0;
	SYSTICK->CSR |= (1 << 0);
}



