/*   Project name : LED_control
*    Julia's project 1
*    Control LED display */
/*---------------------------------------------------------------------------------------*/

#include <stdio.h>

#include "stm32f0xx.h"          // File name depends on device used
#include "RTE_Components.h"      // Component selection 

extern void stdout_init (void);

volatile uint32_t msTicks;         // Counter for millisecond Interval

#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NUCLEO Board */
#define USER    1

/*--------------------------------------------------------------------------------*/
// SysTick Interrupt Handler
/*--------------------------------------------------------------------------------*/
void SysTick_Handler (void) 
{       
    msTicks++;           // Increment Counter
}

/*--------------------------------------------------------------------------------*/
// Delay: delay a number of Systicks
/*--------------------------------------------------------------------------------*/
void Delay(uint32_t dlyTicks){
	uint32_t currentTicks;
	
	currentTicks = msTicks;
	while( (msTicks - currentTicks) < dlyTicks ){
		  __NOP();
	}
}
		
/*----------------------------------------------------------------------------------------------*/
// Configure SystemCoreClock using HSI

//#define  RCC_CR_HSION      ((uint32_t)0x00000001U)      RCC_CR bit 0; Internal High Speed clock enable 
//#define  RCC_CR_HSIRDY     ((uint32_t)0x00000002U)      RCC_CR bit 1; Internal High Speed clock ready flag 
//#define  RCC_CR_PLLON      ((uint32_t)0x01000000U)      RCC_CR bit 24; PLL enable
//#define  RCC_CR_PLLRDY     ((uint32_t)0x02000000U)      RCC_CR bit 25; PLL clock ready flag

//#define  RCC_CFGR_SW_HSI             ((uint32_t)0x00000000U)    RCC_CFGR bit 0,1; HSI selected as system clock 
//#define  RCC_CFGR_SW_PLL             ((uint32_t)0x00000002U)    RCC_CFGR bit 0,1; PLL selected as system clock	
	
//#define  RCC_CFGR_SWS                ((uint32_t)0x0000000CU)    RCC_CFGR bit 2,3; SWS[1:0] bits System Clock Switch Status	
//#define  RCC_CFGR_SWS_HSI            ((uint32_t)0x00000000U)    RCC_CFGR bit 2,3 = 00; HSI oscillator used as system clock 

//#define  RCC_CFGR_HPRE_DIV1          ((uint32_t)0x00000000U)    RCC_CFGR bit 7:4; SYSCLK not divided 
//#define  RCC_CFGR_PPRE_DIV1          ((uint32_t)0x00000000U)    RCC_CFGR bit 10:8; HCLK not divided 

//#define  RCC_CFGR_PLLSRC             ((uint32_t)0x00018000U)    RCC_CFGR bit 16:15; PLL entry clock source	
//#define  RCC_CFGR_PLLSRC_HSI_PREDIV  ((uint32_t)0x00008000U)    RCC_CFGR bit 16:15; HSI/PREDIV clock selected as PLL entry clock source 		

//#define  RCC_CFGR_PLLXTPRE           ((uint32_t)0x00020000U)    RCC_CFGR bit 17; HSE divider for PLL entry	

//#define  RCC_CFGR_PLLMUL             ((uint32_t)0x003C0000U)    RCC_CFGR bit 21:18; PLLMUL[3:0] bits PLL multiplication factor	
//#define  RCC_CFGR_PLLMUL12           ((uint32_t)0x00280000U)    RCC_CFGR bit 21:18; PLL input clock*12
	
//#define  RCC_CFGR2_PREDIV_DIV2       ((uint32_t)0x00000001U)    RCC_CFGR2 bit 3:0; PREDIV input clock divided by 2	
/*------------------------------------------------------------------------------------------------*/

void ConfigureSystemClock(void) {
   
	RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable 8MHz HSI RC
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                 // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // Set HSI as system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

	// Set Flash access control register
  FLASH->ACR  = FLASH_ACR_PRFTBE;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_LATENCY;                         // Flash 1 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // set HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE_DIV1;                         // set PCLK = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL clk 

  // PLL configuration:  = HSI/2 * 12 = 48 MHz
	// HSI used as PLL clock source : SystemCoreClock = HSI/PREDIV * PLLMUL 
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);         // Clear relative bits
  
  RCC->CFGR2 = (RCC_CFGR2_PREDIV_DIV2);                            // PREDIV input clock divided by 2
  RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_PLLMUL12);   // HSI/PREDIV clock selected as PLL entry clock source & PLL input clock*12


  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready
	
  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src
	
}

/*----------------------------------------------------------------------------------------------*/
// GPIOA_Init
// PA.5 to control green LED(LD2)
// set PA.5 is output pin with push-pull output, medium speed, no internal pull-up & pull down
/*----------------------------------------------------------------------------------------------*/

void GPIOA_Init (void) {

  RCC->AHBENR |=  (1ul << 17);                  /* Enable GPIOA clock         */

  /* Configure LED (PA.5) pins as push-pull output, No pull-up, pull-down */
  GPIOA->MODER   &= ~((3ul << 2*5));             // set GPIOA pin 5 as output pin
  GPIOA->MODER   |=  ((1ul << 2*5));
  
	GPIOA->OTYPER  &= ~((1ul <<   5));             // set pin 5 as O/P push-pull 
  
	GPIOA->OSPEEDR &= ~((3ul << 2*5));             // set pin 5 as medium speed O/P 
  GPIOA->OSPEEDR |=  ((1ul << 2*5));
  
	GPIOA->PUPDR   &= ~((3ul << 2*5));             // set pin 5 as no pull-up & pull-down
  
}

/*--------------------------------------------------------------------------------*/
// LED_on();    Turn on LED
/*--------------------------------------------------------------------------------*/
void LED_on(void){                               // set pin 5 = 1 to turn on LED
	GPIOA->BSRR |= ((1ul << 5));
	
}

/*--------------------------------------------------------------------------------*/
// LED_off();    Turn off LED
/*--------------------------------------------------------------------------------*/
void LED_off(void){
	GPIOA->BSRR |= (1ul << (5+16));              // set pin 5 = 0 to turn off LED
	
}

/*--------------------------------------------------------------------------------*/
// Button_Init(void)        ;Initialize button
// PC.13 to control User botton, set PC.13 is input pin 
/*--------------------------------------------------------------------------------*/
void Button_Init(void) {

  RCC->AHBENR |=  (1ul << 19);                  // Enable GPIOC clock       
  GPIOC->MODER &= ~(3ul << 2*13);               // Set PC.13 is input  
   
}

/*------------------------------------------------------------------------------*/
//uint32_t Button_GetState(void)
// Get USER button (PC.13) state
// return: 1 means USER key pressed
/*------------------------------------------------------------------------------*/
uint32_t Button_GetState (void) {

  uint32_t val = 0;

  if ((GPIOC->IDR & (1ul << 13)) == 0) {         //When USER button pressed PC.13=0
    val |= USER;                                 // set USER button pressed
  }
  return (val);

}

/*--------------------------------------------------------------------------------*/
// The processor clock is initialized by CMSIS startup + system file
/*--------------------------------------------------------------------------------*/
int main (void) {        // User application starts here
    
	ConfigureSystemClock();     // Configure system clock
	SystemCoreClockUpdate();    // Set system clock value to variable SystemCoreClock 

	GPIOA_Init();               // Initialize PA.5 to control LED
	Button_Init();              // Initialize PC.13 to detect USER button 
	
	stdout_init();              // Initialize USART 2(PA3 to USART2_RX,PA2 to USART2_TX)  
			
	SysTick_Config(SystemCoreClock/1000);       // System Tick Initializes,set up SysTick 1ms interrupt
	
	for(;;){
		
		printf("This is LED on/off display. \n\r");
		printf("Turn on LED for 4 sec. \n\r");
		LED_on();
		Delay(4000);
		
		printf("Turn off LED for 2 sec. \n\r");
		LED_off();
		Delay(2000);
		
		printf("USER button Detect. \n\r");              
		do{                                    // Wait while holding USER button
			LED_on();
		  Delay(200);
		  LED_off();
		  Delay(200);
	   	printf("LED display on/off until USER button pressed. \n\r");
	  }while (Button_GetState()== 0); 
		
		printf("USER button was pressed. \n\r");
	}
	
}