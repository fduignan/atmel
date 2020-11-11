#include <stdint.h>
#include "../include/ATSAML10E16A.h"
 
void init(void);
void Default_Handler(void);
int main(void);
// The following are 'declared' in the linker script
extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;
// the section "vectors" is placed at the beginning of flash 
// by the linker script
const void * Vectors[] __attribute__((section(".vectors"))) ={
	(void *)0x20000000 + 16384, 	/* Top of stack  */ 
	init,   		    /* Reset Handler -15 */
	Default_Handler,	/* NMI -14 */
	Default_Handler,	/* Hard Fault -13 */
	Default_Handler,	/* MemManage -12 */
	Default_Handler,	/* BusFault  -11 */
	Default_Handler,	/* UsageFault -10 */
	Default_Handler,	/* Reserved -9 */ 
	Default_Handler,	/* Reserved -8 */
	Default_Handler,	/* Reserved -7 */
	Default_Handler,	/* Reserved -6 */
	Default_Handler,	/* SVCall -5 */
	Default_Handler,	/* Reserved -4 */
	Default_Handler,	/* Debug -3 */
	Default_Handler,	/* PendSV -2 */
	Default_Handler,	/* SysTick -1 */	
/* External interrupt handlers follow */
	Default_Handler, 	/* 0: Power and clock interrupts */
	Default_Handler, 	/* 1: WDT */
	Default_Handler, 	/* 2: RTC */
	Default_Handler, 	/* 3: EIC_0 */
	Default_Handler, 	/* 4: EIC_1 */
	Default_Handler, 	/* 5: EIC_2 */
	Default_Handler, 	/* 6: EXT_3 */
	Default_Handler, 	/* 7: EIC_OTHER */
	Default_Handler, 	/* 8: FreqM */
	Default_Handler, 	/* 9: NVMCTRL */
	Default_Handler, 	/* 10: PORT */
	Default_Handler, 	/* 11: DMAC_0*/
	Default_Handler, 	/* 12: DMAC_1 */
	Default_Handler, 	/* 13: DMAC_2 */
	Default_Handler, 	/* 14: DMAC_3 */
	Default_Handler, 	/* 15: DMA_OTHER */
	Default_Handler, 	/* 16: EVSYS_0 */
	Default_Handler, 	/* 17: EVSYS_1 */
	Default_Handler, 	/* 18: EVSYS_2 */
	Default_Handler, 	/* 19: EVSYS_3 */
	Default_Handler, 	/* 20: EVSYS_NSCHK */
	Default_Handler, 	/* 21: PAC */
	Default_Handler, 	/* 22: SERCOM0_0 */
	Default_Handler, 	/* 23: SERCOM0_1 */
	Default_Handler, 	/* 24: SERCOM0_2 */
	Default_Handler, 	/* 25: SERCOM0_OTHER */
	Default_Handler, 	/* 26: SERCOM1_0 */
	Default_Handler, 	/* 27: SERCOM1_1 */
	Default_Handler, 	/* 28: SERCOM1_2 */
	Default_Handler, 	/* 29: SERCOM1_OTHER */
	Default_Handler, 	/* 30: SERCOM2_0 */
	Default_Handler, 	/* 31: SERCOM2_1 */
	Default_Handler, 	/* 32: SERCOM2_2 */
    Default_Handler, 	/* 33: SERCOM2_OTHER */
    Default_Handler, 	/* 34: TC0 */
    Default_Handler, 	/* 35: TC1 */
    Default_Handler, 	/* 36: TC2 */
    Default_Handler, 	/* 37: ADC_OTHER */
    Default_Handler, 	/* 38: ADC_RESRDY */
    Default_Handler, 	/* 39: AC */
    Default_Handler, 	/* 40: DAC_UNDERRUN_A */    
    Default_Handler, 	/* 41: DAC_EMPTY */
    Default_Handler, 	/* 42: Reserved */
    Default_Handler, 	/* 43: TRNG */
    Default_Handler 	/* 44: TRAM */    
};
/*void initClock()
{
// This is potentially a dangerous function as it could
// result in a system with an invalid clock signal - result: a stuck system
        // Set the PLL up
        // First ensure PLL is disabled
        RCC->CR &= ~(1<<24);
        while( (RCC->CR & (1 <<25))); // wait for PLL ready to be cleared
        
  // Warning here: if system clock is greater than 24MHz then wait-state(s) need to be
        // inserted into Flash memory interface
        Flash->ACR |= (1 << 0);
        Flash->ACR &=~((1 << 2) | (1<<1));
        // Turn on FLASH prefetch buffer
        Flash->ACR |= (1 << 4);
        // set PLL multiplier to 12 (yielding 48MHz)
        RCC->CFGR &= ~((1<<21) | (1<<20) | (1<<19) | (1<<18));
        RCC->CFGR |= ((1<<21) | (1<<19) ); 

        // Need to limit ADC clock to below 14MHz so will change ADC prescaler to 4
        RCC->CFGR |= (1<<14);

        // and turn the PLL back on again
        RCC->CR |= (1<<24);        
        // set PLL as system clock source 
        RCC->CFGR |= (1<<1);
}*/
void init()
{
// do global/static data initialization
	unsigned char *src;
	unsigned char *dest;
	unsigned len;
    //initClock();
	src= &INIT_DATA_VALUES;
	dest= &INIT_DATA_START;
	len= &INIT_DATA_END-&INIT_DATA_START;
	while (len--)
		*dest++ = *src++;
// zero out the uninitialized global/static variables
	dest = &BSS_START;
	len = &BSS_END - &BSS_START;
	while (len--)
		*dest++=0;
    
	main();
}

void Default_Handler()
{
	while(1);
}
