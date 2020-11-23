#include "../include/ATSAML10E16A.h"
// Get the system running at 16MHz (without the PLL, just using OSC16M)
void delay(volatile uint32_t dly)
{
	while(dly--);
}
void initSysTick()
{
	SysTick->SYST_RVR = 16000-1; // Divide 32MHz clock by 32000 to get 1ms timebase
	SysTick->SYST_CVR = 100; // start at a low number
	SysTick->SYST_CSR = 7;   // enable counting and interrupts
}
void Systick_Handler()
{
	static int timer=0;
	timer++;
	PORT->GROUP[0].OUTTGL = 1; // Toggle Port A bit 0

}
void initClock()
{
	// Configure flash for 2 wait states during read
	NVMCTRL->CTRLB |= (2 << 1);
	// Enable performance level 2
	PM->PLCFG = 2;
    while( (PM->INTFLAG& 1) == 0);
	// Lets get this going at 16MHz
	OSCCTRL->OSC16MCTRL |= (1<<3) + (1 << 2);
}
int main()
{
	disable_interrupts(); // seems that they are enabled on boot.	
	initClock();
	initSysTick();		  // set systick going
	PORT->GROUP[0].DIRSET = 1; // make Port A bit 0 an output    
	enable_interrupts(); // re-enable interrupts
    while(1)
    {
		asm(" wfi "); // sleep
    }
}
