#include "../include/ATSAML10E16A.h"
// The default clock speed for SysTick seems to be 2MHz.  
void delay(volatile uint32_t dly)
{
	while(dly--);
}
void initSysTick()
{
	SysTick->SYST_RVR = 32000-1; // Divide 32MHz clock by 32000 to get 1ms timebase
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
	// Lets get this going at 32MHz
	
	
}
int main()
{
	disable_interrupts(); // seems that they are enabled on boot.
	delay(1000000);
	initClocks();
	initSysTick();		  // set systick going
	PORT->GROUP[0].DIRSET = 1; // make Port A bit 0 an output    
	enable_interrupts(); // re-enable interrupts
    while(1)
    {
		asm(" wfi "); // sleep
    }
}
