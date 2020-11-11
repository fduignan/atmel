#include "../include/ATSAML10E16A.h"
void delay(volatile uint32_t dly)
{
	while(dly--);
}
int main()
{
    PORT->GROUP[0].DIRSET = 1; // make Port A bit 0 an output
    while(1)
    {
        PORT->GROUP[0].OUTTGL = 1; // Toggle Port A bit 0
        delay(100000);   // Wait
    }
}
