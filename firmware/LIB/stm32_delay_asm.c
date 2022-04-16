//Delay for stm32f103c8 based on asm loop. tested with 72Mhz system frequency only
#include "stm32_delay_asm.h"
__asm void _delay_loop(uint32_t __count)		//asm function, executing in about 3 clocks (??)
{
	//unsigned long r0;

loop	 SUBS	r0, r0, #1			//decrement
			 //NOP
			 BNE		loop							
			 BX			lr
}

void _delay_us(double __us)
{
	uint32_t __ticks;		//
	
	double __tmp = ((SystemCoreClock)/6e6) * __us;
	__ticks = (uint32_t)__tmp;
	_delay_loop(__ticks);
}

void _delay_ms(double __ms)
{	
	uint32_t __ticks;
	double __tmp = ((SystemCoreClock)/6e3) * __ms;
	__ticks = (uint32_t)__tmp;
	_delay_loop(__ticks);
}
