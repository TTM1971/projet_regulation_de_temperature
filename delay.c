/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */

#include "delay.h"

extern volatile uint32_t g_systickCounter;

void delayMicroseconds(uint32_t period)
{
	g_systickCounter = period;
		    while (g_systickCounter != 0U)
		    {
		    }
};

void delay_ms(uint32_t period_ms)
{
	g_systickCounter = period_ms*100;
			    while (g_systickCounter != 0U)
			    {
			    }
}
