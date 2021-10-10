/***********************************************************************************************************************
*                                                                                                                      *
* staticnet v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2021 Andrew D. Zonenberg and contributors                                                              *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include "sshcli.h"
#include <peripheral/Flash.h>
#include <peripheral/GPIO.h>
#include <peripheral/RCC.h>

//UART for the local console port
UART* g_cliUART = NULL;

int main()
{
	//Configure the flash with wait states and prefetching before making any changes to the clock setup.
	//A bit of extra latency is fine, the CPU being faster than flash is not.
	Flash::SetConfiguration(true, true, 175, Flash::RANGE_2V7);

	//Set up the main system PLL. Input from HSI clock (16 MHz RC)
	RCCHelper::InitializePLLFromInternalOscillator(
		8,		//Pre-divider of 8 = 2 MHz input to PLL
		175,	//Multiply by 175 = 350 MHz VCO
		2,		//Divide VCO by 2 for CPU clock (175 MHz)
		10,		//Divide VCO by 10 for RNG (35 MHz)
		5,		//Divide VCO by 4 for MIPI DSI clock (70 MHz, but ignored since we don't have MIPI hardware)
		1,		//Divide CPU clock by 1 to get AHB clock (175 MHz)
		4,		//Divide AHB clock by 4 to get APB1 clock (43.75 MHz)
		2);		//Divide AHB clock by 2 to get APB2 clock (87.5 MHz)

	//TODO: Debug clock output on MCO1 (PA8) pmod, see 5.2.10?

	//Configure UART4_TX (AF8 on PA0) and USART2_RX (AF7 on PA3)
	GPIOPin uart_tx(&GPIOA, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 8);
	GPIOPin uart_rx(&GPIOA, 3, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);

	UART uart(&UART4, &USART2, 380);
	g_cliUART = &uart;

	//Enable IRQ38. This is bit 6 of NVIC_ISER1.
	//TODO: make nice API for this
	volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	*NVIC_ISER1 = 0x40;
	EnableInterrupts();

	//Output a character
	uart.PrintBinary('A');

	while(1)
	{
		if(g_cliUART->HasInput())
		{
			g_cliUART->Printf("Got input: %c\n", g_cliUART->BlockingRead());
		}
	}

	return 0;
}
