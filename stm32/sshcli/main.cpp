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
#include <peripheral/Timer.h>
#include <util/Logger.h>
#include <cli/UARTOutputStream.h>
#include "DemoCLISessionContext.h"

//UART console
UART* g_cliUART = NULL;
Logger g_log;
UARTOutputStream g_uartStream;
DemoCLISessionContext g_uartCliContext;

//need a total of 4 descriptors in the ring
static volatile uint8_t buffers[4][2048];
static volatile edma_rx_descriptor_t rx_dma_descriptor[4];

void InitClocks();
void InitLog();
void InitUART();
void InitEthernet();

int main()
{
	//Hardware setup
	InitClocks();
	InitUART();
	InitLog();
	InitEthernet();

	//Initialize the CLI on the console UART interface
	g_uartStream.Initialize(g_cliUART);
	g_uartCliContext.Initialize(&g_uartStream, "admin");

	//Enable interrupts only after all setup work is done
	EnableInterrupts();

	//Show the initial prompt
	//g_uartCliContext.PrintPrompt();

	//TODO: use loopback mode to sanity check

	//Main event loop
	int nextRxFrame = 0;
	uint32_t numRxFrames = 0;
	uint32_t numRxBad = 0;
	while(1)
	{
		//Check if we have a new frame ready to process
		auto& desc = rx_dma_descriptor[nextRxFrame];
		if( (desc.RDES0 & 0x80000000) == 0)
		{
			numRxFrames ++;

			int len = (desc.RDES0 >> 16) & 0x3fff;
			g_log("Got a frame (%d bytes) at descriptor %d\n", len, nextRxFrame);
			LogIndenter li(g_log);

			g_log("RDES = %08x %08x %08x %08x \n", desc.RDES0, desc.RDES1, desc.RDES2, desc.RDES3);
			if(desc.RDES0 & 0x2)
			{
				g_log(Logger::ERROR, "CRC error\n");
				numRxBad ++;
			}

			g_log("CRC / total: %d / %d", numRxBad, numRxFrames);
			for(int i=0; i<len; i++)
			{
				if( (i & 63) == 0)
					g_cliUART->Printf("\n    ");
				g_cliUART->Printf("%02x ", buffers[nextRxFrame][i]);
			}
			g_cliUART->Printf("\n");

			//Done processing the frame, return it to the MAC
			desc.RDES0 |= 0x80000000;
			EDMA.DMARPDR = 0;

			if(nextRxFrame == 3)
				nextRxFrame = 0;
			else
				nextRxFrame ++;

			//Reset perf counters so they don't wrap
			if(numRxFrames >= 0x80000000)
			{
				numRxFrames = 0;
				numRxBad = 0;
			}

			//If we get a CRC error in the first 100 frames, reset the MAC
			if( (numRxFrames < 100) && numRxBad)
			{
				g_log(Logger::ERROR, "CRC error seen on early packet, resetting MAC...\n");
				numRxFrames = 0;
				numRxBad = 0;
				InitEthernet();
			}
		}

		/*
		//Wait for an interrupt
		asm("wfi");

		if(g_cliUART->HasInput())
			g_uartCliContext.OnKeystroke(g_cliUART->BlockingRead());
		*/
	}

	return 0;
}

void InitClocks()
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
}

void InitLog()
{
	//APB1 is 43.75 MHz, divide down to get 1 kHz ticks
	static Timer logtim(&TIM2, Timer::FEATURE_GENERAL_PURPOSE, 43750);

	g_log.Initialize(g_cliUART, &logtim);
	g_log("UART logging ready\n");
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PA0 for UART4 transmit and PA3 for USART2 RX
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOA, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 8);
	GPIOPin uart_rx(&GPIOA, 3, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);
	static UART uart(&UART4, &USART2, 380);
	volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	*NVIC_ISER1 = 0x40;
	g_cliUART = &uart;
}

void InitEthernet()
{
	g_log("Initializing Ethernet\n");

	//Initialize the Ethernet pins. AF11 on all pins
	GPIOPin rmii_refclk(&GPIOA, 1, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_mdio(&GPIOA, 2, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_crs_dv(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_tx_en(&GPIOB, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_txd0(&GPIOB, 12, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_txd1(&GPIOB, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_mdc(&GPIOC, 1, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_rxd0(&GPIOC, 4, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_rxd1(&GPIOC, 5, GPIOPin::MODE_PERIPHERAL, 11);

	//Ignore the MDIO bus for now

	//Select RMII mode
	//TODO: put this in RCCHelper
	//Disable all Ethernet clocks (except 1588 which we don't need, leave it off to save power), reset MAC
	RCC.AHB1ENR &= ~(RCC_AHB1_EMAC | RCC_AHB1_EMAC_TX | RCC_AHB1_EMAC_RX);
	RCC.AHB1RSTR |= RCC_AHB1_EMAC;

	//Enable SYSCFG before changing any settings on it
	RCC.APB2ENR |= RCC_APB2_SYSCFG;

	//Enable RMII
	SYSCFG.PMC |= ETH_MODE_RMII;

	//Clear resets
	RCC.AHB1RSTR &= ~RCC_AHB1_EMAC;

	//Enable Ethernet clocks (except 1588 since we don't use that)
	RCC.AHB1ENR |= RCC_AHB1_EMAC | RCC_AHB1_EMAC_TX | RCC_AHB1_EMAC_RX | RCC_AHB1_PTP;

	//Wait for DMA to finish power-on reset
	while((EDMA.DMABMR & 1) == 1)
	{}

	//Receive all frames. promiscuous mode
	EMAC.MACFFR = 0x80000001;

	/*
		Create a simple DMA ring with a single descriptor in it
		see 42.6.8
		42.8
	 */
	for(int i=0; i<4; i++)
	{
		rx_dma_descriptor[i].RDES0 = 0x80000000;
		if(i == 3)
			rx_dma_descriptor[i].RDES1 = 0x00008800;
		else
			rx_dma_descriptor[i].RDES1 = 0x00000800;

		rx_dma_descriptor[i].RDES2 = (uint32_t)&buffers[i];
		rx_dma_descriptor[i].RDES3 = 0;
	}

	//Configure DMA descriptor list
	EDMA.DMARDLAR = &rx_dma_descriptor[0];

	//Select mode: 100/full, RX enabled, no TX, no carrier sense
	EMAC.MACCR = 0x1c804;

	//Poll demand DMA RX
	//EDMA.DMARPDR = 0;

	//Enable actual DMA in DMAOMR bits 1/13
	EDMA.DMAOMR |= 2;
}
