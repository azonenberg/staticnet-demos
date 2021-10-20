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

//UART console
UART* g_cliUART = NULL;
Logger g_log;
UARTOutputStream g_uartStream;
DemoCLISessionContext g_uartCliContext;
Timer* g_logTimer;

//SPI interface
GPIOPin* g_spiCS = NULL;
SPI* g_spi = NULL;

//need a total of 4 descriptors in the ring
static volatile uint8_t buffers[4][2048];
static volatile edma_rx_descriptor_t rx_dma_descriptor[4];

void InitClocks();
void InitUART();
void InitCLI();
void InitLog();
void DetectHardware();
void InitSPI();
//void InitEthernet();
void InitEthernetMacAndDMA();
bool TestEthernet(uint32_t num_frames);

uint8_t GetFPGAStatus();

uint8_t g_macAddress[6] = {0};

bool g_hasRmiiErrata = false;

int g_nextRxFrame = 0;

int main()
{
	//Hardware setup
	InitClocks();
	InitUART();
	InitCLI();
	InitLog();
	DetectHardware();
	InitSPI();
	//InitEthernet();

	//Enable interrupts only after all setup work is done
	EnableInterrupts();

	//Show the initial prompt
	g_uartCliContext.PrintPrompt();

	//Main event loop
	int nextRxFrame = 0;
	uint32_t numRxFrames = 0;
	uint32_t numRxBad = 0;
	while(1)
	{
		/*
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
		*/

		//Wait for an interrupt
		//asm("wfi");

		if(g_cliUART->HasInput())
			g_uartCliContext.OnKeystroke(g_cliUART->BlockingRead());
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
	g_logTimer = &logtim;

	g_log.Initialize(g_cliUART, &logtim);
	g_log("UART logging ready\n");
}

void DetectHardware()
{
	g_log("Identifying hardware\n");
	LogIndenter li(g_log);

	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	if(device == 0x451)
	{
		//Look up the stepping number
		const char* srev = NULL;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
				g_hasRmiiErrata = true;
				break;

			case 0x1001:
				srev = "Z";
				break;

			default:
				srev = "(unknown)";
		}

		uint8_t pkg = (PKG_ID >> 8) & 0x7;
		switch(pkg)
		{
			case 7:
				g_log("STM32F767 / 777 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 6:
				g_log("STM32F769 / 779 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 5:
				g_log("STM32F767 / 777 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 4:
				g_log("STM32F769 / 779 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 3:
				g_log("STM32F778 / 779 WLCSP180 rev %s (0x%04x)\n", srev, rev);
				break;
			case 2:
				g_log("STM32F767 / 777 LQFP144 rev %s (0x%04x)\n", srev, rev);
				break;
			case 1:
				g_log("STM32F767 / 777 LQFP100 rev %s (0x%04x)\n", srev, rev);
				break;
			default:
				g_log("Unknown/reserved STM32F76x/F77x rev %s (0x%04x)\n", srev, rev);
				break;
		}
		g_log("512 kB total SRAM, 128 kB DTCM, 16 kB ITCM, 4 kB backup SRAM\n");
		g_log("%d kB Flash\n", F_ID);

		//U_ID fields documented in 45.1 of STM32 programming manual
		uint16_t waferX = U_ID[0] >> 16;
		uint16_t waferY = U_ID[0] & 0xffff;
		uint8_t waferNum = U_ID[1] & 0xff;
		char waferLot[8] =
		{
			static_cast<char>((U_ID[1] >> 24) & 0xff),
			static_cast<char>((U_ID[1] >> 16) & 0xff),
			static_cast<char>((U_ID[1] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 24) & 0xff),
			static_cast<char>((U_ID[2] >> 16) & 0xff),
			static_cast<char>((U_ID[2] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 0) & 0xff),
			'\0'
		};
		g_log("Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);

		if(g_hasRmiiErrata)
			g_log("RMII RXD0 errata present\n");
	}
	else
		g_log(Logger::WARNING, "Unknown device (0x%06x)\n", device);
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

	//Clear screen and move cursor to X0Y0
	uart.Printf("\x1b[2J\x1b[0;0H");
}

void InitCLI()
{
	g_log("Initializing CLI\n");

	//Initialize the CLI on the console UART interface
	g_uartStream.Initialize(g_cliUART);
	g_uartCliContext.Initialize(&g_uartStream, "admin");
}

void InitSPI()
{
	g_log("Initializing SPI interface\n");

	static GPIOPin spi_cs_n(&GPIOH, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	GPIOPin spi_sck(&GPIOH, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	GPIOPin spi_miso(&GPIOH, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	GPIOPin spi_mosi(&GPIOF, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	g_spiCS = &spi_cs_n;
	spi_cs_n = 1;

	//APB2 is 87.5 MHz, /4 is 21.875 MHz
	static SPI spi(&SPI5, true, 4);
	g_spi = &spi;
}

uint8_t GetFPGAStatus()
{
	//Get the status register
	*g_spiCS = 0;
	g_spi->BlockingWrite(REG_STATUS);
	uint8_t sr = g_spi->BlockingRead();
	*g_spiCS = 1;

	return sr;
}

void InitEthernet()
{
	g_log("Initializing Ethernet\n");
	LogIndenter li(g_log);

	//Wait for the FPGA to be up and have our MAC address
	g_log("Polling FPGA status\n");
	while(true)
	{
		auto sr = GetFPGAStatus();
		if(sr & 1)
		{
			g_log(Logger::ERROR, "FPGA failed to get MAC address\n");
			while(1)
			{}
		}

		//address is ready
		if(sr & 2)
			break;
	}
	g_log("FPGA is up and has MAC address ready for us\n");

	//Read the MAC address from the FPGA
	*g_spiCS = 0;
	g_spi->BlockingWrite(REG_MAC_ADDR);
	for(int i=0; i<6; i++)
		g_macAddress[i] = g_spi->BlockingRead();
	*g_spiCS = 1;

	g_log("Our MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n",
		g_macAddress[0], g_macAddress[1], g_macAddress[2], g_macAddress[3], g_macAddress[4], g_macAddress[5]);

	//Initialize the Ethernet pins. AF11 on all pins
	g_log("Initializing Ethernet pins\n");
	GPIOPin rmii_refclk(&GPIOA, 1, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_mdio(&GPIOA, 2, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_crs_dv(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_tx_en(&GPIOB, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_txd0(&GPIOB, 12, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_txd1(&GPIOB, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 11);
	GPIOPin rmii_mdc(&GPIOC, 1, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_rxd0(&GPIOC, 4, GPIOPin::MODE_PERIPHERAL, 11);
	GPIOPin rmii_rxd1(&GPIOC, 5, GPIOPin::MODE_PERIPHERAL, 11);

	//Enable SYSCFG before changing any settings on it
	RCC.APB2ENR |= RCC_APB2_SYSCFG;

	//Ignore the MDIO bus for now

	//Set up all of the SFRs for the Ethernet IP itself
	InitEthernetMacAndDMA();

	//See what part and silicon rev we are
	if(g_hasRmiiErrata)
	{
		g_log("RMII errata workaround: testing for correct init\n");
		bool ok = false;
		int nmax = 5;
		for(int i=0; i<nmax; i++)
		{
			if(TestEthernet(5))
			{
				g_log("RMII errata workaround: init complete after %d resets, enabling RX path in FPGA\n", i);

				*g_spiCS = 0;
				g_spi->BlockingWrite(REG_RX_ENABLE);
				g_spi->WaitForWrites();
				*g_spiCS = 1;
				return;
			}
		}

		g_log(Logger::ERROR, "Still couldn't get Ethernet working reliably after %d resets\n", nmax);
	}
}

void InitEthernetMacAndDMA()
{
	if(g_hasRmiiErrata)
	{
		g_log("RMII errata workaround: disabling RX path in FPGA\n");
		*g_spiCS = 0;
		g_spi->BlockingWrite(REG_RX_DISABLE);
		g_spi->WaitForWrites();
		*g_spiCS = 1;
	}

	//Select RMII mode
	//TODO: put this in RCCHelper
	//Disable all Ethernet clocks (except 1588 which we don't need, leave it off to save power), reset MAC
	RCC.AHB1ENR &= ~(RCC_AHB1_EMAC | RCC_AHB1_EMAC_TX | RCC_AHB1_EMAC_RX);
	RCC.AHB1RSTR |= RCC_AHB1_EMAC;

	//Enable RMII
	SYSCFG.PMC |= ETH_MODE_RMII;

	//Clear resets
	RCC.AHB1RSTR &= ~RCC_AHB1_EMAC;

	//Enable Ethernet clocks (except 1588 since we don't use that)
	RCC.AHB1ENR |= RCC_AHB1_EMAC | RCC_AHB1_EMAC_TX | RCC_AHB1_EMAC_RX | RCC_AHB1_PTP;

	//Wait for DMA to finish power-on reset
	g_log("Waiting for Ethernet DMA reset\n");
	while((EDMA.DMABMR & 1) == 1)
	{}

	g_log("Clearing performance counters\n");
	EMAC.MMCCR = 1;

	//Receive all frames. promiscuous mode
	EMAC.MACFFR = 0x80000001;

	g_log("Setting up DMA ring\n");

	//Initialize DMA ring buffers
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
	EDMA.DMARDLAR = &rx_dma_descriptor[0];

	//Select mode: 100/full, RX enabled, no TX, no carrier sense
	EMAC.MACCR = 0x1c804;

	//Poll demand DMA RX
	EDMA.DMARPDR = 0;

	//Enable actual DMA in DMAOMR bits 1/13
	EDMA.DMAOMR |= 2;

	//Reset DMA buffer index
	g_nextRxFrame = 0;
}

bool TestEthernet(uint32_t num_frames)
{
	g_log("Testing %d frames\n", num_frames);
	LogIndenter li(g_log);

	const int timeout = 5;

	for(uint32_t i=0; i<num_frames; i++)
	{
		//Ask for the frame
		*g_spiCS = 0;
		g_spi->BlockingWrite(REG_SEND_TEST);
		g_spi->WaitForWrites();
		*g_spiCS = 1;

		//Get current time
		auto tim = g_logTimer->GetCount();

		while(true)
		{
			auto& desc = rx_dma_descriptor[g_nextRxFrame];
			if( (desc.RDES0 & 0x80000000) == 0)
			{
				g_log("Got frame %d\n", i);

				int len = (desc.RDES0 >> 16) & 0x3fff;
				if(len != 68)
				{
					g_log(Logger::ERROR, "Bad length on frame %d (%d bytes, expected 68)\n", i, len);
					return false;
				}

				if(desc.RDES0 & 0x2)
				{
					g_log(Logger::ERROR, "Bad CRC on frame %d (in DMA header)\n", i);
					g_log(Logger::ERROR, "Frame content: ");
					for(int i=0; i<len; i++)
						g_cliUART->Printf("%02x ", buffers[g_nextRxFrame][i]);
					g_cliUART->Printf("\n");
					return false;
				}

				//Done processing the frame, return it to the MAC
				desc.RDES0 |= 0x80000000;
				EDMA.DMARPDR = 0;

				if(g_nextRxFrame == 3)
					g_nextRxFrame = 0;
				else
					g_nextRxFrame ++;
				break;
			}

			//If we see a CRC error in the counters but the frame didn't get DMA'd, it's still a fail
			if(EMAC.MMCRFCECR != 0)
			{
				g_log(Logger::ERROR, "Bad CRC on frame %d (reported by counters)\n", i);
				return false;
			}

			//Time out if it's been too long
			auto delta = g_logTimer->GetCount() - tim;
			if(delta >= timeout)
			{
				g_log(Logger::ERROR, "Timed out after %d ms waiting for frame %d\n", timeout, i);
				g_log("MACDBGR   = %08x\n", EMAC.MACDBGR);
				g_log("MMCRFCECR = %08x\n", EMAC.MMCRFCECR);
				g_log("DMASR     = %08x\n", EDMA.DMASR);
				for(int j=0; j<4; j++)
					g_log("RDES0[%d]  = %08x\n", j, rx_dma_descriptor[j].RDES0);
				return false;
			}
		}
	}

	return true;
}
