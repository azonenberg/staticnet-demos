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
#include "DemoCLISessionContext.h"

char g_hostname[33] = "stm32";

//List of all valid commands
enum cmdid_t
{
	CMD_ADDRESS,
	CMD_BAR,
	CMD_BAZ,
	CMD_EXIT,
	CMD_FOO,
	CMD_HARDWARE,
	CMD_HOSTNAME,
	CMD_IP,
	CMD_SHOW
};

static const clikeyword_t g_hostnameCommands[] =
{
	{"<string>",	FREEFORM_TOKEN,		NULL,	"New host name"},
	{NULL,			INVALID_COMMAND,	NULL,	NULL}
};

static const clikeyword_t g_ipCommands[] =
{
	{"address",		CMD_ADDRESS,		NULL,	"ip help"},

	{NULL,			INVALID_COMMAND,	NULL,	NULL}
};

static const clikeyword_t g_showCommands[] =
{
	{"bar",			CMD_BAR,			NULL,	"Look at bar"},
	{"baz",			CMD_BAZ,			NULL,	"Look at baz"},
	{"foo",			CMD_FOO,			NULL,	"Look at foo"},
	{"hardware",	CMD_HARDWARE,		NULL,	"Look at hardware"},

	{NULL,			INVALID_COMMAND,	NULL,	NULL}
};

//The top level command list
static const clikeyword_t g_rootCommands[] =
{
	{"exit",		CMD_EXIT,			NULL,					"Log out"},
	{"hostname",	CMD_HOSTNAME,		g_hostnameCommands,		"Change the host name"},
	{"ip",			CMD_IP,				g_ipCommands,			"Configure IP addresses"},
	{"show",		CMD_SHOW,			g_showCommands,			"Print information"},

	{NULL,			INVALID_COMMAND,	NULL,	NULL}
};

DemoCLISessionContext::DemoCLISessionContext()
	: CLISessionContext(g_rootCommands)
	, m_stream(NULL)
{
}

void DemoCLISessionContext::PrintPrompt()
{
	m_stream->Printf("%s@%s$ ", m_username, g_hostname);
	m_stream->Flush();
}

void DemoCLISessionContext::OnExecute()
{
	switch(m_command[0].m_commandID)
	{
		case CMD_EXIT:
			m_stream->Flush();
			m_stream->Disconnect();
			break;

		case CMD_HOSTNAME:
			strncpy(g_hostname, m_command[1].m_text, sizeof(g_hostname)-1);
			break;

		case CMD_IP:
			m_stream->Printf("set ip\n");
			break;

		case CMD_SHOW:
			switch(m_command[1].m_commandID)
			{
				case CMD_HARDWARE:
					ShowHardware();
					break;

				case CMD_FOO:
					m_stream->Printf("showing foo\n");
					break;

				case CMD_BAR:
					m_stream->Printf("showing bar\n");
					break;

				case CMD_BAZ:
					m_stream->Printf("showing baz\n");
					break;
			}
			break;

		default:
			break;
	}

	m_stream->Flush();
}

void DemoCLISessionContext::ShowHardware()
{
	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	m_stream->Printf("CPU:\n");
	if(device == 0x451)
	{
		//Look up the stepping number
		const char* srev = NULL;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
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
				m_stream->Printf("    STM32F767 / 777 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 6:
				m_stream->Printf("    STM32F769 / 779 LQFP208/TFBGA216 rev %s (0x%04x)\n", srev, rev);
				break;
			case 5:
				m_stream->Printf("    STM32F767 / 777 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 4:
				m_stream->Printf("    STM32F769 / 779 LQFP176 rev %s (0x%04x)\n", srev, rev);
				break;
			case 3:
				m_stream->Printf("    STM32F778 / 779 WLCSP180 rev %s (0x%04x)\n", srev, rev);
				break;
			case 2:
				m_stream->Printf("    STM32F767 / 777 LQFP144 rev %s (0x%04x)\n", srev, rev);
				break;
			case 1:
				m_stream->Printf("    STM32F767 / 777 LQFP100 rev %s (0x%04x)\n", srev, rev);
				break;
			default:
				m_stream->Printf("    Unknown/reserved STM32F76x/F77x rev %s (0x%04x)\n", srev, rev);
				break;
		}
		m_stream->Printf("    512 kB total SRAM, 128 kB DTCM, 16 kB ITCM, 4 kB backup SRAM\n");
		m_stream->Printf("    %d kB Flash\n", F_ID);

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
		m_stream->Printf("    Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);

		if(g_hasRmiiErrata)
			m_stream->Printf("    RMII RXD0 errata present\n");
	}
	else
		m_stream->Printf("Unknown device (0x%06x)\n", device);

	m_stream->Printf("Our MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n",
		g_macAddress[0], g_macAddress[1], g_macAddress[2], g_macAddress[3], g_macAddress[4], g_macAddress[5]);
}
