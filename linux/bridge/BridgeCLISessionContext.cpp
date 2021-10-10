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

#include "bridge.h"
#include "BridgeCLISessionContext.h"

//List of all valid commands
enum cmdid_t
{
	CMD_EXIT,
	CMD_HOSTNAME,
	CMD_SHOW,
	CMD_IP,
	CMD_ADDRESS,
	CMD_FOO,
	CMD_BAR,
	CMD_BAZ
};


static const clikeyword_t g_hostnameCommands[] =
{
	{"",			FREEFORM_TOKEN,		NULL,	"New host name"},
	{NULL,			INVALID_COMMAND,	NULL,	NULL}
};

static const clikeyword_t g_ipCommands[] =
{
	{"address",		CMD_ADDRESS,		NULL,	"ip help"},

	{NULL,			INVALID_COMMAND,	NULL,	NULL}
};

static const clikeyword_t g_showCommands[] =
{
	{"foo",			CMD_FOO,			NULL,	"Look at foo"},
	{"bar",			CMD_BAR,			NULL,	"Look at bar"},
	{"baz",			CMD_BAZ,			NULL,	"Look at baz"},

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

BridgeCLISessionContext::BridgeCLISessionContext()
	: CLISessionContext(g_rootCommands)
{
}

void BridgeCLISessionContext::PrintPrompt()
{
	m_stream.PutString(m_username);
	m_stream.PutString("@demo$ ");
	m_stream.Flush();
}

void BridgeCLISessionContext::OnExecute()
{
	switch(m_command[0].m_commandID)
	{
		case CMD_EXIT:
			m_stream.PutString("\n");
			m_stream.Flush();
			m_stream.GetServer()->GracefulDisconnect(m_stream.GetSessionID(), m_stream.GetSocket());
			break;

		case CMD_HOSTNAME:

		default:
			break;
	}
}
