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
#include "DemoTCPProtocol.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

DemoTCPProtocol::DemoTCPProtocol(IPv4Protocol* ipv4)
	: TCPProtocol(ipv4)
	, m_server(*this)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Message handlers

bool DemoTCPProtocol::IsPortOpen(uint16_t port)
{
	return (port == 22);
}

void DemoTCPProtocol::OnConnectionAccepted(TCPTableEntry* state)
{
	//Tell the SSH server process to do its thing
	m_server.OnConnectionAccepted(state);
}

void DemoTCPProtocol::OnConnectionClosed(TCPTableEntry* state)
{
	m_server.OnConnectionClosed(state);
}

bool DemoTCPProtocol::OnRxData(TCPTableEntry* state, uint8_t* payload, uint16_t payloadLen)
{
	//Discard anything not to port 22
	if(state->m_localPort != 22)
		return true;

	//Pass the incoming traffic off to the SSH server process
	return m_server.OnRxData(state, payload, payloadLen);
}

uint32_t DemoTCPProtocol::GenerateInitialSequenceNumber()
{
	uint32_t ret = 0;
	m_crypt.GenerateRandom(reinterpret_cast<uint8_t*>(&ret), sizeof(ret));
	return ret;
}
