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

//Set private key to a hard coded constant (from testdata/id_ed25519)
uint8_t g_hostkeyPriv[32] =
{
	0xb2, 0xc8, 0x0c, 0x44, 0xb1, 0xad, 0x19, 0xb5,
	0x7a, 0x66, 0x5e, 0xa1, 0x7c, 0x78, 0x8b, 0x7b,
	0x4d, 0x20, 0xbf, 0x19, 0x49, 0x85, 0x97, 0x9e,
	0xf2, 0x79, 0x3e, 0xdc, 0x83, 0xf4, 0xd1, 0xa7
};

uint8_t g_hostkeyPub[32] =
{
	0xf7, 0x45, 0xd2, 0x13, 0x13, 0x4b, 0x19, 0x97,
	0xcf, 0xcf, 0x86, 0x98, 0xcc, 0x2b, 0x0c, 0xd2,
	0xc0, 0x45, 0xb1, 0xc9, 0xd4, 0xba, 0x22, 0x9f,
	0x08, 0x8c, 0x66, 0x90, 0xf2, 0x4b, 0xf4, 0xbf
};

int main(int /*argc*/, char* /*argv*/[])
{
	//Bring up the tap interface
	TapEthernetInterface iface("simtap");

	//Address configuration
	MACAddress mac = {{ 0x02, 0xde, 0xad, 0xbe, 0xef, 0x41 }};
	IPv4Config ipconfig;
	ipconfig.m_address		= { .m_octets{192, 168,   1,   2} };
	ipconfig.m_netmask		= { .m_octets{255, 255, 255,   0} };
	ipconfig.m_broadcast	= { .m_octets{192, 168,   1, 255} };
	ipconfig.m_gateway		= { .m_octets{192, 168,   1,   1} };

	//ARP cache (shared by all interfaces)
	ARPCache cache;

	//Per-interface protocol stacks
	EthernetProtocol eth(iface, mac);
	ARPProtocol arp(eth, ipconfig.m_address, cache);

	//Global protocol stacks
	IPv4Protocol ipv4(eth, ipconfig, cache);
	ICMPv4Protocol icmpv4(ipv4);
	BridgeTCPProtocol tcp(&ipv4);

	//Register protocol handlers with the lower layer
	eth.UseARP(&arp);
	eth.UseIPv4(&ipv4);
	ipv4.UseICMPv4(&icmpv4);
	ipv4.UseTCP(&tcp);

	//Set up SSH host keys
	CryptoEngine::SetHostKey(g_hostkeyPub, g_hostkeyPriv);

	//Main event handling loop
	while(true)
	{
		auto frame = iface.GetRxFrame();
		if(frame)
			eth.OnRxFrame(frame);

		else
			usleep(1000);
	}

	return 0;
}
