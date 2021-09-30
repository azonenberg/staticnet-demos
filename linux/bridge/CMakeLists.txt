include_directories(../../staticnet/)
include_directories(.)

###############################################################################
#C++ compilation
add_executable(bridge
	../../staticnet/crypt/CryptoEngine.cpp

	../../staticnet/drivers/base/EthernetInterface.cpp
	../../staticnet/drivers/tap/TapEthernetInterface.cpp

	../../staticnet/net/arp/ARPCache.cpp
	../../staticnet/net/arp/ARPPacket.cpp
	../../staticnet/net/arp/ARPProtocol.cpp
	../../staticnet/net/ethernet/EthernetFrame.cpp
	../../staticnet/net/ethernet/EthernetProtocol.cpp
	../../staticnet/net/icmpv4/ICMPv4Protocol.cpp
	../../staticnet/net/ipv4/IPv4Packet.cpp
	../../staticnet/net/ipv4/IPv4Protocol.cpp
	../../staticnet/net/tcp/TCPSegment.cpp
	../../staticnet/net/tcp/TCPProtocol.cpp

	../../staticnet/ssh/SSHKexEcdhReplyPacket.cpp
	../../staticnet/ssh/SSHKexInitPacket.cpp
	../../staticnet/ssh/SSHTransportPacket.cpp
	../../staticnet/ssh/SSHTransportServer.cpp

	../../staticnet/contrib/tweetnacl_25519.cpp

	main.cpp
	BridgeCryptoEngine.cpp
	BridgePasswordAuthenticator.cpp
	BridgeSSHTransportServer.cpp
	BridgeTCPProtocol.cpp
)

###############################################################################
#Linker settings
target_link_libraries(bridge
	cryptopp
	)

