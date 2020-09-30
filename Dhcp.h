// DHCP Library v0.3 - April 25, 2009
// Author: Jordan Terrell - blog.jordanterrell.com

#ifndef Dhcp_h
#define Dhcp_h

#include "utility/uipopt.h"

/* DHCP state machine. */
#define STATE_DHCP_START 0
#define	STATE_DHCP_DISCOVER	1
#define	STATE_DHCP_REQUEST	2
#define	STATE_DHCP_LEASED	3
#define	STATE_DHCP_REREQUEST	4
#define	STATE_DHCP_RELEASE	5

#define DHCP_FLAGSBROADCAST	0x8000

/* UDP port numbers for DHCP */
#define	DHCP_SERVER_PORT	67	/* from server to client */
#define DHCP_CLIENT_PORT	68	/* from client to server */

/* DHCP message OP code */
#define DHCP_BOOTREQUEST	1
#define DHCP_BOOTREPLY		2

/* DHCP message type */
#define	DHCP_DISCOVER		1
#define DHCP_OFFER		  2
#define	DHCP_REQUEST		3
#define	DHCP_DECLINE		4
#define	DHCP_ACK		    5
#define DHCP_NAK		    6
#define	DHCP_RELEASE		7
#define DHCP_INFORM		  8

#define DHCP_HTYPE10MB		1
#define DHCP_HTYPE100MB		2

#define DHCP_HLENETHERNET	6
#define DHCP_HOPS		0
#define DHCP_SECS		0

#define MAGIC_COOKIE		0x63825363
#define MAX_DHCP_OPT	16

#define HOST_NAME "ENC28J"
#define DEFAULT_LEASE	(900) //default lease time in seconds
#define DHCP_TIMEOUT            60000
#define DHCP_RESPONSE_TIMEOUT   4000

#define DHCP_CHECK_NONE         (0)
#define DHCP_CHECK_RENEW_FAIL   (1)
#define DHCP_CHECK_RENEW_OK     (2)
#define DHCP_CHECK_REBIND_FAIL  (3)
#define DHCP_CHECK_REBIND_OK    (4)

#if UIP_UDP
#include "UIPUdp.h"

enum
{
	padOption		=	0,
	subnetMask		=	1,
	timerOffset		=	2,
	routersOnSubnet		=	3,
	/* timeServer		=	4,
	nameServer		=	5,*/
	dns			=	6,
	/*logServer		=	7,
	cookieServer		=	8,
	lprServer		=	9,
	impressServer		=	10,
	resourceLocationServer	=	11,*/
	hostName		=	12,
	/*bootFileSize		=	13,
	meritDumpFile		=	14,*/
	domainName		=	15,
	/*swapServer		=	16,
	rootPath		=	17,
	extentionsPath		=	18,
	IPforwarding		=	19,
	nonLocalSourceRouting	=	20,
	policyFilter		=	21,
	maxDgramReasmSize	=	22,
	defaultIPTTL		=	23,
	pathMTUagingTimeout	=	24,
	pathMTUplateauTable	=	25,
	ifMTU			=	26,
	allSubnetsLocal		=	27,
	broadcastAddr		=	28,
	performMaskDiscovery	=	29,
	maskSupplier		=	30,
	performRouterDiscovery	=	31,
	routerSolicitationAddr	=	32,
	staticRoute		=	33,
	trailerEncapsulation	=	34,
	arpCacheTimeout		=	35,
	ethernetEncapsulation	=	36,
	tcpDefaultTTL		=	37,
	tcpKeepaliveInterval	=	38,
	tcpKeepaliveGarbage	=	39,
	nisDomainName		=	40,
	nisServers		=	41,*/
	ntpServers		=	42,
	/*vendorSpecificInfo	=	43,
	netBIOSnameServer	=	44,
	netBIOSdgramDistServer	=	45,
	netBIOSnodeType		=	46,
	netBIOSscope		=	47,
	xFontServer		=	48,
	xDisplayManager		=	49,*/
	dhcpRequestedIPaddr	=	50,
	dhcpIPaddrLeaseTime	=	51,
	/*dhcpOptionOverload	=	52,*/
	dhcpMessageType		=	53,
	dhcpServerIdentifier	=	54,
	dhcpParamRequest	=	55,
	/*dhcpMsg			=	56,
	dhcpMaxMsgSize		=	57,*/
	dhcpT1value		=	58,
	dhcpT2value		=	59,
	/*dhcpClassIdentifier	=	60,*/
	dhcpClientIdentifier	=	61,
	endOption		=	255
};

typedef struct _RIP_MSG_FIXED
{
	uint8_t  op; 
	uint8_t  htype; 
	uint8_t  hlen;
	uint8_t  hops;
	uint32_t xid;
	uint16_t secs;
	uint16_t flags;
	uint8_t  ciaddr[4];
	uint8_t  yiaddr[4];
	uint8_t  siaddr[4];
	uint8_t  giaddr[4];
	uint8_t  chaddr[6];
}RIP_MSG_FIXED;

typedef struct
{
  uint8_t  LocalIp[4];
  uint8_t  SubnetMask[4];
  uint8_t  GatewayIp[4];
  uint8_t  DhcpServerIp[4];
  uint8_t  DnsServerIp[4];
  uint8_t  NTPServerIp[4];
} TIPV4Struct;

class DhcpClass {
private:
  uint32_t _dhcpInitialTransactionId;
  uint32_t _dhcpTransactionId;
  uint8_t  _dhcpMacAddr[6];
  TIPV4Struct _dhcpipv4struct;
  uint32_t _dhcpLeaseTime;
  uint32_t _dhcpT1, _dhcpT2;
  signed long _renewInSec;
  signed long _rebindInSec;
  signed long _lastCheck;
  unsigned long _secTimeout;
  uint8_t _dhcp_state;
  UIPUDP _dhcpUdpSocket;
  
  int request_DHCP_lease(void);
  void reset_DHCP_lease(void);
  void presend_DHCP(void);
  void send_DHCP_MESSAGE(uint8_t, uint16_t);
  void printByte(char *, uint8_t);
  
  uint8_t parseDHCPResponse(uint32_t& transactionId);
public:
  IPAddress getLocalIp(void);
  IPAddress getSubnetMask(void);
  IPAddress getGatewayIp(void);
  IPAddress getDhcpServerIp(void);
  IPAddress getDnsServerIp(void);
  IPAddress getNTPServerIp(void);  
  
  int beginWithDHCP(uint8_t *);
  int checkLease(void);
};
#endif
#endif
