/*
 UIPEthernet.cpp - Arduino implementation of a uIP wrapper class.
 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
  */

#if defined(ARDUINO)
  #include <Arduino.h>
#endif
#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/millis.h"
#endif
#include "UIPEthernet.h"
#include "utility/logging.h"
#include "utility/Enc28J60Network.h"

#include "UIPUdp.h"

extern "C"
{
#include "utility/uipopt.h"
#include "utility/uip.h"
#include "utility/uip_arp.h"
#include "utility/uip_timer.h"
}

#define ETH_HDR ((struct uip_eth_hdr *)&uip_buf[0])

memhandle UIPEthernetClass::in_packet(NOBLOCK);
memhandle UIPEthernetClass::uip_packet(NOBLOCK);
uint8_t UIPEthernetClass::uip_hdrlen(0);
uint8_t UIPEthernetClass::packetstate(0);

unsigned long UIPEthernetClass::periodic_timer;

IPAddress UIPEthernetClass::_dnsServerAddress;
#if UIP_UDP
  DhcpClass* UIPEthernetClass::_dhcp(NULL);
  static DhcpClass s_dhcp; // Placing this instance here is saving 40K to final *.bin (see bug below)
#endif

// Because uIP isn't encapsulated within a class we have to use global
// variables, so we can only have one TCP/IP stack per program.

UIPEthernetClass::UIPEthernetClass()
{
}

void UIPEthernetClass::init(const uint8_t pin)
{
  ENC28J60ControlCS = pin;
}

#if UIP_UDP
int
UIPEthernetClass::begin(const uint8_t* mac)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::begin(const uint8_t* mac) DEBUG_V3:Function started"));
  #endif
  //static DhcpClass s_dhcp; // <-- this is a bug !
  // I leave it there commented for history. It is bring all GCC "new" memory allocation code, making the *.bin almost 40K bigger. I've move it globally.
  _dhcp = &s_dhcp;
  // Initialise the basic info
  netInit(mac);

  // Now try to get our config info from a DHCP server
  int ret = _dhcp->beginWithDHCP((uint8_t*)mac);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    configure(_dhcp->getLocalIp(),_dhcp->getDnsServerIp(),_dhcp->getGatewayIp(),_dhcp->getSubnetMask());
  }
  return ret;
}
#endif

void
UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip) DEBUG_V3:Function started"));
  #endif
  IPAddress dns = ip;
  dns[3] = 1;
  begin(mac, ip, dns);
}

void
UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip, IPAddress dns)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip, IPAddress dns) DEBUG_V3:Function started"));
  #endif
  IPAddress gateway = ip;
  gateway[3] = 1;
  begin(mac, ip, dns, gateway);
}

void
UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip, IPAddress dns, IPAddress gateway)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip, IPAddress dns, IPAddress gateway) DEBUG_V3:Function started"));
  #endif
  IPAddress subnet(255, 255, 255, 0);
  begin(mac, ip, dns, gateway, subnet);
}

void
UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::begin(const uint8_t* mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet) DEBUG_V3:Function started"));
  #endif
  netInit(mac);
  configure(ip,dns,gateway,subnet);
}

int UIPEthernetClass::maintain(){
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::maintain() DEBUG_V3:Function started"));
  #endif
  tick();
  int rc = DHCP_CHECK_NONE;
#if UIP_UDP
  if(_dhcp != NULL){
    //we have a pointer to dhcp, use it
    rc = _dhcp->checkLease();
    switch ( rc ){
      case DHCP_CHECK_NONE:
        //nothing done
        break;
      case DHCP_CHECK_RENEW_OK:
      case DHCP_CHECK_REBIND_OK:
        //we might have got a new IP.
        configure(_dhcp->getLocalIp(),_dhcp->getDnsServerIp(),_dhcp->getGatewayIp(),_dhcp->getSubnetMask());
        break;
      default:
        //this is actually a error, it will retry though
        break;
    }
  }
  return rc;
#endif
}

EthernetLinkStatus UIPEthernetClass::linkStatus()
{
  if (!Enc28J60.geterevid())
    return Unknown;
  return Enc28J60.linkStatus() ? LinkON : LinkOFF;
}

IPAddress UIPEthernetClass::localIP()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::localIP() DEBUG_V3:Function started"));
  #endif
  IPAddress ret;
  uip_ipaddr_t a;
  uip_gethostaddr(a);
  return ip_addr_uip(a);
}

IPAddress UIPEthernetClass::subnetMask()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::subnetMask() DEBUG_V3:Function started"));
  #endif
  IPAddress ret;
  uip_ipaddr_t a;
  uip_getnetmask(a);
  return ip_addr_uip(a);
}

IPAddress UIPEthernetClass::gatewayIP()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::gatewayIP() DEBUG_V3:Function started"));
  #endif
  IPAddress ret;
  uip_ipaddr_t a;
  uip_getdraddr(a);
  return ip_addr_uip(a);
}

IPAddress UIPEthernetClass::dnsServerIP()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::dnsServerIP() DEBUG_V3:Function started"));
  #endif
  return _dnsServerAddress;
}

void
UIPEthernetClass::tick()
{
#if ACTLOGLEVEL>=LOG_DEBUG_V3
  LogObject.uart_send_strln(F("UIPEthernetClass::tick() DEBUG_V3:Function started"));
#endif
if (Enc28J60Network::geterevid()==0)
   {
   #if ACTLOGLEVEL>=LOG_ERR
     LogObject.uart_send_strln(F("UIPEthernetClass::tick() ERROR:EREVID=0 -> Not found ENC28j60 device !! Function ended !!"));
   #endif
   return;
   }
#if defined(ESP8266)
  wdt_reset();
#endif
  if (in_packet == NOBLOCK)
    {
    in_packet = Enc28J60Network::receivePacket();
    #if ACTLOGLEVEL>=LOG_DEBUG
    if (in_packet != NOBLOCK)
      {
      LogObject.uart_send_str(F("UIPEthernetClass::tick() DEBUG:receivePacket: "));
      LogObject.uart_send_decln(in_packet);
      }
    #endif
    }
  if (in_packet != NOBLOCK)
    {
    packetstate = UIPETHERNET_FREEPACKET;
    uip_len = Enc28J60Network::blockSize(in_packet);
    if (uip_len > 0)
      {
      Enc28J60Network::readPacket(in_packet,0,(uint8_t*)uip_buf,UIP_BUFSIZE);
      if (ETH_HDR ->type == HTONS(UIP_ETHTYPE_IP))
        {
        uip_packet = in_packet; //required for upper_layer_checksum of in_packet!
        #if ACTLOGLEVEL>=LOG_DEBUG
          LogObject.uart_send_str(F("UIPEthernetClass::tick() DEBUG:readPacket type IP, uip_len: "));
          LogObject.uart_send_decln(uip_len);
        #endif
        uip_arp_ipin();
        uip_input();
        if (uip_len > 0)
          {
          uip_arp_out();
          network_send();
          }
        }
      else if (ETH_HDR ->type == HTONS(UIP_ETHTYPE_ARP))
             {
             #if ACTLOGLEVEL>=LOG_DEBUG
               LogObject.uart_send_str(F("UIPEthernetClass::tick() DEBUG:readPacket type ARP, uip_len: "));
               LogObject.uart_send_decln(uip_len);
             #endif
             uip_arp_arpin();
             if (uip_len > 0)
               {
               network_send();
               }
             }
      }
    if (in_packet != NOBLOCK && (packetstate & UIPETHERNET_FREEPACKET))
      {
      #if ACTLOGLEVEL>=LOG_DEBUG
        LogObject.uart_send_str(F("UIPEthernetClass::tick() DEBUG:freeing packet: "));
        LogObject.uart_send_decln(in_packet);
      #endif
      Enc28J60Network::freePacket();
      in_packet = NOBLOCK;
      }
    }

  unsigned long now = millis();

#if UIP_CLIENT_TIMER >= 0
  bool periodic = (long)( now - periodic_timer ) >= 0;
  for (int i = 0; i < UIP_CONNS; i++)
    {
#else
  if ((long)( now - periodic_timer ) >= 0)
    {
      periodic_timer = now + UIP_PERIODIC_TIMER;

      for (int i = 0; i < UIP_CONNS; i++)
        {
#endif

      uip_conn = &uip_conns[i];

#if UIP_CLIENT_TIMER >= 0
      if (periodic)
        {
#endif

          uip_process(UIP_TIMER);

#if UIP_CLIENT_TIMER >= 0
        }
      else
        {
        if (uip_conn!=NULL)
           {
           if (((uip_userdata_t*)uip_conn->appstate)!=NULL)
              {
              if ((long)( now - ((uip_userdata_t*)uip_conn->appstate)->timer) >= 0)
                 {
                 uip_process(UIP_POLL_REQUEST);
                 }
              else
                 {
                 continue;
                 }
              }
           else
              {
              #if ACTLOGLEVEL>=LOG_DEBUG_V3
                 LogObject.uart_send_strln(F("UIPEthernetClass::tick() DEBUG_V3:((uip_userdata_t*)uip_conn->appstate) is NULL"));
              #endif
              if ((long)( now - ((uip_userdata_t*)uip_conn)->timer) >= 0)
                 {
                 uip_process(UIP_POLL_REQUEST);
                 }
              else
                 {
                 continue;
                 }
              }
           }
        else
           {
           #if ACTLOGLEVEL>=LOG_ERR
             LogObject.uart_send_strln(F("UIPEthernetClass::tick() ERROR:uip_conn is NULL"));
           #endif
           continue;
           }
        }
#endif
        // If the above function invocation resulted in data that
        // should be sent out on the Enc28J60Network, the global variable
        // uip_len is set to a value > 0.
      if (uip_len > 0)
        {
          uip_arp_out();
          network_send();
        }
    }
#if UIP_CLIENT_TIMER >= 0
  if (periodic)
    {
      periodic_timer = now + UIP_PERIODIC_TIMER;
#endif
#if UIP_UDP
      for (int i = 0; i < UIP_UDP_CONNS; i++)
        {
          uip_udp_periodic(i);
          // If the above function invocation resulted in data that
          // should be sent out on the Enc28J60Network, the global variable
          // uip_len is set to a value > 0. */
          if (uip_len > 0)
            {
              UIPUDP::_send((uip_udp_userdata_t *)(uip_udp_conns[i].appstate));
            }
        }
#endif /* UIP_UDP */
    }
}

bool UIPEthernetClass::network_send()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::network_send() DEBUG_V3:Function started"));
  #endif
  if (packetstate & UIPETHERNET_SENDPACKET)
    {
#if ACTLOGLEVEL>=LOG_DEBUG
      LogObject.uart_send_str(F("UIPEthernetClass::network_send() DEBUG:uip_packet: "));
      LogObject.uart_send_dec(uip_packet);
      LogObject.uart_send_str(F(", hdrlen: "));
      LogObject.uart_send_decln(uip_hdrlen);
#endif
      Enc28J60Network::writePacket(uip_packet,0,uip_buf,uip_hdrlen);
      packetstate &= ~ UIPETHERNET_SENDPACKET;
      goto sendandfree;
    }
  uip_packet = Enc28J60Network::allocBlock(uip_len);
  if (uip_packet != NOBLOCK)
    {
#if ACTLOGLEVEL>=LOG_DEBUG
      LogObject.uart_send_str(F("UIPEthernetClass::network_send() DEBUG:uip_buf (uip_len): "));
      LogObject.uart_send_dec(uip_len);
      LogObject.uart_send_str(F(", packet: "));
      LogObject.uart_send_decln(uip_packet);
#endif
      Enc28J60Network::writePacket(uip_packet,0,uip_buf,uip_len);
      goto sendandfree;
    }
  return false;
sendandfree:
  Enc28J60Network::sendPacket(uip_packet);
  Enc28J60Network::freeBlock(uip_packet);
  uip_packet = NOBLOCK;
  return true;
}

void UIPEthernetClass::netInit(const uint8_t* mac) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::netInit(const uint8_t* mac) DEBUG_V3:Function started"));
  #endif
  periodic_timer = millis() + UIP_PERIODIC_TIMER;

  Enc28J60Network::init((uint8_t*)mac);
  uip_seteth_addr(mac);

  uip_init();
  uip_arp_init();
}

void UIPEthernetClass::configure(IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::configure(IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet) DEBUG_V3:Function started"));
  #endif
  uip_ipaddr_t ipaddr;

  uip_ip_addr(ipaddr, ip);
  uip_sethostaddr(ipaddr);

  uip_ip_addr(ipaddr, gateway);
  uip_setdraddr(ipaddr);

  uip_ip_addr(ipaddr, subnet);
  uip_setnetmask(ipaddr);

  _dnsServerAddress = dns;
}

UIPEthernetClass UIPEthernet;

/*---------------------------------------------------------------------------*/
uint16_t
UIPEthernetClass::chksum(uint16_t sum, const uint8_t *data, uint16_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::chksum(uint16_t sum, const uint8_t *data, uint16_t len) DEBUG_V3:Function started"));
  #endif
  uint16_t t;
  const uint8_t *dataptr;
  const uint8_t *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while(dataptr < last_byte) {  /* At least two more bytes */
    t = (dataptr[0] << 8) + dataptr[1];
    sum += t;
    if(sum < t) {
      sum++;            /* carry */
    }
    dataptr += 2;
  }

  if(dataptr == last_byte) {
    t = (dataptr[0] << 8) + 0;
    sum += t;
    if(sum < t) {
      sum++;            /* carry */
    }
  }

  /* Return sum in host byte order. */
  return sum;
}

/*---------------------------------------------------------------------------*/

uint16_t
UIPEthernetClass::ipchksum(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPEthernetClass::ipchksum(void) DEBUG_V3:Function started"));
  #endif
  uint16_t sum;

  sum = chksum(0, &uip_buf[UIP_LLH_LEN], UIP_IPH_LEN);
  return (sum == 0) ? 0xffff : htons(sum);
}

/*---------------------------------------------------------------------------*/
uint16_t
#if UIP_UDP
UIPEthernetClass::upper_layer_chksum(uint8_t proto)
#else
uip_tcpchksum(void)
#endif
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    #if UIP_UDP
      LogObject.uart_send_strln(F("UIPEthernetClass::upper_layer_chksum(uint8_t proto) DEBUG_V3:Function started"));
    #else
      LogObject.uart_send_strln(F("uip_tcpchksum(void) INFO:Function started"));
    #endif
  #endif
  uint16_t upper_layer_len;
  uint16_t sum;

#if UIP_CONF_IPV6
  upper_layer_len = (((u16_t)(BUF->len[0]) << 8) + BUF->len[1]);
#else /* UIP_CONF_IPV6 */
  upper_layer_len = (((u16_t)(BUF->len[0]) << 8) + BUF->len[1]) - UIP_IPH_LEN;
#endif /* UIP_CONF_IPV6 */

  /* First sum pseudoheader. */

  /* IP protocol and length fields. This addition cannot carry. */
#if UIP_UDP
  sum = upper_layer_len + proto;
#else
  sum = upper_layer_len + UIP_PROTO_TCP;
#endif
  /* Sum IP source and destination addresses. */
  sum = UIPEthernetClass::chksum(sum, (u8_t *)&BUF->srcipaddr[0], 2 * sizeof(uip_ipaddr_t));

  uint8_t upper_layer_memlen;
#if UIP_UDP
  switch(proto)
  {
//    case UIP_PROTO_ICMP:
//    case UIP_PROTO_ICMP6:
//      upper_layer_memlen = upper_layer_len;
//      break;
  case UIP_PROTO_UDP:
    upper_layer_memlen = UIP_UDPH_LEN;
    break;
  default:
//  case UIP_PROTO_TCP:
#endif
    upper_layer_memlen = (BUF->tcpoffset >> 4) << 2;
#if UIP_UDP
    break;
  }
#endif
  sum = UIPEthernetClass::chksum(sum, &uip_buf[UIP_IPH_LEN + UIP_LLH_LEN], upper_layer_memlen);
#if ACTLOGLEVEL>=LOG_DEBUG
  #if UIP_UDP
    LogObject.uart_send_str(F("UIPEthernetClass::upper_layer_chksum(uint8_t proto) DEBUG:uip_buf["));
  #else
    LogObject.uart_send_str(F("uip_tcpchksum(void) DEBUG:uip_buf["));
  #endif
  LogObject.uart_send_dec(UIP_IPH_LEN + UIP_LLH_LEN);
  LogObject.uart_send_str(F("-"));
  LogObject.uart_send_dec(UIP_IPH_LEN + UIP_LLH_LEN + upper_layer_memlen);
  LogObject.uart_send_str(F("]: "));
  LogObject.uart_send_hexln(htons(sum));
#endif
  if (upper_layer_memlen < upper_layer_len)
    {
      sum = Enc28J60Network::chksum(
          sum,
          UIPEthernetClass::uip_packet,
          UIP_IPH_LEN + UIP_LLH_LEN + upper_layer_memlen,
          upper_layer_len - upper_layer_memlen
      );
#if ACTLOGLEVEL>=LOG_DEBUG
      #if UIP_UDP
        LogObject.uart_send_str(F("UIPEthernetClass::upper_layer_chksum(uint8_t proto) DEBUG:uip_packet("));
      #else
        LogObject.uart_send_str(F("uip_tcpchksum(void) DEBUG:uip_packet("));
      #endif
      LogObject.uart_send_dec(UIPEthernetClass::uip_packet);
      LogObject.uart_send_str(F(")["));
      LogObject.uart_send_dec(UIP_IPH_LEN + UIP_LLH_LEN + upper_layer_memlen);
      LogObject.uart_send_str(F("-"));
      LogObject.uart_send_dec(UIP_IPH_LEN + UIP_LLH_LEN + upper_layer_len);
      LogObject.uart_send_str(F("]: "));
      LogObject.uart_send_hexln(htons(sum));
#endif
    }
  return (sum == 0) ? 0xffff : htons(sum);
}

uint16_t
uip_ipchksum(void)
{
  return UIPEthernet.ipchksum();
}

#if UIP_UDP
uint16_t
uip_tcpchksum(void)
{
  uint16_t sum = UIPEthernet.upper_layer_chksum(UIP_PROTO_TCP);
  return sum;
}

uint16_t
uip_udpchksum(void)
{
  uint16_t sum = UIPEthernet.upper_layer_chksum(UIP_PROTO_UDP);
  return sum;
}
#endif
