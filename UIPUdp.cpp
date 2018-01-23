/*
 UIPUdp.cpp - Arduino implementation of a uIP wrapper class.
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

#include "UIPEthernet.h"
#include "UIPUdp.h"
#include "Dns.h"
#include "utility/logging.h"

extern "C" {
#include "utility/uipopt.h"
#include "utility/uip.h"
#include "utility/uip_arp.h"
}

#if UIP_UDP
#define UIP_ARPHDRSIZE 42
#define UDPBUF ((struct uip_udpip_hdr *)&uip_buf[UIP_LLH_LEN])

// Constructor
UIPUDP::UIPUDP(void) :
    _uip_udp_conn(NULL)
{
  memset(&appdata,0,sizeof(appdata));
}

// initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
uint8_t
UIPUDP::begin(uint16_t port)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::begin(uint16_t port) DEBUG_V3:Function started"));
  #endif
  if (!_uip_udp_conn)
    {
      _uip_udp_conn = uip_udp_new(NULL, 0);
    }
  if (_uip_udp_conn)
    {
      uip_udp_bind(_uip_udp_conn,htons(port));
      _uip_udp_conn->appstate = &appdata;
      return 1;
    }
  return 0;
}

// Finish with the UDP socket
void
UIPUDP::stop(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::stop(void) DEBUG_V3:Function started"));
  #endif
  if (_uip_udp_conn)
    {
      uip_udp_remove(_uip_udp_conn);
      _uip_udp_conn->appstate = NULL;
      _uip_udp_conn=NULL;
      Enc28J60Network::freeBlock(appdata.packet_in);
      Enc28J60Network::freeBlock(appdata.packet_next);
      Enc28J60Network::freeBlock(appdata.packet_out);
      memset(&appdata,0,sizeof(appdata));
    }
}

// Sending UDP packets

// Start building up a packet to send to the remote host specific in ip and port
// Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
int
UIPUDP::beginPacket(IPAddress ip, uint16_t port)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::beginPacket(IPAddress ip, uint16_t port) DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  if (ip && port)
    {
      uip_ipaddr_t ripaddr;
      uip_ip_addr(&ripaddr, ip);
#if ACTLOGLEVEL>=LOG_DEBUG
      LogObject.uart_send_str(F("UIPUDP::beginPacket DEBUG:udp beginPacket, "));
#endif
      if (_uip_udp_conn)
        {
          _uip_udp_conn->rport = htons(port);
          uip_ipaddr_copy(_uip_udp_conn->ripaddr, &ripaddr);
        }
      else
        {
          _uip_udp_conn = uip_udp_new(&ripaddr,htons(port));
          if (_uip_udp_conn)
            {
#if ACTLOGLEVEL>=LOG_DEBUG
              LogObject.uart_send_str(F("new connection, "));
#endif
              _uip_udp_conn->appstate = &appdata;
            }
          else
            {
#if ACTLOGLEVEL>=LOG_ERR
              LogObject.uart_send_strln(F("\nUIPUDP::beginPacket ERROR:failed to allocate new connection"));
#endif
              return 0;
            }
        }
#if ACTLOGLEVEL>=LOG_DEBUG
          LogObject.uart_send_str(F("rip: "));
          #if defined(ARDUINO)
            LogObject.print(ip);
          #endif
          #if defined(__MBED__)
            LogObject.printf("%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]);
          #endif
          LogObject.uart_send_str(F(", port: "));
          LogObject.uart_send_decln(port);
#endif
    }
  if (_uip_udp_conn)
    {
      if (appdata.packet_out == NOBLOCK)
        {
          appdata.packet_out = Enc28J60Network::allocBlock(UIP_UDP_MAXPACKETSIZE);
          appdata.out_pos = UIP_UDP_PHYH_LEN;
          if (appdata.packet_out != NOBLOCK)
            return 1;
#if ACTLOGLEVEL>=LOG_ERR
          else
            LogObject.uart_send_strln(F("\nUIPUDP::beginPacket ERROR:Failed to allocate memory for packet"));
#endif
        }
#if ACTLOGLEVEL>=LOG_WARNING
      else
        LogObject.uart_send_strln(F("\nUIPUDP::beginPacket WARNING:Previous packet on that connection not sent yet"));
#endif
    }
  return 0;
}

// Start building up a packet to send to the remote host specific in host and port
// Returns 1 if successful, 0 if there was a problem resolving the hostname or port
int
UIPUDP::beginPacket(const char *host, uint16_t port)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::beginPacket(const char *host, uint16_t port) DEBUG_V3:Function started"));
  #endif
  // Look up the host first
  int ret = 0;
  DNSClient dns;
  IPAddress remote_addr;

  dns.begin(UIPEthernet.dnsServerIP());
  ret = dns.getHostByName(host, remote_addr);
  if (ret == 1) {
    return beginPacket(remote_addr, port);
  } else {
    return ret;
  }
}

// Finish off this packet and send it
// Returns 1 if the packet was sent successfully, 0 if there was an error
int
UIPUDP::endPacket(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::endPacket(void) DEBUG_V3:Function started"));
  #endif
  if (_uip_udp_conn && appdata.packet_out != NOBLOCK)
    {
      appdata.send = true;
      Enc28J60Network::resizeBlock(appdata.packet_out,0,appdata.out_pos);
      uip_udp_periodic_conn(_uip_udp_conn);
      if (uip_len > 0)
        {
    	  _send(&appdata);
          return 1;
        }
    }
  return 0;
}

// Write a single byte into the packet
size_t
UIPUDP::write(uint8_t c)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::write(uint8_t c) DEBUG_V3:Function started"));
  #endif
  return write(&c,1);
}

// Write size bytes from buffer into the packet
size_t
UIPUDP::write(const uint8_t *buffer, size_t size)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::write(const uint8_t *buffer, size_t size) DEBUG_V3:Function started"));
  #endif
  if (appdata.packet_out != NOBLOCK)
    {
      size_t ret = Enc28J60Network::writePacket(appdata.packet_out,appdata.out_pos,(uint8_t*)buffer,size);
      appdata.out_pos += ret;
      return ret;
    }
  return 0;
}

// Start processing the next available incoming packet
// Returns the size of the packet in bytes, or 0 if no packets are available
int
UIPUDP::parsePacket(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::parsePacket(void) DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  if (appdata.packet_in != NOBLOCK)
    {
    #if ACTLOGLEVEL>=LOG_DEBUG
      LogObject.uart_send_str(F("UIPUDP::parsePacket(void) DEBUG:udp parsePacket freeing previous packet: "));
      LogObject.uart_send_decln(appdata.packet_in);
    #endif
    Enc28J60Network::freeBlock(appdata.packet_in);
    }

  appdata.packet_in = appdata.packet_next;
  appdata.packet_next = NOBLOCK;

#if ACTLOGLEVEL>=LOG_DEBUG
  if (appdata.packet_in != NOBLOCK)
    {
      LogObject.uart_send_str(F("UIPUDP::parsePacket(void) DEBUG:udp parsePacket received packet: "));
      LogObject.uart_send_dec(appdata.packet_in);
    }
#endif
  int size = Enc28J60Network::blockSize(appdata.packet_in);
#if ACTLOGLEVEL>=LOG_DEBUG
  if (appdata.packet_in != NOBLOCK)
    {
      LogObject.uart_send_str(F(", size: "));
      LogObject.uart_send_decln(size);
    }
#endif
  return size;
}

// Number of bytes remaining in the current packet
int
UIPUDP::available(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::available(void) DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  return Enc28J60Network::blockSize(appdata.packet_in);
}

// Read a single byte from the current packet
int
UIPUDP::read(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::read(void) DEBUG_V3:Function started"));
  #endif
  unsigned char c;
  if (read(&c,1) > 0)
    {
      return c;
    }
  return -1;
}

// Read up to len bytes from the current packet and place them into buffer
// Returns the number of bytes read, or 0 if none are available
int
UIPUDP::read(unsigned char* buffer, size_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::read(unsigned char* buffer, size_t len) DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  if (appdata.packet_in != NOBLOCK)
    {
      memaddress read = Enc28J60Network::readPacket(appdata.packet_in,0,(uint8_t*)buffer,(uint16_t)len);
      if (read == Enc28J60Network::blockSize(appdata.packet_in))
        {
          Enc28J60Network::freeBlock(appdata.packet_in);
          appdata.packet_in = NOBLOCK;
        }
      else
        Enc28J60Network::resizeBlock(appdata.packet_in,read);
      return read;
    }
  return 0;
}

// Return the next byte from the current packet without moving on to the next byte
int
UIPUDP::peek(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::peek(void) DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  if (appdata.packet_in != NOBLOCK)
    {
      unsigned char c;
      if (Enc28J60Network::readPacket(appdata.packet_in,0,(uint8_t*)&c,1) == 1)
        return c;
    }
  return -1;
}

// Finish reading the current packet
void
UIPUDP::flush(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::flush(void) DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  Enc28J60Network::freeBlock(appdata.packet_in);
  appdata.packet_in = NOBLOCK;
}

// Return the IP address of the host who sent the current incoming packet
IPAddress
UIPUDP::remoteIP(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::remoteIP(void) DEBUG_V3:Function started"));
  #endif
  return _uip_udp_conn ? ip_addr_uip(_uip_udp_conn->ripaddr) : IPAddress();
}

// Return the port of the host who sent the current incoming packet
uint16_t
UIPUDP::remotePort(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::remotePort(void) DEBUG_V3:Function started"));
  #endif
  return _uip_udp_conn ? ntohs(_uip_udp_conn->rport) : 0;
}

// uIP callback function

void
uipudp_appcall(void) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("uipudp_appcall(void) DEBUG_V3:Function started"));
  #endif
  if (uip_udp_userdata_t *data = (uip_udp_userdata_t *)(uip_udp_conn->appstate))
    {
      if (uip_newdata())
        {
          if (data->packet_next == NOBLOCK)
            {
              uip_udp_conn->rport = UDPBUF->srcport;
              uip_ipaddr_copy(uip_udp_conn->ripaddr,UDPBUF->srcipaddr);
              data->packet_next = Enc28J60Network::allocBlock(ntohs(UDPBUF->udplen)-UIP_UDPH_LEN);
                  //if we are unable to allocate memory the packet is dropped. udp doesn't guarantee packet delivery
              if (data->packet_next != NOBLOCK)
                {
                  //discard Linklevel and IP and udp-header and any trailing bytes:
                  Enc28J60Network::copyPacket(data->packet_next,0,UIPEthernetClass::in_packet,UIP_UDP_PHYH_LEN,Enc28J60Network::blockSize(data->packet_next));
    #if ACTLOGLEVEL>=LOG_DEBUG
                  LogObject.uart_send_str(F("uipudp_appcall(void) DEBUG:udp, uip_newdata received packet: "));
                  LogObject.uart_send_dec(data->packet_next);
                  LogObject.uart_send_str(F(", size: "));
                  LogObject.uart_send_decln(Enc28J60Network::blockSize(data->packet_next));
    #endif
                }
            }
        }
      if (uip_poll() && data->send)
        {
          //set uip_slen (uip private) by calling uip_udp_send
#if ACTLOGLEVEL>=LOG_DEBUG
          LogObject.uart_send_str(F("uipudp_appcall(void) DEBUG:udp, uip_poll preparing packet to send: "));
          LogObject.uart_send_dec(data->packet_out);
          LogObject.uart_send_str(F(", size: "));
          LogObject.uart_send_decln(Enc28J60Network::blockSize(data->packet_out));
#endif
          UIPEthernetClass::uip_packet = data->packet_out;
          UIPEthernetClass::uip_hdrlen = UIP_UDP_PHYH_LEN;
          uip_udp_send(data->out_pos - (UIP_UDP_PHYH_LEN));
        }
    }
}

void
UIPUDP::_send(uip_udp_userdata_t *data) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPUDP::_send(uip_udp_userdata_t *data) DEBUG_V3:Function started"));
  #endif
  uip_arp_out(); //add arp
  if (uip_len == UIP_ARPHDRSIZE)
    {
      UIPEthernetClass::uip_packet = NOBLOCK;
      UIPEthernetClass::packetstate &= ~UIPETHERNET_SENDPACKET;
#if ACTLOGLEVEL>=LOG_DEBUG
      LogObject.uart_send_strln(F("UIPUDP::_send() DEBUG:udp, uip_poll results in ARP-packet"));
#endif
    }
  else
  //arp found ethaddr for ip (otherwise packet is replaced by arp-request)
    {
      data->send = false;
      data->packet_out = NOBLOCK;
      UIPEthernetClass::packetstate |= UIPETHERNET_SENDPACKET;
#if ACTLOGLEVEL>=LOG_DEBUG
      LogObject.uart_send_str(F("UIPUDP::_send() DEBUG:udp, uip_packet to send: "));
      LogObject.uart_send_decln(UIPEthernetClass::uip_packet);
#endif
    }
  UIPEthernetClass::network_send();
}
#endif
