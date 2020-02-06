/*
 UIPClient.cpp - Arduino implementation of a uIP wrapper class.
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
#include "utility/logging.h"

extern "C"
{
#include "utility/uipopt.h"
#include "utility/uip.h"
#include "utility/uip_arp.h"
#include "string.h"
}
#include "UIPEthernet.h"
#include "UIPClient.h"
#if UIP_UDP
  #include "Dns.h"
#endif

#if defined(__AVR__)
   #include <avr/wdt.h>
#endif

#if defined(__MBED__)
   #include "mbed/millis.h"
#endif

#define UIP_TCP_PHYH_LEN UIP_LLH_LEN+UIP_IPTCPH_LEN

uip_userdata_t UIPClient::all_data[UIP_CONNS];

UIPClient::UIPClient() :
    data(NULL)
{
}

UIPClient::UIPClient(uip_userdata_t* conn_data) :
    data(conn_data)
{
}

int
UIPClient::connect(IPAddress ip, uint16_t port)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::connect(IPAddress ip, uint16_t port) DEBUG_V3:Function started"));
  #endif
  stop();
  uip_ipaddr_t ipaddr;
  uip_ip_addr(ipaddr, ip);
  struct uip_conn* conn = uip_connect(&ipaddr, htons(port));
  if (conn)
    {
#if UIP_CONNECT_TIMEOUT > 0
      int32_t timeout = millis() + 1000 * UIP_CONNECT_TIMEOUT;
#endif
      while((conn->tcpstateflags & UIP_TS_MASK) != UIP_CLOSED)
        {
          UIPEthernetClass::tick();
          if ((conn->tcpstateflags & UIP_TS_MASK) == UIP_ESTABLISHED)
            {
            data = (uip_userdata_t*) conn->appstate;
            #if ACTLOGLEVEL>=LOG_DEBUG
              LogObject.uart_send_str(F("UIPClient::connect DEBUG:connected, state: "));
              LogObject.uart_send_dec(data->state);
              LogObject.uart_send_str(F(", first packet in: "));
              LogObject.uart_send_decln(data->packets_in[0]);
            #endif
            return 1;
            }
#if UIP_CONNECT_TIMEOUT > 0
          if (((int32_t)(millis() - timeout)) > 0)
            {
              conn->tcpstateflags = UIP_CLOSED;
              break;
            }
#endif
        }
    }
  return 0;
}

int
UIPClient::connect(const char *host, uint16_t port)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::connect(const char *host, uint16_t port) DEBUG_V3:Function started"));
  #endif
  // Look up the host first
  int ret = 0;
#if UIP_UDP
  DNSClient dns;
  IPAddress remote_addr;

  dns.begin(UIPEthernetClass::_dnsServerAddress);
  ret = dns.getHostByName(host, remote_addr);
  if (ret == 1) {
    return connect(remote_addr, port);
  }
#endif
  return ret;
}

void
UIPClient::stop()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::stop() DEBUG_V3:Function started"));
  #endif
  if (data && data->state)
    {
    #if ACTLOGLEVEL>=LOG_DEBUG_V2
      LogObject.uart_send_strln(F("UIPClient::stop() DEBUG_V2:Before stop(), with data"));
      _dumpAllData();
    #endif
      _flushBlocks(&data->packets_in[0]);
      if (data->state & UIP_CLIENT_REMOTECLOSED)
        {
          data->state = 0;
        }
      else
        {
          data->state |= UIP_CLIENT_CLOSE;
        }
    #if ACTLOGLEVEL>=LOG_DEBUG_V2
      LogObject.uart_send_strln(F("UIPClient::stop() DEBUG_V2:after stop()"));
      _dumpAllData();
    #endif
    }
#if ACTLOGLEVEL>=LOG_DEBUG_V3
  else
    {
      LogObject.uart_send_strln(F("UIPClient::stop() DEBUG_V3:stop(), data: NULL"));
    }
#endif
  data = NULL;
  UIPEthernetClass::tick();
}

uint8_t
UIPClient::connected()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::connected() DEBUG_V3:Function started"));
  #endif
  return (data && (data->packets_in[0] != NOBLOCK || (data->state & UIP_CLIENT_CONNECTED))) ? 1 : 0;
}

bool
UIPClient::operator==(const UIPClient& rhs)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::operator==(const UIPClient& rhs) DEBUG_V3:Function started"));
  #endif
  return data && rhs.data && (data == rhs.data);
}

UIPClient::operator bool()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::operator bool() DEBUG_V3:Function started"));
  #endif
  UIPEthernetClass::tick();
  return data && (!(data->state & UIP_CLIENT_REMOTECLOSED) || data->packets_in[0] != NOBLOCK);
}

size_t
UIPClient::write(uint8_t c)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::write(uint8_t c) DEBUG_V3:Function started"));
  #endif
  return _write(data, &c, 1);
}

size_t
UIPClient::write(const uint8_t *buf, size_t size)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::write(const uint8_t *buf, size_t size) DEBUG_V3:Function started"));
  #endif
  return _write(data, buf, size);
}

uint16_t
UIPClient::_write(uip_userdata_t* u, const uint8_t *buf, size_t size)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::_write(uip_userdata_t* u, const uint8_t *buf, size_t size) DEBUG_V3:Function started"));
  #endif
  int remain = size;
  uint16_t written;
#if UIP_ATTEMPTS_ON_WRITE > 0
  uint16_t attempts = UIP_ATTEMPTS_ON_WRITE;
#endif
  repeat:
  UIPEthernetClass::tick();
  if (u && !(u->state & (UIP_CLIENT_CLOSE | UIP_CLIENT_REMOTECLOSED)))
    {
      uint8_t p = _currentBlock(&u->packets_out[0]);
      if (u->packets_out[p] == NOBLOCK)
        {
newpacket:
          u->packets_out[p] = Enc28J60Network::allocBlock(UIP_SOCKET_DATALEN);
          if (u->packets_out[p] == NOBLOCK)
            {
#if UIP_ATTEMPTS_ON_WRITE > 0
              if ((--attempts)>0)
#endif
#if UIP_ATTEMPTS_ON_WRITE != 0
                goto repeat;
#endif
              goto ready;
            }
          u->out_pos = 0;
        }
#if ACTLOGLEVEL>=LOG_DEBUG_V2
      LogObject.uart_send_str(F("UIPClient::_write DEBUG_V2:writePacket("));
      LogObject.uart_send_dec(u->packets_out[p]);
      LogObject.uart_send_str(F(") pos: "));
      LogObject.uart_send_dec(u->out_pos);
      LogObject.uart_send_str(F(", buf["));
      LogObject.uart_send_dec(size-remain);
      LogObject.uart_send_str(F("-"));
      LogObject.uart_send_dec(remain);
      LogObject.uart_send_str(F("]: '"));
      for (uint16_t i=size-remain; i<=remain; i++)
        {
        LogObject.uart_send_hex(buf[i]);
        LogObject.uart_send_str(F(" "));
        }
      LogObject.uart_send_strln(F("'"));
#endif
      written = Enc28J60Network::writePacket(u->packets_out[p],u->out_pos,(uint8_t*)buf+size-remain,remain);
      remain -= written;
      u->out_pos+=written;
      if (remain > 0)
        {
          if (p == UIP_SOCKET_NUMPACKETS-1)
            {
#if UIP_ATTEMPTS_ON_WRITE > 0
              if ((--attempts)>0)
#endif
#if UIP_ATTEMPTS_ON_WRITE != 0
                goto repeat;
#endif
              goto ready;
            }
          p++;
          goto newpacket;
        }
ready:
#if ACTLOGLEVEL>=LOG_DEBUG_V2
      LogObject.uart_send_str(F("UIPClient::_write DEBUG_V2: READY"));
#endif
#if UIP_CLIENT_TIMER >= 0
      u->timer = millis()+UIP_CLIENT_TIMER;
#endif
      return size-remain;
    }
  //return -1; -1 is wrong because return type is unsigned
  return 0;
}

int
UIPClient::available()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::available() DEBUG_V3:Function started"));
  #endif
  if (*this)
    return _available(data);
  return 0;
}

int
UIPClient::_available(uip_userdata_t *u)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::_available(uip_userdata_t *u) DEBUG_V3:Function started"));
  #endif
  int len = 0;
  for (uint8_t i = 0; i < UIP_SOCKET_NUMPACKETS; i++)
    {
      len += Enc28J60Network::blockSize(u->packets_in[i]);
    }
  return len;
}

int
UIPClient::read(uint8_t *buf, size_t size)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::read(uint8_t *buf, size_t size) DEBUG_V3:Function started"));
  #endif
  if (*this)
    {
      uint16_t remain = size;
      if (data->packets_in[0] == NOBLOCK)
        return 0;
      uint16_t read;
      do
        {
          read = Enc28J60Network::readPacket(data->packets_in[0],0,buf+size-remain,remain);
          if (read == Enc28J60Network::blockSize(data->packets_in[0]))
            {
              remain -= read;
              _eatBlock(&data->packets_in[0]);
              if (uip_stopped(&uip_conns[data->conn_index]) && !(data->state & (UIP_CLIENT_CLOSE | UIP_CLIENT_REMOTECLOSED)))
                data->state |= UIP_CLIENT_RESTART;
              if (data->packets_in[0] == NOBLOCK)
                {
                  if (data->state & UIP_CLIENT_REMOTECLOSED)
                    {
                      data->state = 0;
                      data = NULL;
                    }
                  return size-remain;
                }
            }
          else
            {
              Enc28J60Network::resizeBlock(data->packets_in[0],read);
              break;
            }
        }
      while(remain > 0);
      return size;
    }
  return -1;
}

int
UIPClient::read()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::read() DEBUG_V3:Function started"));
  #endif
  uint8_t c;
  if (read(&c,1) <= 0)
    return -1;
  return c;
}

int
UIPClient::peek()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::peek() DEBUG_V3:Function started"));
  #endif
  if (*this)
    {
      if (data->packets_in[0] != NOBLOCK)
        {
          uint8_t c;
          Enc28J60Network::readPacket(data->packets_in[0],0,&c,1);
          return c;
        }
    }
  return -1;
}

void
UIPClient::flush()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::flush() DEBUG_V3:Function started"));
  #endif
  if (*this)
    {
      _flushBlocks(&data->packets_in[0]);
    }
}

void
uipclient_appcall(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V3:Function started"));
  #endif
  uint16_t send_len = 0;
  uip_userdata_t *u = (uip_userdata_t*)uip_conn->appstate;
  if (!u && uip_connected())
    {
#if ACTLOGLEVEL>=LOG_DEBUG_V2
      LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V2:UIPClient uip_connected"));
      UIPClient::_dumpAllData();
#endif
      u = (uip_userdata_t*) UIPClient::_allocateData();
      if (u)
        {
          uip_conn->appstate = u;
#if ACTLOGLEVEL>=LOG_DEBUG_V1
          LogObject.uart_send_str(F("uipclient_appcall(void) DEBUG_V1:UIPClient allocated state: "));
          LogObject.uart_send_binln(u->state);
#endif
        }
#if ACTLOGLEVEL>=LOG_ERR
      else
        LogObject.uart_send_strln(F("uipclient_appcall(void) ERROR:UIPClient allocation failed"));
#endif
    }
  if (u)
    {
      if (uip_newdata())
        {
#if ACTLOGLEVEL>=LOG_DEBUG
          LogObject.uart_send_str(F("uipclient_appcall(void) DEBUG:UIPClient uip_newdata, uip_len:"));
          LogObject.uart_send_decln(uip_len);
#endif
          if (uip_len && !(u->state & (UIP_CLIENT_CLOSE | UIP_CLIENT_REMOTECLOSED)))
            {
              for (uint8_t i=0; i < UIP_SOCKET_NUMPACKETS; i++)
                {
                  if (u->packets_in[i] == NOBLOCK)
                    {
                      u->packets_in[i] = Enc28J60Network::allocBlock(uip_len);
                      if (u->packets_in[i] != NOBLOCK)
                        {
                          Enc28J60Network::copyPacket(u->packets_in[i],0,UIPEthernetClass::in_packet,((uint8_t*)uip_appdata)-uip_buf,uip_len);
                          if (i == UIP_SOCKET_NUMPACKETS-1)
                            uip_stop();
                          goto finish_newdata;
                        }
                    }
                }
              UIPEthernetClass::packetstate &= ~UIPETHERNET_FREEPACKET;
              uip_stop();
            }
        }
finish_newdata:
      if (u->state & UIP_CLIENT_RESTART)
        {
          u->state &= ~UIP_CLIENT_RESTART;
          uip_restart();
        }
      // If the connection has been closed, save received but unread data.
      if (uip_closed() || uip_timedout())
        {
#if ACTLOGLEVEL>=LOG_DEBUG_V2
          LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V2:UIPClient uip_closed"));
          UIPClient::_dumpAllData();
#endif
          // drop outgoing packets not sent yet:
          UIPClient::_flushBlocks(&u->packets_out[0]);
          if (u->packets_in[0] != NOBLOCK)
            {
              ((uip_userdata_closed_t *)u)->lport = uip_conn->lport;
              u->state |= UIP_CLIENT_REMOTECLOSED;
            }
          else
            u->state = 0;
          // disassociate appdata.
#if ACTLOGLEVEL>=LOG_DEBUG_V2
          LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V2:After UIPClient uip_closed"));
          UIPClient::_dumpAllData();
#endif
          uip_conn->appstate = NULL;
          goto finish;
        }
      if (uip_acked())
        {
#if ACTLOGLEVEL>=LOG_DEBUG
          LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG:UIPClient uip_acked"));
#endif
          UIPClient::_eatBlock(&u->packets_out[0]);
        }
      if (uip_poll() || uip_rexmit())
        {
#if ACTLOGLEVEL>=LOG_DEBUG_V3
          LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V3:UIPClient uip_poll || uip_rexmit"));
#endif
          if (u->packets_out[0] != NOBLOCK)
            {
              if (u->packets_out[1] == NOBLOCK)
                {
                  send_len = u->out_pos;
                  if (send_len > 0)
                    {
                      Enc28J60Network::resizeBlock(u->packets_out[0],0,send_len);
                    }
                }
              else
                send_len = Enc28J60Network::blockSize(u->packets_out[0]);
              if (send_len > 0)
                {
                  UIPEthernetClass::uip_hdrlen = ((uint8_t*)uip_appdata)-uip_buf;
                  UIPEthernetClass::uip_packet = Enc28J60Network::allocBlock(UIPEthernetClass::uip_hdrlen+send_len);
                  if (UIPEthernetClass::uip_packet != NOBLOCK)
                    {
                      Enc28J60Network::copyPacket(UIPEthernetClass::uip_packet,UIPEthernetClass::uip_hdrlen,u->packets_out[0],0,send_len);
                      UIPEthernetClass::packetstate |= UIPETHERNET_SENDPACKET;
                    }
                }
              goto finish;
            }
        }
      // don't close connection unless all outgoing packets are sent
      if (u->state & UIP_CLIENT_CLOSE)
        {
#if ACTLOGLEVEL>=LOG_DEBUG_V2
          LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V2:UIPClient state UIP_CLIENT_CLOSE"));
          UIPClient::_dumpAllData();
#endif
          if (u->packets_out[0] == NOBLOCK)
            {
              u->state = 0;
              uip_conn->appstate = NULL;
              uip_close();
#if ACTLOGLEVEL>=LOG_DEBUG_V2
              LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG_V2:no blocks out -> free userdata"));
              UIPClient::_dumpAllData();
#endif
            }
          else
            {
              uip_stop();
#if ACTLOGLEVEL>=LOG_DEBUG
              LogObject.uart_send_strln(F("uipclient_appcall(void) DEBUG:blocks outstanding transfer -> uip_stop()"));
#endif
            }
        }
    }
  finish:
  uip_send(uip_appdata,send_len);
  uip_len = send_len;
#if ACTLOGLEVEL>=LOG_DEBUG_V3
  LogObject.uart_send_str(F("uipclient_appcall(void) DEBUG_V3: uip_len set to:"));
  LogObject.uart_send_decln(uip_len);
#endif
}

uip_userdata_t *
UIPClient::_allocateData()
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::_allocateData() DEBUG_V3:Function started"));
  #endif
  for ( uint8_t sock = 0; sock < UIP_CONNS; sock++ )
    {
      uip_userdata_t* data = &UIPClient::all_data[sock];
      if (!data->state)
        {
          data->conn_index = uip_conn - uip_conns;
          data->state = UIP_CLIENT_CONNECTED;
          memset(&data->packets_in[0],0,sizeof(uip_userdata_t)-sizeof(data->state));
          return data;
        }
    }
  return NULL;
}

uint8_t
UIPClient::_currentBlock(memhandle* block)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("UIPClient::_currentBlock(memhandle* block) DEBUG_V3:Function started"));
  #endif
  for (uint8_t i = 1; i < UIP_SOCKET_NUMPACKETS; i++)
    {
      if (block[i] == NOBLOCK)
        return i-1;
    }
  return UIP_SOCKET_NUMPACKETS-1;
}

void
UIPClient::_eatBlock(memhandle* block)
{
#if ACTLOGLEVEL>=LOG_DEBUG_V3
  LogObject.uart_send_strln(F("UIPClient::_eatBlock(memhandle* block) DEBUG_V3:Function started"));
#endif
#if ACTLOGLEVEL>=LOG_DEBUG
  memhandle* start = block;
  LogObject.uart_send_str(F("UIPClient::_eatBlock DEBUG:eatblock("));
  LogObject.uart_send_dec(*block);
  LogObject.uart_send_str(F("): "));
  for (uint8_t i = 0; i < UIP_SOCKET_NUMPACKETS; i++)
    {
      LogObject.uart_send_dec(start[i]);
      LogObject.uart_send_str(F(" "));
    }
  LogObject.uart_send_str(F("-> "));
#endif
  Enc28J60Network::freeBlock(block[0]);
  for (uint8_t i = 0; i < UIP_SOCKET_NUMPACKETS-1; i++)
    {
      block[i] = block[i+1];
    }
  block[UIP_SOCKET_NUMPACKETS-1] = NOBLOCK;
#if ACTLOGLEVEL>=LOG_DEBUG
  for (uint8_t i = 0; i < UIP_SOCKET_NUMPACKETS; i++)
    {
      LogObject.uart_send_dec(start[i]);
      LogObject.uart_send_str(F(" "));
    }
  LogObject.uart_send_strln(F(""));
#endif
}

void
UIPClient::_flushBlocks(memhandle* block)
{
#if ACTLOGLEVEL>=LOG_DEBUG_V3
  LogObject.uart_send_strln(F("UIPClient::_flushBlocks(memhandle* block) DEBUG_V3:Function started"));
#endif
  for (uint8_t i = 0; i < UIP_SOCKET_NUMPACKETS; i++)
    {
      Enc28J60Network::freeBlock(block[i]);
      block[i] = NOBLOCK;
    }
}

#if ACTLOGLEVEL>=LOG_DEBUG_V2
void
UIPClient::_dumpAllData(void) {
  for (uint8_t i=0; i < UIP_CONNS; i++)
    {
      LogObject.uart_send_str(F("UIPClient::_dumpAllData() DEBUG_V2:UIPClient::all_data["));
      LogObject.uart_send_dec(i);
      LogObject.uart_send_str(F("], state:"));
      LogObject.uart_send_binln(all_data[i].state);
      LogObject.uart_send_str(F("packets_in: "));
      for (uint8_t j=0; j < UIP_SOCKET_NUMPACKETS; j++)
        {
          LogObject.uart_send_dec(all_data[i].packets_in[j]);
          LogObject.uart_send_str(F(" "));
        }
      LogObject.uart_send_strln(F(""));
      if (all_data[i].state & UIP_CLIENT_REMOTECLOSED)
        {
          LogObject.uart_send_str(F("state remote closed, local port: "));
          LogObject.uart_send_decln(htons(((uip_userdata_closed_t *)(&all_data[i]))->lport));
        }
      else
        {
          LogObject.uart_send_str(F("packets_out: "));
          for (uint8_t j=0; j < UIP_SOCKET_NUMPACKETS; j++)
            {
              LogObject.uart_send_dec(all_data[i].packets_out[j]);
              LogObject.uart_send_str(F(" "));
            }
          LogObject.uart_send_strln(F(""));
          LogObject.uart_send_str(F("out_pos: "));
          LogObject.uart_send_decln(all_data[i].out_pos);
        }
    }
}
#endif
