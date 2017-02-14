/*
 UIPUdp.h - Arduino implementation of a uIP wrapper class
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
 
#ifndef UIPUDP_H
#define UIPUDP_H
 
#include "ethernet_comp.h"
#if defined(ARDUINO)
  #include <Arduino.h>
  #if defined(__RFduino__)
    #include "Print.h"
  #else
    #include "Print.h"
  #endif
  #if defined(__STM32F3__) || defined(STM32F3) || defined(__RFduino__)
    #include "mbed/Udp.h"
  #else
    #include <Udp.h>
  #endif
#endif
#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/Print.h"
  #include "mbed/Udp.h"
#endif
#include "utility/mempool.h"
extern "C" {
  #include "utility/uip.h"
}
 
#define UIP_UDP_MAXDATALEN 1500
#define UIP_UDP_PHYH_LEN UIP_LLH_LEN+UIP_IPUDPH_LEN
#define UIP_UDP_MAXPACKETSIZE UIP_UDP_MAXDATALEN+UIP_UDP_PHYH_LEN
 
typedef struct {
  memaddress out_pos;
  memhandle packet_next;
  memhandle packet_in;
  memhandle packet_out;
  bool send;
} uip_udp_userdata_t;
 
#if defined(ARDUINO) && !defined(STM32F3) && !defined(__RFduino__)
  class UIPUDP : public UDP {
#endif
#if defined(__MBED__) || defined(STM32F3) || defined(__RFduino__)
  class UIPUDP : public Print, public UDP {
#endif
private:
  struct uip_udp_conn *_uip_udp_conn;
 
  uip_udp_userdata_t appdata;
 
public:
  UIPUDP(void);   // Constructor
  virtual uint8_t   begin(uint16_t);// initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
  virtual void      stop(void);  // Finish with the UDP socket
 
  // Sending UDP packets
 
  // Start building up a packet to send to the remote host specific in ip and port
  // Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
  virtual int       beginPacket(IPAddress ip, uint16_t port);
  // Start building up a packet to send to the remote host specific in host and port
  // Returns 1 if successful, 0 if there was a problem resolving the hostname or port
  virtual int       beginPacket(const char *host, uint16_t port);
  // Finish off this packet and send it
  // Returns 1 if the packet was sent successfully, 0 if there was an error
  virtual int       endPacket(void);
  // Write a single byte into the packet
  virtual size_t    write(uint8_t);
  // Write size bytes from buffer into the packet
  virtual size_t    write(const uint8_t *buffer, size_t size);

  using Print::write;

  // Start processing the next available incoming packet
  // Returns the size of the packet in bytes, or 0 if no packets are available
  virtual int       parsePacket(void);
  // Number of bytes remaining in the current packet
  virtual int       available(void);
  // Read a single byte from the current packet
  virtual int       read(void);
  // Read up to len bytes from the current packet and place them into buffer
  // Returns the number of bytes read, or 0 if none are available
  virtual int       read(unsigned char* buffer, size_t len);
  // Read up to len characters from the current packet and place them into buffer
  // Returns the number of characters read, or 0 if none are available
  virtual int       read(char* buffer, size_t len)
  {
    return read((unsigned char*) buffer, len);
  }
  ;
  // Return the next byte from the current packet without moving on to the next byte
  virtual int       peek(void);
  virtual void      flush(void);    // Finish reading the current packet
 
  // Return the IP address of the host who sent the current incoming packet
  virtual IPAddress remoteIP(void);
 
  // Return the port of the host who sent the current incoming packet
  virtual uint16_t  remotePort(void);
 
private:
 
  friend void uipudp_appcall(void);
 
  friend class UIPEthernetClass;
  static void _send(uip_udp_userdata_t *data);
 
};
 
#endif
