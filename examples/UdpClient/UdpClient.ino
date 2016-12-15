/*
 * UIPEthernet UdpClient example.
 *
 * UIPEthernet is a TCP/IP stack that can be used with a enc28j60 based
 * Ethernet-shield.
 *
 * UIPEthernet uses the fine uIP stack by Adam Dunkels <adam@sics.se>
 *
 *      -----------------
 *
 * This UdpClient example tries to send a packet via udp to 192.168.0.1
 * on port 5000 every 5 seconds. After successfully sending the packet it
 * waits for up to 5 seconds for a response on the local port that has been
 * implicitly opened when sending the packet.
 *
 * Copyright (C) 2013 by Norbert Truchsess (norbert.truchsess@t-online.de)
 */

#include <UIPEthernet.h>
#include "utility/logging.h"

EthernetUDP udp;
unsigned long next;

void setup() {

  #if ACTLOGLEVEL>LOG_NONE
    LogObject.begin(9600);
  #endif

  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

  Ethernet.begin(mac,IPAddress(192,168,0,6));

  next = millis()+5000;
}

void loop() {

  int success;
  int len = 0;

  if (((signed long)(millis()-next))>0)
    {
      do
        {
          success = udp.beginPacket(IPAddress(192,168,0,1),5000);
          #if ACTLOGLEVEL>=LOG_INFO
            LogObject.uart_send_str(F("beginPacket: "));
            LogObject.uart_send_strln(success ? "success" : "failed");
          #endif
          //beginPacket fails if remote ethaddr is unknown. In this case an
          //arp-request is send out first and beginPacket succeeds as soon
          //the arp-response is received.
        }
      while (!success && ((signed long)(millis()-next))<0);
      if (!success )
        goto stop;

      success = udp.write("hello world from arduino");

      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_str(F("bytes written: "));
        LogObject.uart_send_decln(success);
      #endif

      success = udp.endPacket();

      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_str(F("endPacket: "));
        LogObject.uart_send_strln(success ? "success" : "failed");
      #endif

      do
        {
          //check for new udp-packet:
          success = udp.parsePacket();
        }
      while (!success && ((signed long)(millis()-next))<0);
      if (!success )
        goto stop;

      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_str(F("received: '"));
      #endif
      do
        {
          int c = udp.read();
          #if ACTLOGLEVEL>=LOG_INFO
            LogObject.write(c);
          #endif
          len++;
        }
      while ((success = udp.available())>0);
      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_str(F("', "));
        LogObject.uart_send_dec(len);
        LogObject.uart_send_strln(F(" bytes"));
      #endif

      //finish reading this packet:
      udp.flush();

      stop:
      udp.stop();
      next = millis()+5000;
    }
}
