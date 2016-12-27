/*
 * UIPEthernet TcpClient example.
 *
 * UIPEthernet is a TCP/IP stack that can be used with a enc28j60 based
 * Ethernet-shield.
 *
 * UIPEthernet uses the fine uIP stack by Adam Dunkels <adam@sics.se>
 *
 *      -----------------
 *
 * This TcpClient example gets its local ip-address via dhcp and sets
 * up a tcp socket-connection to 192.168.0.1 port 5000 every 5 Seconds.
 * After sending a message it waits for a response. After receiving the
 * response the client disconnects and tries to reconnect after 5 seconds.
 *
 * Copyright (C) 2013 by Norbert Truchsess <norbert.truchsess@t-online.de>
 */

#include <UIPEthernet.h>
#include "utility/logging.h"

EthernetClient client;
signed long next;

void setup() {

  #if ACTLOGLEVEL>LOG_NONE
    LogObject.begin(9600);
  #endif

  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
  Ethernet.begin(mac);

  #if ACTLOGLEVEL>=LOG_INFO
    LogObject.uart_send_str(F("localIP: "));
    LogObject.println(Ethernet.localIP());
    LogObject.uart_send_str(F("subnetMask: "));
    LogObject.println(Ethernet.subnetMask());
    LogObject.uart_send_str(F("gatewayIP: "));
    LogObject.println(Ethernet.gatewayIP());
    LogObject.uart_send_str(F("dnsServerIP: "));
    LogObject.println(Ethernet.dnsServerIP());
  #endif

  next = 0;
}

void loop() {

  if (((signed long)(millis() - next)) > 0)
    {
      next = millis() + 5000;
      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_strln(F("Client connect"));
      #endif
      // replace hostname with name of machine running tcpserver.pl
//      if (client.connect("server.local",5000))
      if (client.connect(IPAddress(192,168,0,1),5000))
        {
          #if ACTLOGLEVEL>=LOG_INFO
            LogObject.uart_send_strln(F("Client connected"));
          #endif
          client.println(F("DATA from Client"));
          int size;
          while ((client.available()==0) && (millis()<next))
            {
            #if defined(ESP8266)
              wdt_reset();
            #endif
            }
          while((size = client.available()) > 0)
            {
              uint8_t* msg = (uint8_t*)malloc(size);
              size = client.read(msg,size);
              #if ACTLOGLEVEL>=LOG_INFO
                LogObject.write(msg,size);
              #endif
              free(msg);
            }
          //disconnect client
          #if ACTLOGLEVEL>=LOG_INFO
            LogObject.uart_send_strln(F("Client disconnect"));
          #endif
          client.stop();
        }
      else
        #if ACTLOGLEVEL>=LOG_INFO
          LogObject.uart_send_strln(F("Client connect failed"));
        #endif
    }
}
