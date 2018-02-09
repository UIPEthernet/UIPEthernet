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

#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/millis.h"
  #define delay(x) wait_ms(x)
  #define PROGMEM
  #include "mbed/Print.h"
#endif

#include <UIPEthernet.h>
#include "utility/logging.h"

EthernetClient client;
unsigned long next;

#if defined(ARDUINO)
void setup() {
#endif  
#if defined(__MBED__)
int main() {
#endif
  #if ACTLOGLEVEL>LOG_NONE
    #if defined(ARDUINO)
      LogObject.begin(9600);
    #endif
    #if defined(__MBED__)
      Serial LogObject(SERIAL_TX,SERIAL_RX);
    #endif
  #endif

  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
  Ethernet.begin(mac); //Configure IP address via DHCP

  #if ACTLOGLEVEL>=LOG_INFO
    LogObject.uart_send_str(F("localIP: "));
    #if defined(ARDUINO)
      LogObject.println(Ethernet.localIP());
    #endif
    #if defined(__MBED__)
      LogObject.printf("%d.%d.%d.%d",Ethernet.localIP()[0],Ethernet.localIP()[1],Ethernet.localIP()[2],Ethernet.localIP()[3]);
    #endif
    LogObject.uart_send_str(F("subnetMask: "));
    #if defined(ARDUINO)
      LogObject.println(Ethernet.subnetMask());
    #endif
    #if defined(__MBED__)
      LogObject.printf("%d.%d.%d.%d",Ethernet.subnetMask()[0],Ethernet.subnetMask()[1],Ethernet.subnetMask()[2],Ethernet.subnetMask()[3]);
    #endif
    LogObject.uart_send_str(F("gatewayIP: "));
    #if defined(ARDUINO)
      LogObject.println(Ethernet.gatewayIP());
    #endif
    #if defined(__MBED__)
      LogObject.printf("%d.%d.%d.%d",Ethernet.gatewayIP()[0],Ethernet.gatewayIP()[1],Ethernet.gatewayIP()[2],Ethernet.gatewayIP()[3]);
    #endif
    LogObject.uart_send_str(F("dnsServerIP: "));
    #if defined(ARDUINO)
      LogObject.println(Ethernet.dnsServerIP());
    #endif
    #if defined(__MBED__)
      LogObject.printf("%d.%d.%d.%d",Ethernet.dnsServerIP()[0],Ethernet.dnsServerIP()[1],Ethernet.dnsServerIP()[2],Ethernet.dnsServerIP()[3]);
    #endif
  #endif

  next = 0;
#if defined(ARDUINO)
}

void loop() {
#endif  

#if defined(__MBED__)
while(true) {
#endif
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
              uint8_t* msg = (uint8_t*)malloc(size+1);
              memset(msg, 0, size+1);
              size = client.read(msg,size);
              #if ACTLOGLEVEL>=LOG_INFO
                #if defined(ARDUINO)
                  LogObject.write(msg,size);
                #endif
                #if defined(__MBED__)
                  LogObject.uart_send_str(msg);
                #endif
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
        {
        #if ACTLOGLEVEL>=LOG_INFO
          LogObject.uart_send_strln(F("Client connect failed"));
        #endif
        }
    }
}
#if defined(__MBED__)
}
#endif
