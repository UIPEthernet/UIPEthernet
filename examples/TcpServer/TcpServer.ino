/*
 * UIPEthernet TCPServer example.
 *
 * UIPEthernet is a TCP/IP stack that can be used with a enc28j60 based
 * Ethernet-shield.
 *
 * UIPEthernet uses the fine uIP stack by Adam Dunkels <adam@sics.se>
 *
 *      -----------------
 *
 * This Hello World example sets up a server at 192.168.1.6 on port 1000.
 * Telnet here to access the service.  The uIP stack will also respond to
 * pings to test if you have successfully established a TCP connection to
 * the Arduino.
 *
 * This example was based upon uIP hello-world by Adam Dunkels <adam@sics.se>
 * Ported to the Arduino IDE by Adam Nielsen <malvineous@shikadi.net>
 * Adaption to Enc28J60 by Norbert Truchsess <norbert.truchsess@t-online.de>
 */

#define MACADDRESS 0x00,0x01,0x02,0x03,0x04,0x05
#define MYIPADDR 192,168,1,6
#define MYIPMASK 255,255,255,0
#define MYDNS 192,168,1,1
#define MYGW 192,168,1,1
#define LISTENPORT 1000
#define UARTBAUD 115200

#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/millis.h"
  #define delay(x) wait_ms(x)
  #define PROGMEM
  #include "mbed/Print.h"
#endif

#include <UIPEthernet.h>
#include "utility/logging.h"

EthernetServer server = EthernetServer(LISTENPORT);

#if defined(ARDUINO)
void setup() {
#endif  
#if defined(__MBED__)
int main() {
#endif
  #if ACTLOGLEVEL>LOG_NONE
    #if defined(ARDUINO)
      LogObject.begin(UARTBAUD);
    #endif
    #if defined(__MBED__)
      Serial LogObject(SERIAL_TX,SERIAL_RX);
      LogObject.baud(UARTBAUD);
    #endif
  #endif

  uint8_t mac[6] = {MACADDRESS};
  uint8_t myIP[4] = {MYIPADDR};
  uint8_t myMASK[4] = {MYIPMASK};
  uint8_t myDNS[4] = {MYDNS};
  uint8_t myGW[4] = {MYGW};

//  Ethernet.begin(mac,myIP);
  Ethernet.begin(mac,myIP,myDNS,myGW,myMASK);

  server.begin();
#if defined(ARDUINO)
}

void loop() {
#endif  

#if defined(__MBED__)
while(true) {
#endif
  size_t size;

  if (EthernetClient client = server.available())
    {
      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_str(F("DATA from Client:'"));
      #endif
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
      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_strln(F("'"));
      #endif
      client.stop();
    }
}
#if defined(__MBED__)
}
#endif
