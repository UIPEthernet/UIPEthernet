/*
 Advanced Chat Server

 A simple server that distributes any incoming messages to all
 connected clients but the client the message comes from.
 To use telnet to  your device's IP address and type.
 You can see the client's input in the serial monitor as well.
 Using an Arduino Wiznet Ethernet shield.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 * Analog inputs attached to pins A0 through A5 (optional)

 created 18 Dec 2009
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 redesigned to make use of operator== 25 Nov 2013
 by Norbert Truchsess

 */

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.

#define MACADDRESS 0x00,0x01,0x02,0x03,0x04,0x05
#define MYIPADDR 192,168,1,6
#define MYIPMASK 255,255,255,0
#define MYDNS 192,168,1,1
#define MYGW 192,168,1,1
// telnet defaults to port 23
#define LISTENPORT 23
#define UARTBAUD 115200

#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/millis.h"
  #define delay(x) wait_ms(x)
  #define PROGMEM
  #include "mbed/Print.h"
#endif

#include <UIPEthernet.h>
#include <utility/logging.h>

  uint8_t mac[6] = {MACADDRESS};
  uint8_t myIP[4] = {MYIPADDR};
  uint8_t myMASK[4] = {MYIPMASK};
  uint8_t myDNS[4] = {MYDNS};
  uint8_t myGW[4] = {MYGW};

EthernetServer server(LISTENPORT);

EthernetClient clients[4];

#if defined(ARDUINO)
void setup() {
#endif  
#if defined(__MBED__)
int main() {
#endif  
  // initialize the ethernet device
  //Ethernet.begin(mac,myIP);
  Ethernet.begin(mac,myIP,myDNS,myGW,myMASK);
  // start listening for clients
  server.begin();
 // Open serial communications and wait for port to open:
  #if ACTLOGLEVEL>LOG_NONE

    #if defined(ARDUINO)
      LogObject.begin(UARTBAUD);
    #endif
    #if defined(__MBED__)
      Serial LogObject(SERIAL_TX,SERIAL_RX);
      LogObject.baud(UARTBAUD);
    #endif
    while (!LogObject)
      {
      ; // wait for serial port to connect. Needed for Leonardo only
      }
  #endif

  #if ACTLOGLEVEL>=LOG_INFO
    LogObject.uart_send_str(F("Chat server listen on:"));
    #if defined(ARDUINO)
      LogObject.print(Ethernet.localIP()[0]);
      LogObject.print(F("."));
      LogObject.print(Ethernet.localIP()[1]);
      LogObject.print(F("."));
      LogObject.print(Ethernet.localIP()[2]);
      LogObject.print(F("."));
      LogObject.print(Ethernet.localIP()[3]);
      LogObject.print(F(":"));
      LogObject.println(LISTENPORT);
    #endif  
    #if defined(__MBED__)
      LogObject.printf("%d.%d.%d.%d:%d\r\n",Ethernet.localIP()[0],Ethernet.localIP()[1],Ethernet.localIP()[2],Ethernet.localIP()[3],LISTENPORT);
    #endif  
  #endif

#if defined(ARDUINO)
}

void loop() {
#endif  

#if defined(__MBED__)
while(true) {
#endif  
  // wait for a new client:
  EthernetClient client = server.available();

  if (client)
    {
    //check whether this client refers to the same socket as one of the existing instances:
    bool newClient = true;
    uint8_t i=0;
    while ((i<4) && (clients[i]!=client))
      {i++;}
    if (i<4) {newClient = false;}

    if (newClient)
      {
      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_strln(F("New client try connect"));
      #endif
      //Search unused client:
      uint8_t j=0;
      while ((j<4) && (clients[j]))
        {j++;}
      if (j>=4)
        {
        #if ACTLOGLEVEL>=LOG_INFO
          LogObject.uart_send_strln(F("Too many client"));
        #endif
        client.stop();
        }
      else
        {
        #if ACTLOGLEVEL>=LOG_INFO
          LogObject.uart_send_str(F("Save client to ID:"));
          LogObject.uart_send_decln(j);
        #endif
        clients[j] = client;
        // clead out the input buffer:
        client.flush();
        #if ACTLOGLEVEL>=LOG_INFO
          LogObject.uart_send_strln(F("We have a new client"));
        #endif
        client.println(F("Hello, client!"));
        client.print(F("my IP: "));
        client.println(Ethernet.localIP());
        }
      }

    if (client.available() > 0)
      {
      i=0;
      while ((i<4) && (clients[i]!=client))
        {i++;}
      #if ACTLOGLEVEL>=LOG_INFO
        LogObject.uart_send_str(F("Message received from client ID:"));
        LogObject.uart_send_decln(i);
      #endif
      // read the bytes incoming from the client:
      char thisChar = client.read();
      // echo the bytes back to all other connected clients:
      for (uint8_t j=0;j<4;j++)
        {
        if (clients[j] && clients[j]!=client)
          {
          #if ACTLOGLEVEL>=LOG_INFO
            LogObject.uart_send_str(F("Message forwarded to client ID:"));
            LogObject.uart_send_decln(j);
          #endif
          clients[j].write(thisChar);
          }
        }
      // echo the bytes to the server as well:
      #if ACTLOGLEVEL>=LOG_INFO
        #if defined(ARDUINO)
          LogObject.write(thisChar);
        #endif  
        #if defined(__MBED__)
          LogObject.putc(thisChar);
        #endif  
      #endif
    }
  }
  for (uint8_t i=0;i<4;i++) {
    if (!(clients[i].connected())) {
      // client.stop() invalidates the internal socket-descriptor, so next use of == will allways return false;
      clients[i].stop();
    }
  }
}

#if defined(__MBED__)
}
#endif
