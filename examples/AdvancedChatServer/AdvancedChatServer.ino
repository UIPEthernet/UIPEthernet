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

#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/millis.h"
  #define delay(x) wait_ms(x)
  #define PROGMEM
  #include "mbed/Print.h"
#endif

#include <UIPEthernet.h>
#include <utility/logging.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.

uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,0,6);

// telnet defaults to port 23
EthernetServer server(23);

EthernetClient clients[4];

#if defined(ARDUINO)
void setup() {
#endif  
#if defined(__MBED__)
int main() {
#endif  
  // initialize the ethernet device
  Ethernet.begin(mac, ip);
  // start listening for clients
  server.begin();
 // Open serial communications and wait for port to open:
  #if ACTLOGLEVEL>LOG_NONE

    #if defined(ARDUINO)
      LogObject.begin(9600);
    #endif
    #if defined(__MBED__)
      Serial LogObject(SERIAL_TX,SERIAL_RX);
    #endif
    while (!LogObject)
      {
      ; // wait for serial port to connect. Needed for Leonardo only
      }
  #endif

  #if ACTLOGLEVEL>=LOG_INFO
    LogObject.uart_send_str(F("Chat server address:"));
    #if defined(ARDUINO)
      LogObject.println(Ethernet.localIP());
    #endif  
    #if defined(__MBED__)
      LogObject.printf("%d.%d.%d.%d",Ethernet.localIP()[0],Ethernet.localIP()[1],Ethernet.localIP()[2],Ethernet.localIP()[3]);
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

  if (client) {

    bool newClient = true;
    for (uint8_t i=0;i<4;i++) {
      //check whether this client refers to the same socket as one of the existing instances:
      if (clients[i]==client) {
        newClient = false;
        break;
      }
    }

    if (newClient) {
      //check which of the existing clients can be overridden:
      for (uint8_t i=0;i<4;i++) {
        if (!clients[i] && clients[i]!=client) {
          clients[i] = client;
          // clead out the input buffer:
          client.flush();
          // clead out the input buffer:
          client.flush();
          #if ACTLOGLEVEL>=LOG_INFO
            LogObject.uart_send_strln(F("We have a new client"));
          #endif
            client.println(F("Hello, client!"));
            client.print(F("my IP: "));
            client.println(Ethernet.localIP());
          break;
        }
      }
    }

    if (client.available() > 0) {
      // read the bytes incoming from the client:
      char thisChar = client.read();
      // echo the bytes back to all other connected clients:
      for (uint8_t i=0;i<4;i++) {
        if (clients[i] && clients[i]!=client) {
          clients[i].write(thisChar);
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
