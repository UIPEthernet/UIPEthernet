/*
 * UIPEthernet UdpServer example.
 *
 * UIPEthernet is a TCP/IP stack that can be used with a enc28j60 based
 * Ethernet-shield.
 *
 * UIPEthernet uses the fine uIP stack by Adam Dunkels <adam@sics.se>
 *
 *      -----------------
 *
 * This UdpServer example sets up a udp-server at 192.168.0.6 on port 5000.
 * send packet via upd to test
 *
 * Copyright (C) 2013 by Norbert Truchsess (norbert.truchsess@t-online.de)
 */

#include <UIPEthernet.h>
#include "utility/logging.h"

EthernetUDP udp;

void setup() {

  LogObject.begin(9600);

  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

  Ethernet.begin(mac,IPAddress(192,168,0,6));

  int success = udp.begin(5000);

  LogObject.print(F("initialize: "));
  LogObject.println(success ? "success" : "failed");

}

void loop() {

  //check for new udp-packet:
  int size = udp.parsePacket();
  if (size > 0) {
    do
      {
        char* msg = (char*)malloc(size+1);
        int len = udp.read(msg,size+1);
        msg[len]=0;
        LogObject.print(F("received: '"));
        LogObject.print(msg);
        free(msg);
      }
    while ((size = udp.available())>0);
    //finish reading this packet:
    udp.flush();
    LogObject.println(F("'"));
    int success;
    do
      {
        LogObject.print(F("remote ip: "));
        LogObject.println(udp.remoteIP());
        LogObject.print(F("remote port: "));
        LogObject.println(udp.remotePort());
        //send new packet back to ip/port of client. This also
        //configures the current connection to ignore packets from
        //other clients!
        success = udp.beginPacket(udp.remoteIP(),udp.remotePort());
        LogObject.print(F("beginPacket: "));
        LogObject.println(success ? "success" : "failed");
    //beginPacket fails if remote ethaddr is unknown. In this case an
    //arp-request is send out first and beginPacket succeeds as soon
    //the arp-response is received.
      }
    while (!success);

    success = udp.println(F("hello world from arduino"));

    LogObject.print(F("bytes written: "));
    LogObject.println(success);

    success = udp.endPacket();

    LogObject.print(F("endPacket: "));
    LogObject.println(success ? "success" : "failed");

    udp.stop();
    //restart with new connection to receive packets from other clients
    LogObject.print(F("restart connection: "));
    LogObject.println (udp.begin(5000) ? "success" : "failed");
  }
}
