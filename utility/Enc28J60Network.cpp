/*
 Enc28J60NetworkClass.h
 UIPEthernet network driver for Microchip ENC28J60 Ethernet Interface.

 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 based on enc28j60.c file from the AVRlib library by Pascal Stang.
 For AVRlib See http://www.procyonengineering.com/

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

#include "Enc28J60Network.h"
#if defined(ARDUINO)
  #include "Arduino.h"
#endif
#if defined(__MBED__)
  #include <mbed.h>
  #include "mbed/millis.h"
  #define delay(x) wait_ms(x)
#endif
#include "logging.h"

uint8_t ENC28J60ControlCS = ENC28J60_CONTROL_CS;

#if ENC28J60_USE_SPILIB
   #if defined(ARDUINO)
     #if defined(STM32F2)
       #include <SPI.h>
     #elif !defined(STM32F3) && !defined(__STM32F4__)
       #include <SPI.h>
       extern SPIClass SPI;
     //#elif defined(ARDUINO_ARCH_AMEBA)
       //SPIClass SPI((void *)(&spi_obj), 11, 12, 13, 10);
       //SPI _spi(SPI_MOSI,SPI_MISO,SPI_SCK,ENC28J60ControlCS);
     #else
       #include "HardwareSPI.h"
       extern HardwareSPI SPI(1);
     #endif
   #endif
   #if defined(__MBED__)
     SPI _spi(SPI_MOSI,SPI_MISO,SPI_SCK);
     DigitalOut _cs(ENC28J60ControlCS);
     Serial LogObject(SERIAL_TX,SERIAL_RX);
   #endif
#endif

extern "C" {
  #if defined(ARDUINO_ARCH_AVR)
  // AVR-specific code
  #include <avr/io.h>
  #elif defined(ARDUINO_ARCH_SAM)
  // SAM-specific code
  #elif defined(ARDUINO_ARCH_SAMD)
  // SAMD-specific code
  #else
  // generic, non-platform specific code
  #endif
#include "enc28j60.h"
#include "uip.h"
}

#if defined(ARDUINO)
	// set CS to 0 = active
	#define CSACTIVE digitalWrite(ENC28J60ControlCS, LOW)
	// set CS to 1 = passive
	#define CSPASSIVE digitalWrite(ENC28J60ControlCS, HIGH)
#endif
#if defined(__MBED__)
   // set CS to 0 = active
   #define CSACTIVE _cs=0
   // set CS to 1 = passive
   #define CSPASSIVE _cs=1
#endif

//
#if defined(ARDUINO_ARCH_AVR)
#define waitspi() while(!(SPSR&(1<<SPIF)))
#endif

uint16_t Enc28J60Network::nextPacketPtr;
uint8_t Enc28J60Network::bank=0xff;
uint8_t Enc28J60Network::erevid=0;

struct memblock Enc28J60Network::receivePkt;

bool Enc28J60Network::broadcast_enabled = false;


void Enc28J60Network::init(uint8_t* macaddr)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::init(uint8_t* macaddr) DEBUG_V3:Function started"));
  #endif
  receivePkt.begin = 0;
  receivePkt.size = 0;

  unsigned int timeout = 15;
  MemoryPool::init(); // 1 byte in between RX_STOP_INIT and pool to allow prepending of controlbyte
  // initialize I/O
  // ss as output:
  #if defined(ARDUINO)
  	  pinMode(ENC28J60ControlCS, OUTPUT);
  #endif
  #if defined(__MBED__)
  	 millis_start(); 
  #endif
  CSPASSIVE; // ss=0
  //

  #if ACTLOGLEVEL>=LOG_DEBUG
    LogObject.uart_send_str(F("ENC28J60::init DEBUG:csPin = "));
    LogObject.uart_send_decln(ENC28J60ControlCS);
    LogObject.uart_send_str(F("ENC28J60::init DEBUG:miso = "));
    LogObject.uart_send_decln(SPI_MISO);
    LogObject.uart_send_str(F("ENC28J60::init DEBUG:mosi = "));
    LogObject.uart_send_decln(SPI_MOSI);
    LogObject.uart_send_str(F("ENC28J60::init DEBUG:sck = "));
    LogObject.uart_send_decln(SPI_SCK);
  #endif
#if ENC28J60_USE_SPILIB
  #if ACTLOGLEVEL>=LOG_DEBUG
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG:Use SPI lib SPI.begin()"));
  #endif
  #if defined(ARDUINO)
    #if defined(__STM32F3__) || defined(STM32F3) || defined(__STM32F4__)
      SPI.begin(SPI_9MHZ, MSBFIRST, 0);
    #else
      SPI.begin();
    #endif
  #endif
  #if defined(ARDUINO_ARCH_AVR)
    // AVR-specific code
    SPI.setClockDivider(SPI_CLOCK_DIV2); //results in 8MHZ at 16MHZ system clock.
  #elif defined(ARDUINO_ARCH_SAM)
    // SAM-specific code
    SPI.setClockDivider(10); //defaults to 21 which results in aprox. 4MHZ. A 10 should result in a little more than 8MHZ.
  #elif defined(ARDUINO_ARCH_SAMD)
    // SAMD-specific code
    // Should we set clock divider?
    SPI.setClockDivider(10);
  #elif defined(__STM32F1__) || defined(__STM32F3__)
    // generic, non-platform specific code
    #define USE_STM32F1_DMAC 1 //on STM32
    // BOARD_NR_SPI >= 1   BOARD_SPI1_NSS_PIN,     BOARD_SPI1_SCK_PIN,     BOARD_SPI1_MISO_PIN,     BOARD_SPI1_MOSI_PIN
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV8); //value 8 the result is 9MHz at 72MHz clock.
  #else
    #if defined(ARDUINO)
      #if !defined(__STM32F3__) && !defined(STM32F3) && !defined(__STM32F4__)
        SPI.setBitOrder(MSBFIRST);
      #endif
      //Settings for ESP8266
      //SPI.setDataMode(SPI_MODE0);
      //SPI.setClockDivider(SPI_CLOCK_DIV16);
    #endif
    #if defined(__MBED__)
      _spi.format(8, 0);          // 8bit, mode 0
      _spi.frequency(7000000);    // 7MHz
    #endif
  #endif
#else
  #if ACTLOGLEVEL>=LOG_DEBUG
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG:Use Native hardware SPI"));
  #endif
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_SCK, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  //Hardware SS must be configured as OUTPUT to enable SPI-master (regardless of which pin is configured as ENC28J60ControlCS)
  pinMode(SS, OUTPUT);
  digitalWrite(SS,HIGH);

  digitalWrite(SPI_MOSI, LOW);
  digitalWrite(SPI_SCK, LOW);

  // initialize SPI interface
  // master mode and Fosc/2 clock:
  SPCR = (1<<SPE)|(1<<MSTR);
  SPSR |= (1<<SPI2X);
#endif

  // perform system reset
  writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
  delay(2); // errata B7/2
  delay(50);
  // check CLKRDY bit to see if reset is complete
  // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
  //while(!(readReg(ESTAT) & ESTAT_CLKRDY));
  // do bank 0 stuff
  // initialize receive buffer
  // 16-bit transfers, must write low byte first
  // set receive buffer start address
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:Before readOp(ENC28J60_READ_CTRL_REG, ESTAT)"));
  #endif
  nextPacketPtr = RXSTART_INIT;
  while ((!readOp(ENC28J60_READ_CTRL_REG, ESTAT) & ESTAT_CLKRDY) && (timeout>0))
    {
    timeout=timeout-1;
    delay(10);
    #if defined(ESP8266)
       wdt_reset();
    #endif
    }
  #if ACTLOGLEVEL>=LOG_ERR
    if (timeout==0) {LogObject.uart_send_strln(F("ENC28J60::init ERROR:TIMEOUT !!"));}
  #endif
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:After readOp(ENC28J60_READ_CTRL_REG, ESTAT)"));
  #endif
  // Rx start
  writeRegPair(ERXSTL, RXSTART_INIT);
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:After writeRegPair(ERXSTL, RXSTART_INIT)"));
  #endif
  // set receive pointer address
  writeRegPair(ERXRDPTL, RXSTART_INIT);
  // RX end
  writeRegPair(ERXNDL, RXSTOP_INIT);
  // TX start
  //writeRegPair(ETXSTL, TXSTART_INIT);
  // TX end
  //writeRegPair(ETXNDL, TXSTOP_INIT);
  // do bank 1 stuff, packet filter:
  // For broadcast packets we allow only ARP packtets
  // All other packets should be unicast only for our mac (MAADR)
  //
  // The pattern to match on is therefore
  // Type     ETH.DST
  // ARP      BROADCAST
  // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
  // in binary these poitions are:11 0000 0011 1111
  // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
  //TODO define specific pattern to receive dhcp-broadcast packages instead of setting ERFCON_BCEN!
//    enableBroadcast(); // change to add ERXFCON_BCEN recommended by epam
  writeReg(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN);
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:After writeReg(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN)"));
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  writeRegPair(EPMM0, 0x303f);
  writeRegPair(EPMCSL, 0xf7f9);
  //
  //
  // do bank 2 stuff
  // enable MAC receive
  // and bring MAC out of reset (writes 0x00 to MACON2)
  writeRegPair(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
  // enable automatic padding to 60bytes and CRC operations
  writeOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:After writeOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN)"));
  #endif
  // set inter-frame gap (non-back-to-back)
  writeRegPair(MAIPGL, 0x0C12);
  // set inter-frame gap (back-to-back)
  writeReg(MABBIPG, 0x12);
  // Set the maximum packet size which the controller will accept
  // Do not send packets longer than MAX_FRAMELEN:
  writeRegPair(MAMXFLL, MAX_FRAMELEN);
  // do bank 3 stuff
  // write MAC address
  // NOTE: MAC address in ENC28J60 is byte-backward
  writeReg(MAADR5, macaddr[0]);
  writeReg(MAADR4, macaddr[1]);
  writeReg(MAADR3, macaddr[2]);
  writeReg(MAADR2, macaddr[3]);
  writeReg(MAADR1, macaddr[4]);
  writeReg(MAADR0, macaddr[5]);
  // no loopback of transmitted frames
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:Before phyWrite(PHCON2, PHCON2_HDLDIS)"));
  #endif
  phyWrite(PHCON2, PHCON2_HDLDIS);
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:After phyWrite(PHCON2, PHCON2_HDLDIS)"));
  #endif
  // switch to bank 0
  setBank(ECON1);
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:After setBank(ECON1)"));
  #endif
  // enable interrutps
  writeOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
  // enable packet reception
  writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
  //Configure leds
  phyWrite(PHLCON,0x476);

  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("ENC28J60::init DEBUG_V3:Before readReg(EREVID);"));
  #endif
  erevid=readReg(EREVID);
  if (erevid==0xFF) {erevid=0;}
  // microchip forgot to step the number on the silcon when they
  // released the revision B7. 6 is now rev B7. We still have
  // to see what they do when they release B8. At the moment
  // there is no B8 out yet
  //if (erevid > 5) ++erevid;
  #if ACTLOGLEVEL>=LOG_INFO
    LogObject.uart_send_str(F("ENC28J60::init INFO: Chip erevid="));
    LogObject.uart_send_dec(erevid);
    LogObject.uart_send_strln(F(" initialization completed."));
  #endif

//  return Enc28J60Network::erevid;
}

memhandle
Enc28J60Network::receivePacket(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::receivePacket(void) DEBUG_V3:Function started"));
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  uint8_t rxstat;
  uint16_t len;
  // check if a packet has been received and buffered
  //if( !(readReg(EIR) & EIR_PKTIF) ){
  // The above does not work. See Rev. B4 Silicon Errata point 6.
  #if ACTLOGLEVEL>=LOG_ERR
    if (erevid==0)
      {
      LogObject.uart_send_strln(F("Enc28J60Network::receivePacket(void) ERROR:ENC28j50 Device not found !! Bypass receivePacket function !!"));
      }
  #endif
  uint8_t epktcnt=readReg(EPKTCNT);
  if ((erevid!=0) && (epktcnt!=0))
    {
      uint16_t readPtr = nextPacketPtr+6 > RXSTOP_INIT ? nextPacketPtr+6-RXSTOP_INIT+RXSTART_INIT : nextPacketPtr+6;
      // Set the read pointer to the start of the received packet
      writeRegPair(ERDPTL, nextPacketPtr);
      // read the next packet pointer
      nextPacketPtr = readOp(ENC28J60_READ_BUF_MEM, 0);
      nextPacketPtr |= readOp(ENC28J60_READ_BUF_MEM, 0) << 8;
      // read the packet length (see datasheet page 43)
      len = readOp(ENC28J60_READ_BUF_MEM, 0);
      len |= readOp(ENC28J60_READ_BUF_MEM, 0) << 8;
      len -= 4; //remove the CRC count
      // read the receive status (see datasheet page 43)
      rxstat = readOp(ENC28J60_READ_BUF_MEM, 0);
      //rxstat |= readOp(ENC28J60_READ_BUF_MEM, 0) << 8;
      #if ACTLOGLEVEL>=LOG_DEBUG
        LogObject.uart_send_str(F("Enc28J60Network::receivePacket(void) DEBUG:receivePacket ["));
        LogObject.uart_send_hex(readPtr);
        LogObject.uart_send_str(F("-"));
        LogObject.uart_send_hex((readPtr+len) % (RXSTOP_INIT+1));
        LogObject.uart_send_str(F("], next: "));
        LogObject.uart_send_hex(nextPacketPtr);
        LogObject.uart_send_str(F(", stat: "));
        LogObject.uart_send_hex(rxstat);
        LogObject.uart_send_str(F(", Packet count: "));
        LogObject.uart_send_dec(epktcnt);
        LogObject.uart_send_str(F(" -> "));
        LogObject.uart_send_strln((rxstat & 0x80)!=0 ? "OK" : "failed");
      #endif
      // decrement the packet counter indicate we are done with this packet
      writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
      // check CRC and symbol errors (see datasheet page 44, table 7-3):
      // The ERXFCON.CRCEN is set by default. Normally we should not
      // need to check this.
      if (((rxstat & 0x80) != 0) && (nextPacketPtr<=RXSTOP_INIT))
        {
          receivePkt.begin = readPtr;
          receivePkt.size = len;
          #if ACTLOGLEVEL>=LOG_DEBUG
            LogObject.uart_send_str(F("Enc28J60Network::receivePacket(void) DEBUG: rxstat OK. receivePkt.size="));
            LogObject.uart_send_decln(len);
          #endif
          return UIP_RECEIVEBUFFERHANDLE;
        }
      // Move the RX read pointer to the start of the next received packet
      // This frees the memory we just read out
      setERXRDPT();
    }
  return (NOBLOCK);
}

void
Enc28J60Network::setERXRDPT(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::setERXRDPT(void) DEBUG_V3:Function started"));
  #endif
  // Make sure the value is odd. See Rev. B1,B4,B5,B7 Silicon Errata issues 14
  uint16_t actnextPacketPtr = nextPacketPtr == RXSTART_INIT ? RXSTOP_INIT : nextPacketPtr-1;
  #if ACTLOGLEVEL>=LOG_DEBUG
    LogObject.uart_send_str(F("Enc28J60Network::setERXRDPT(void) DEBUG:Set actnextPacketPtr:"));
    LogObject.uart_send_hexln(actnextPacketPtr);
  #endif
  // datasheet: The ENC28J60 will always write up to, but not including
  writeRegPair(ERXRDPTL, actnextPacketPtr);
}

memaddress
Enc28J60Network::blockSize(memhandle handle)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::blockSize(memhandle handle) DEBUG_V3:Function started"));
  #endif
  return ((handle == NOBLOCK) || (erevid==0)) ? 0 : handle == UIP_RECEIVEBUFFERHANDLE ? receivePkt.size : blocks[handle].size;
}

void
Enc28J60Network::sendPacket(memhandle handle)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::sendPacket(memhandle handle) INFO:Function started"));
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  if (erevid==0)
    {
    #if ACTLOGLEVEL>=LOG_ERR
      LogObject.uart_send_strln(F("Enc28J60Network::sendPacket(memhandle handle) ERROR:ENC28j50 Device not found !! Bypass sendPacket function !!"));
    #endif
    return;
    }

  memblock *packet = &blocks[handle];
  uint16_t start = packet->begin-1;
  uint16_t end = start + packet->size;

  // backup data at control-byte position
  uint8_t data = readByte(start);
  // write control-byte (if not 0 anyway)
  if (data)
    writeByte(start, 0);

  #if ACTLOGLEVEL>=LOG_DEBUG
    LogObject.uart_send_str(F("Enc28J60Network::sendPacket(memhandle handle) DEBUG:sendPacket("));
    LogObject.uart_send_dec(handle);
    LogObject.uart_send_str(F(") ["));
    LogObject.uart_send_hex(start);
    LogObject.uart_send_str(F("-"));
    LogObject.uart_send_hex(end);
    LogObject.uart_send_str(F("]: "));
    for (uint16_t i=start; i<=end; i++)
      {
      LogObject.uart_send_hex(readByte(i));
      LogObject.uart_send_str(F(" "));
      }
    LogObject.uart_send_strln(F(""));
  #endif

  // TX start
  writeRegPair(ETXSTL, start);
  // Set the TXND pointer to correspond to the packet size given
  writeRegPair(ETXNDL, end);
  // send the contents of the transmit buffer onto the network
 
  unsigned int retry = TX_COLLISION_RETRY_COUNT;
  unsigned int timeout = 100;
  do
    {
    // seydamir added
    // Reset the transmit logic problem. See Rev. B7 Silicon Errata issues 12 and 13
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
    writeOp(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF | EIR_TXIF);
    // end
 
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
    //if( (readReg(EIR) & EIR_TXERIF) )
    //  {
    //    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
    //  }

    timeout = 100;
    while (((readReg(EIR) & (EIR_TXIF | EIR_TXERIF)) == 0) && (timeout>0))
      {
      timeout=timeout-1;
      delay(10);
      #if defined(ESP8266)
         wdt_reset();
      #endif
      }
    if (timeout==0)
      {
      /* Transmit hardware probably hung, try again later. */
      /* Shouldn't happen according to errata 12 and 13. */
      writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);	
      #if ACTLOGLEVEL>=LOG_WARN
        LogObject.uart_send_strln(F("Enc28J60Network::sendPacket(memhandle handle) WARNING:Collision"));
      #endif
      retry=retry-1;
      }
    } while ((timeout == 0) && (retry != 0));

  //restore data on control-byte position
  if (data)
    writeByte(start, data);

  if (retry == 0)
    {
    #if ACTLOGLEVEL>=LOG_ERROR
      LogObject.uart_send_strln(F("Enc28J60Network::sendPacket(memhandle handle) ERROR:COLLISION !!"));
    #endif
    return;
    }
}

uint16_t
Enc28J60Network::setReadPtr(memhandle handle, memaddress position, uint16_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::setReadPtr(memhandle handle, memaddress position, uint16_t len) DEBUG_V3:Function started"));
  #endif
  memblock *packet = handle == UIP_RECEIVEBUFFERHANDLE ? &receivePkt : &blocks[handle];
  memaddress start = handle == UIP_RECEIVEBUFFERHANDLE && packet->begin + position > RXSTOP_INIT ? packet->begin + position-RXSTOP_INIT+RXSTART_INIT : packet->begin + position;

  writeRegPair(ERDPTL, start);
  
  if (len > packet->size - position)
    len = packet->size - position;
  return len;
}

uint16_t
Enc28J60Network::readPacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::readPacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len) DEBUG_V3:Function started"));
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  len = setReadPtr(handle, position, len);
  readBuffer(len, buffer);
  #if ACTLOGLEVEL>=LOG_DEBUG_V2
    LogObject.uart_send_str(F("Enc28J60Network::readPacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len) DEBUG_V2: Read bytes:"));
    LogObject.uart_send_dec(len);
    LogObject.uart_send_str(F(" save to block("));
    LogObject.uart_send_dec(handle);
    LogObject.uart_send_str(F(") ["));
    LogObject.uart_send_hex(position);
    LogObject.uart_send_str(F("]: "));
    for (uint16_t i=0; i<len; i++)
      {
      LogObject.uart_send_hex(buffer[i]);
      LogObject.uart_send_str(F(" "));
      }
    LogObject.uart_send_strln(F(""));
  #endif
  return len;
}

uint16_t
Enc28J60Network::writePacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_str(F("Enc28J60Network::writePacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len) DEBUG_V3:Function started with len:"));
    LogObject.uart_send_decln(len);
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  memblock *packet = &blocks[handle];
  uint16_t start = packet->begin + position;

  writeRegPair(EWRPTL, start);

  if (len > packet->size - position)
    len = packet->size - position;
  writeBuffer(len, buffer);
  #if ACTLOGLEVEL>=LOG_DEBUG_V2
    LogObject.uart_send_str(F("Enc28J60Network::writePacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len) DEBUG_V2: Write bytes:"));
    LogObject.uart_send_dec(len);
    LogObject.uart_send_str(F(" save to block("));
    LogObject.uart_send_dec(handle);
    LogObject.uart_send_str(F(") ["));
    LogObject.uart_send_hex(start);
    LogObject.uart_send_str(F("]: "));
    for (uint16_t i=0; i<len; i++)
      {
      LogObject.uart_send_hex(buffer[i]);
      LogObject.uart_send_str(F(" "));
      }
    LogObject.uart_send_strln(F(""));
  #endif
  return len;
}


void Enc28J60Network::enableBroadcast (bool temporary) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::enableBroadcast (bool temporary) DEBUG_V3:Function started"));
  #endif
    writeRegByte(ERXFCON, readRegByte(ERXFCON) | ERXFCON_BCEN);
    if(!temporary)
        broadcast_enabled = true;
}

void Enc28J60Network::disableBroadcast (bool temporary) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::disableBroadcast (bool temporary) DEBUG_V3:Function started"));
  #endif
    if(!temporary)
        broadcast_enabled = false;
    if(!broadcast_enabled)
        writeRegByte(ERXFCON, readRegByte(ERXFCON) & ~ERXFCON_BCEN);
}

void Enc28J60Network::enableMulticast (void) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::enableMulticast (void) DEBUG_V3:Function started"));
  #endif
    writeRegByte(ERXFCON, readRegByte(ERXFCON) | ERXFCON_MCEN);
}

void Enc28J60Network::disableMulticast (void) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::disableMulticast (void) DEBUG_V3:Function started"));
  #endif
    writeRegByte(ERXFCON, readRegByte(ERXFCON) & ~ERXFCON_MCEN);
}

uint8_t Enc28J60Network::readRegByte (uint8_t address) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::readRegByte (uint8_t address) DEBUG_V3:Function started"));
  #endif
    setBank(address);
    return readOp(ENC28J60_READ_CTRL_REG, address);
}

void Enc28J60Network::writeRegByte (uint8_t address, uint8_t data) {
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::writeRegByte (uint8_t address, uint8_t data) DEBUG_V3:Function started"));
  #endif
    setBank(address);
    writeOp(ENC28J60_WRITE_CTRL_REG, address, data);
}


uint8_t Enc28J60Network::readByte(uint16_t addr)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::readByte(uint16_t addr) DEBUG_V3:Function started"));
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  writeRegPair(ERDPTL, addr);

  CSACTIVE;
  #if ENC28J60_USE_SPILIB
    // issue read command
    #if defined(ARDUINO)
      SPI.transfer(ENC28J60_READ_BUF_MEM);
      // read data
      uint8_t c = SPI.transfer(0x00);
    #endif
    #if defined(__MBED__)
      _spi.write(ENC28J60_READ_BUF_MEM);
      // read data
      uint8_t c = _spi.write(0x00);
    #endif
    CSPASSIVE;
    return (c);
  #else
    // issue read command
    SPDR = ENC28J60_READ_BUF_MEM;
    waitspi();
    // read data
    SPDR = 0x00;
    waitspi();
    CSPASSIVE;
    return (SPDR);
  #endif  
}

void Enc28J60Network::writeByte(uint16_t addr, uint8_t data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::writeByte(uint16_t addr, uint8_t data) DEBUG_V3:Function started"));
  #endif
  #if defined(ESP8266)
     wdt_reset();
  #endif
  writeRegPair(EWRPTL, addr);

  CSACTIVE;
  #if ENC28J60_USE_SPILIB
    // issue write command
    #if defined(ARDUINO)
      SPI.transfer(ENC28J60_WRITE_BUF_MEM);
      // write data
      SPI.transfer(data);
    #endif
    #if defined(__MBED__)
      _spi.write(ENC28J60_WRITE_BUF_MEM);
      // write data
      _spi.write(data);
    #endif
  #else
    // issue write command
    SPDR = ENC28J60_WRITE_BUF_MEM;
    waitspi();
    // write data
    SPDR = data;
    waitspi();
  #endif
  CSPASSIVE;
}

void
Enc28J60Network::copyPacket(memhandle dest_pkt, memaddress dest_pos, memhandle src_pkt, memaddress src_pos, uint16_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::copyPacket(memhandle dest_pkt, memaddress dest_pos, memhandle src_pkt, memaddress src_pos, uint16_t len) DEBUG_V3:Function started"));
  #endif
  memblock *dest = &blocks[dest_pkt];
  memblock *src = src_pkt == UIP_RECEIVEBUFFERHANDLE ? &receivePkt : &blocks[src_pkt];
  memaddress start = src_pkt == UIP_RECEIVEBUFFERHANDLE && src->begin + src_pos > RXSTOP_INIT ? src->begin + src_pos-RXSTOP_INIT+RXSTART_INIT : src->begin + src_pos;
  enc28J60_mempool_block_move_callback(dest->begin+dest_pos,start,len);
  // setERXRDPT(); let it to freePacket after all packets are saved
}

void
enc28J60_mempool_block_move_callback(memaddress dest, memaddress src, memaddress len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("enc28J60_mempool_block_move_callback(memaddress dest, memaddress src, memaddress len) DEBUG_V3:Function started"));
  #endif
//void
//Enc28J60Network::memblock_mv_cb(uint16_t dest, uint16_t src, uint16_t len)
//{
  //as ENC28J60 DMA is unable to copy single bytes:
  if (len == 1)
    {
      Enc28J60Network::writeByte(dest,Enc28J60Network::readByte(src));
    }
  else
    {
      // calculate address of last byte
      len += src - 1;

      /*  1. Appropriately program the EDMAST, EDMAND
       and EDMADST register pairs. The EDMAST
       registers should point to the first byte to copy
       from, the EDMAND registers should point to the
       last byte to copy and the EDMADST registers
       should point to the first byte in the destination
       range. The destination range will always be
       linear, never wrapping at any values except from
       8191 to 0 (the 8-Kbyte memory boundary).
       Extreme care should be taken when
       programming the start and end pointers to
       prevent a never ending DMA operation which
       would overwrite the entire 8-Kbyte buffer.
       */
      Enc28J60Network::writeRegPair(EDMASTL, src);
      Enc28J60Network::writeRegPair(EDMADSTL, dest);

      if ((src <= RXSTOP_INIT)&& (len > RXSTOP_INIT))len -= ((RXSTOP_INIT + 1)-RXSTART_INIT);
      Enc28J60Network::writeRegPair(EDMANDL, len);

      /*
       2. If an interrupt at the end of the copy process is
       desired, set EIE.DMAIE and EIE.INTIE and
       clear EIR.DMAIF.

       3. Verify that ECON1.CSUMEN is clear. */
      Enc28J60Network::writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_CSUMEN);

      /* 4. Start the DMA copy by setting ECON1.DMAST. */
      Enc28J60Network::writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_DMAST);

      // wait until runnig DMA is completed
      while (Enc28J60Network::readOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_DMAST)
         {
         delay(1);
         }
    }
}

void
Enc28J60Network::freePacket(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::freePacket(void) DEBUG_V3:Function started"));
  #endif
    setERXRDPT();
}

uint8_t
Enc28J60Network::readOp(uint8_t op, uint8_t address)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::readOp(uint8_t op, uint8_t address) DEBUG_V3:Function started"));
  #endif
  CSACTIVE;
  // issue read command
  #if ENC28J60_USE_SPILIB
    #if defined(ARDUINO)
      SPI.transfer(op | (address & ADDR_MASK));
      // read data
      if(address & 0x80)
        {
        // do dummy read if needed (for mac and mii, see datasheet page 29)
        SPI.transfer(0x00);
        }
      uint8_t c = SPI.transfer(0x00);
    #endif
    #if defined(__MBED__)
      _spi.write(op | (address & ADDR_MASK));
      // read data
      if(address & 0x80)
        {
        // do dummy read if needed (for mac and mii, see datasheet page 29)
        _spi.write(0x00);
        }
      uint8_t c = _spi.write(0x00);
    #endif
    // release CS
    CSPASSIVE;
    return(c);
  #else
    // issue read command
    SPDR = op | (address & ADDR_MASK);
    waitspi();
    // read data
    SPDR = 0x00;
    waitspi();
    // do dummy read if needed (for mac and mii, see datasheet page 29)
    if(address & 0x80)
      {
      SPDR = 0x00;
      waitspi();
      }
    // release CS
    CSPASSIVE;
    return(SPDR);
  #endif
  #if defined(ESP8266)
     yield();
  #endif
}

void
Enc28J60Network::writeOp(uint8_t op, uint8_t address, uint8_t data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::writeOp(uint8_t op, uint8_t address, uint8_t data) DEBUG_V3:Function started"));
  #endif
  CSACTIVE;
  // issue write command
  #if ENC28J60_USE_SPILIB
    #if defined(ARDUINO)
      SPI.transfer(op | (address & ADDR_MASK));
      // write data
      SPI.transfer(data);
    #endif
    #if defined(__MBED__)
      _spi.write(op | (address & ADDR_MASK));
      // write data
      _spi.write(data);
    #endif
  #else
    // issue write command
    SPDR = op | (address & ADDR_MASK);
    waitspi();
    // write data
    SPDR = data;
    waitspi();
  #endif
  CSPASSIVE;
  #if defined(ESP8266)
     yield();
  #endif
}

void
Enc28J60Network::readBuffer(uint16_t len, uint8_t* data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::readBuffer(uint16_t len, uint8_t* data) DEBUG_V3:Function started"));
  #endif
  CSACTIVE;
  // issue read command
  #if ENC28J60_USE_SPILIB  
    #if defined(ARDUINO)
      SPI.transfer(ENC28J60_READ_BUF_MEM);
    #endif
    #if defined(__MBED__)
      _spi.write(ENC28J60_READ_BUF_MEM);
    #endif
  #else
    SPDR = ENC28J60_READ_BUF_MEM;
    waitspi();
  #endif
  while(len)
    {
    len--;
    // read data
    #if ENC28J60_USE_SPILIB    
      #if defined(ARDUINO)
        *data = SPI.transfer(0x00);
      #endif
      #if defined(__MBED__)
        *data = _spi.write(0x00);
      #endif
    #else
      SPDR = 0x00;
      waitspi();
      *data = SPDR;
    #endif    
    data++;
    }
  //*data='\0';
  CSPASSIVE;
}

void
Enc28J60Network::writeBuffer(uint16_t len, uint8_t* data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::writeBuffer(uint16_t len, uint8_t* data) DEBUG_V3:Function started"));
  #endif
  CSACTIVE;
  // issue write command
  #if ENC28J60_USE_SPILIB  
    #if defined(ARDUINO)
      SPI.transfer(ENC28J60_WRITE_BUF_MEM);
    #endif
    #if defined(__MBED__)
      _spi.write(ENC28J60_WRITE_BUF_MEM);
    #endif
  #else
    SPDR = ENC28J60_WRITE_BUF_MEM;
    waitspi();
  #endif
  while(len)
    {
    len--;
    // write data
    #if ENC28J60_USE_SPILIB  
      #if defined(ARDUINO)
        SPI.transfer(*data);
      #endif
      #if defined(__MBED__)
        _spi.write(*data);
      #endif
      data++;
    #else
      SPDR = *data;
      data++;
      waitspi();
    #endif
    }
  CSPASSIVE;
}

void
Enc28J60Network::setBank(uint8_t address)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::setBank(uint8_t address) DEBUG_V3:Function started"));
  #endif
  // set the bank (if needed)
  if((address & BANK_MASK) != bank)
  {
    // set the bank
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
    bank = (address & BANK_MASK);
  }
}

uint8_t
Enc28J60Network::readReg(uint8_t address)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::readReg(uint8_t address) DEBUG_V3:Function started"));
  #endif
  // set the bank
  setBank(address);
  // do the read
  return readOp(ENC28J60_READ_CTRL_REG, address);
}

void
Enc28J60Network::writeReg(uint8_t address, uint8_t data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::writeReg(uint8_t address, uint8_t data) DEBUG_V3:Function started"));
  #endif
  // set the bank
  setBank(address);
  // do the write
  writeOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void
Enc28J60Network::writeRegPair(uint8_t address, uint16_t data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::writeRegPair(uint8_t address, uint16_t data) DEBUG_V3:Function started"));
  #endif
  // set the bank
  setBank(address);
  // do the write
  writeOp(ENC28J60_WRITE_CTRL_REG, address, (data&0xFF));
  writeOp(ENC28J60_WRITE_CTRL_REG, address+1, (data) >> 8);
}

void
Enc28J60Network::phyWrite(uint8_t address, uint16_t data)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::phyWrite(uint8_t address, uint16_t data) DEBUG_V3:Function started"));
  #endif
  unsigned int timeout = 15;
  // set the PHY register address
  writeReg(MIREGADR, address);
  // write the PHY data
  writeRegPair(MIWRL, data);
  // wait until the PHY write completes
  while (readReg(MISTAT) & MISTAT_BUSY)
    {
    delay(10);
    #if defined(ESP8266)
       wdt_reset();
    #endif
    if (--timeout == 0)
      {
      #if ACTLOGLEVEL>=LOG_ERR
         LogObject.uart_send_strln(F("Enc28J60Network::phyWrite ERROR:TIMEOUT !!"));
      #endif
      return;
      }
    }
}

uint16_t
Enc28J60Network::phyRead(uint8_t address)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::phyRead(uint8_t address) DEBUG_V3:Function started"));
  #endif
  unsigned int timeout = 15;
  writeReg(MIREGADR,address);
  writeReg(MICMD, MICMD_MIIRD);
  // wait until the PHY read completes
  while(readReg(MISTAT) & MISTAT_BUSY)
    {
    delay(10);
    #if defined(ESP8266)
       wdt_reset();
    #endif
    if (--timeout == 0)
      {
      #if ACTLOGLEVEL>=LOG_ERR
         LogObject.uart_send_strln(F("Enc28J60Network::phyRead ERROR:TIMEOUT !!"));
      #endif
      return 0;
      }
    }
  writeReg(MICMD, 0);
  return (readReg(MIRDL) | readReg(MIRDH) << 8);
}

void
Enc28J60Network::clkout(uint8_t clk)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::clkout(uint8_t clk) DEBUG_V3:Function started"));
  #endif
  //setup clkout: 2 is 12.5MHz:
  writeReg(ECOCON, clk & 0x7);
}

uint16_t
Enc28J60Network::chksum(uint16_t sum, memhandle handle, memaddress pos, uint16_t len)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::chksum(uint16_t sum, memhandle handle, memaddress pos, uint16_t len) DEBUG_V3:Function started"));
  #endif
  uint16_t t;
  len = setReadPtr(handle, pos, len)-1;
  CSACTIVE;
  // issue read command
  #if ENC28J60_USE_SPILIB
    #if defined(ARDUINO)
    SPI.transfer(ENC28J60_READ_BUF_MEM);
    #endif
    #if defined(__MBED__)
    _spi.write(ENC28J60_READ_BUF_MEM);
    #endif
  #else
    SPDR = ENC28J60_READ_BUF_MEM;
    waitspi();
  #endif
  uint16_t i;
  for (i = 0; i < len; i+=2)
    {
    // read data
    #if ENC28J60_USE_SPILIB
      #if defined(ARDUINO)
        t = SPI.transfer(0x00) << 8;
        t += SPI.transfer(0x00);
      #endif
      #if defined(__MBED__)
        t = _spi.write(0x00) << 8;
        t += _spi.write(0x00);
      #endif
    #else
      SPDR = 0x00;
      waitspi();
      t = SPDR << 8;
      SPDR = 0x00;
      waitspi();
      t += SPDR;
    #endif
    sum += t;
    if(sum < t)
      {
      sum++;            /* carry */
      }
    }
  if(i == len)
    {
    #if ENC28J60_USE_SPILIB  
      #if defined(ARDUINO)
        t = (SPI.transfer(0x00) << 8) + 0;
      #endif
      #if defined(__MBED__)
        t = (_spi.write(0x00) << 8) + 0;
      #endif
    #else
      SPDR = 0x00;
      waitspi();
      t = (SPDR << 8) + 0;
    #endif    
    sum += t;
    if(sum < t)
      {
      sum++;            /* carry */
      }
    }
  CSPASSIVE;

  /* Return sum in host byte order. */
  return sum;
}

void
Enc28J60Network::powerOff(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::powerOff(void) DEBUG_V3:Function started"));
  #endif
  writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
  delay(50);
  writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_VRPS);
  delay(50);
  writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PWRSV);
}

void
Enc28J60Network::powerOn(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::powerOn(void) DEBUG_V3:Function started"));
  #endif
  writeOp(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
  delay(50);
  writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
  delay(50);
}

// read erevid from object:
uint8_t
Enc28J60Network::geterevid(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_str(F("Enc28J60Network::geterevid(void) DEBUG_V3:Function started and return:"));
    LogObject.uart_send_decln(erevid);
  #endif
  return(erevid);
}

// read the phstat2 of the chip:
uint16_t
Enc28J60Network::PhyStatus(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_str(F("Enc28J60Network::PhyStatus(void) DEBUG_V3:Function started"));
    LogObject.uart_send_decln(erevid);
  #endif
  uint16_t phstat2;
  phstat2=phyRead(PHSTAT2);
  if ((phstat2 & 0x20) > 0) {phstat2=phstat2 &0x100;}
  phstat2=(phstat2 & 0xFF00) | erevid;
  if ((phstat2 & 0x8000) > 0) {phstat2=0;}
  return phstat2;
}

bool
Enc28J60Network::linkStatus(void)
{
  #if ACTLOGLEVEL>=LOG_DEBUG_V3
    LogObject.uart_send_strln(F("Enc28J60Network::linkStatus(void) DEBUG_V3:Function started"));
  #endif
  return (phyRead(PHSTAT2) & 0x0400) > 0;
}

Enc28J60Network Enc28J60;
