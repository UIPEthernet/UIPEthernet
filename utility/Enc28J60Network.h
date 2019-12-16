/*
 Enc28J60NetworkClass.h
 UIPEthernet network driver for Microchip ENC28J60 Ethernet Interface.

 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 inspired by enc28j60.c file from the AVRlib library by Pascal Stang.
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

#ifndef Enc28J60Network_H_
#define Enc28J60Network_H_

#include "mempool.h"
#if defined(__MBED__)
  #include <mbed.h>
  //UIPEthernet(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);
  #if defined(TARGET_LPC1768)
    #define SPI_MOSI p11
    #define SPI_MISO p12
    #define SPI_SCK  p13
    #define SPI_CS   p8
  #elif defined(TARGET_LPC1114)
    #define SPI_MOSI dp2
    #define SPI_MISO dp1
    #define SPI_SCK  dp6
    #define SPI_CS   dp25
  #elif defined(TARGET_LPC11U68)
    #define SPI_MOSI P0_9
    #define SPI_MISO P0_8
    #define SPI_SCK  P1_29
    #define SPI_CS   P0_2
  #elif defined(TARGET_NUCLEO_F103RB) || defined(TARGET_NUCLEO_L152RE) || defined(TARGET_NUCLEO_F030R8)  \
    || defined(TARGET_NUCLEO_F401RE) || defined(TARGET_NUCLEO_F302R8) || defined(TARGET_NUCLEO_L053R8)  \
    || defined(TARGET_NUCLEO_F411RE) || defined(TARGET_NUCLEO_F334R8) || defined(TARGET_NUCLEO_F072RB)  \
    || defined(TARGET_NUCLEO_F091RC) || defined(TARGET_NUCLEO_F303RE) || defined(TARGET_NUCLEO_F070RB)
    #define SPI_MOSI D4
    #define SPI_MISO D5
    #define SPI_SCK  D3
    #define SPI_CS   D2
  #endif
  #define ENC28J60_CONTROL_CS SPI_CS
#endif

#if defined(STM32F3) || defined(STM32F2)                //This is workaround for stm32duino STM32F2, and adafruit wiced feather STM32F2
  #define BOARD_SPI1_NSS_PIN        PA4
  #define BOARD_SPI1_SCK_PIN        PA5
  #define BOARD_SPI1_MISO_PIN       PA6
  #define BOARD_SPI1_MOSI_PIN       PA7
#endif                              			//This is workaround for stm32duino STM32F3, and adafruit wiced feather STM32F2

#if defined(BOARD_discovery_f4)
  #define __STM32F4__
#endif
#if defined(__MK20DX128__) || defined(__MKL26Z64__)
  #include <SPIFIFO.h>
#endif

#if !defined(ENC28J60_CONTROL_CS)
   #if defined(__AVR__) || defined(ESP8266) || defined(__RFduino__)
      // Arduino Uno (__AVR__) SS defined to pin 10
      // Arduino Leonardo (ARDUINO_AVR_LEONARDO) SS defined to LED_BUILTIN_RX (17)
      // Arduino Mega(__AVR_ATmega2560__) SS defined to pin 53
      // ESP8266 (ESP8266) SS defined to pin 15
      #if defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_MICRO)
        #define ENC28J60_CONTROL_CS     PIN_A10
        #warning "Using LEONARDO borad PIN_A10 for ENC28J60_CONTROL_CS. Use UIPEthernet::init(uint8_t) to change it."
      #else
        #define ENC28J60_CONTROL_CS     SS
      #endif
   #elif defined(ARDUINO_ARCH_AMEBA) //Defined SS to pin 10
      #define ENC28J60_CONTROL_CS     SS //PC_0 A5 10
   #elif defined(ARDUINO_ARCH_SAM)
      // Arduino Due (ARDUINO_ARCH_SAM) BOARD_SPI_DEFAULT_SS (SS3) defined to pin 78
      //#define ENC28J60_CONTROL_CS     BOARD_SPI_DEFAULT_SS
      #define ENC28J60_CONTROL_CS     BOARD_SPI_SS0
   #elif defined(ARDUINO_ARCH_SAMD)
      #define ENC28J60_CONTROL_CS     SS
   #elif defined(__ARDUINO_ARC__) //Intel ARC32 Genuino 101
      #define ENC28J60_CONTROL_CS     SS
   #elif defined(__RFduino__) //RFduino
      #define ENC28J60_CONTROL_CS     SS
   #elif defined(ARDUINO_ARCH_STM32) // STM32duino core
      #define ENC28J60_CONTROL_CS     SS
   #elif defined(ARDUINO_ARCH_ESP32) // arduino-esp32
      #define ENC28J60_CONTROL_CS     SS
   #elif defined(STM32_MCU_SERIES) || defined(__STM32F1__) || defined(__STM32F3__) || defined(STM32F3) || defined(__STM32F4__) || defined(STM32F2)
      #if defined(BOARD_SPI1_NSS_PIN)
         #define ENC28J60_CONTROL_CS     BOARD_SPI1_NSS_PIN
      #elif defined(ARDUINO_STM32F4_NETDUINO2PLUS)
         #define ENC28J60_CONTROL_CS     PC8
      #else
         #define ENC28J60_CONTROL_CS     SPI.nssPin()
         //#define ENC28J60_CONTROL_CS     PA4
      #endif
   #elif defined(__MK20DX128__) || defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
      #define ENC28J60_CONTROL_CS     PIN_SPI_SS
   #endif
#endif
#if !defined(ENC28J60_CONTROL_CS)
   #warning "Default ENC28J60_CONTROL_CS could not be defined! Use UIPEthernet::init(uint8_t) to set it."
   #define ENC28J60_CONTROL_CS 0
#endif

extern uint8_t ENC28J60ControlCS;

#if !defined(SPI_MOSI)
   #if defined(__AVR__) || defined(ESP8266) || defined(__RFduino__)
      #define SPI_MOSI MOSI
   #elif defined(ARDUINO_ARCH_AMEBA)
      #define SPI_MOSI 11 //PC_2
   #elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
      #define SPI_MOSI PIN_SPI_MOSI
   #elif defined(__ARDUINO_ARC__) //Intel ARC32 Genuino 101
      #define SPI_MOSI MOSI
   #elif defined(__RFduino__) //RFduino
      #define SPI_MOSI MOSI
   #elif defined(ARDUINO_ARCH_STM32) // STM32duino core
      #define SPI_MOSI MOSI
   #elif defined(ARDUINO_ARCH_ESP32) // arduino-esp32
      #define SPI_MOSI MOSI
   #elif defined(STM32_MCU_SERIES) || defined(__STM32F1__) || defined(__STM32F3__) || defined(STM32F3) || defined(__STM32F4__) || defined(STM32F2)
      #if defined(BOARD_SPI1_MOSI_PIN)
         #define SPI_MOSI BOARD_SPI1_MOSI_PIN
      #else
         #define SPI_MOSI SPI.mosiPin()
      #endif
   #elif defined(__MK20DX128__) || defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
      #define SPI_MOSI PIN_SPI_MOSI
   #endif
#endif
#if !defined(SPI_MOSI)
   #error "Not defined SPI_MOSI!"
#endif

#if !defined(SPI_MISO)
   #if defined(__AVR__) || defined(ESP8266) || defined(__RFduino__)
      #define SPI_MISO MISO
   #elif defined(ARDUINO_ARCH_AMEBA)
      #define SPI_MISO 12 //PC_3
   #elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
      #define SPI_MISO PIN_SPI_MISO
   #elif defined(__ARDUINO_ARC__) //Intel ARC32 Genuino 101
      #define SPI_MISO MISO
   #elif defined(__RFduino__) //RFduino
      #define SPI_MISO MISO
   #elif defined(ARDUINO_ARCH_STM32) // STM32duino core
      #define SPI_MISO MISO
   #elif defined(ARDUINO_ARCH_ESP32) // arduino-esp32
      #define SPI_MISO MISO
   #elif defined(STM32_MCU_SERIES) || defined(__STM32F1__) || defined(__STM32F3__) || defined(STM32F3) || defined(__STM32F4__) || defined(STM32F2)
      #if defined(BOARD_SPI1_MISO_PIN)
         #define SPI_MISO BOARD_SPI1_MISO_PIN
      #else
         #define SPI_MISO SPI.misoPin()
      #endif
   #elif defined(__MK20DX128__) || defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
      #define SPI_MISO PIN_SPI_MISO
   #endif
#endif
#if !defined(SPI_MISO)
   #error "Not defined SPI_MISO!"
#endif
#if !defined(SPI_SCK)
   #if defined(__AVR__) || defined(ESP8266) || defined(__RFduino__)
      #define SPI_SCK SCK
   #elif defined(ARDUINO_ARCH_AMEBA)
      #define SPI_SCK 13 //PC_1 A4
   #elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
      #define SPI_SCK PIN_SPI_SCK
   #elif defined(__ARDUINO_ARC__) //Intel ARC32 Genuino 101
      #define SPI_SCK SCK
   #elif defined(__RFduino__) //RFduino
      #define SPI_SCK SCK
   #elif defined(ARDUINO_ARCH_STM32) // STM32duino core
      #define SPI_SCK SCK
   #elif defined(ARDUINO_ARCH_ESP32) // arduino-esp32
      #define SPI_SCK SCK
   #elif defined(STM32_MCU_SERIES) || defined(__STM32F1__) || defined(__STM32F3__) || defined(STM32F3) || defined(__STM32F4__) || defined(STM32F2)
      #if defined(BOARD_SPI1_SCK_PIN)
         #define SPI_SCK BOARD_SPI1_SCK_PIN
      #else
         #define SPI_SCK SPI.sckPin()
      #endif
   #elif defined(__MK20DX128__) || defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__)
      #define SPI_SCK PIN_SPI_SCK
   #endif
#endif
#if !defined(SPI_SCK)
   #error "Not defined SPI_SCK!"
#endif

#if defined(__MBED__) || defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || defined(__ARDUINO_ARC__) || defined(__STM32F1__) || defined(__STM32F3__) || defined(STM32F3) || defined(__STM32F4__) || defined(STM32F2) || defined(ESP8266) || defined(ARDUINO_ARCH_AMEBA) || defined(__MK20DX128__) || defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1062__) || defined(__RFduino__) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
   #if defined(ARDUINO) && defined(STM32F3)
      #include "HardwareSPI.h"
   #else
      #include <SPI.h>
   #endif
   #define ENC28J60_USE_SPILIB 1
#endif

#define UIP_RECEIVEBUFFERHANDLE 0xff

/*
 * Empfangen von ip-header, arp etc...
 * wenn tcp/udp -> tcp/udp-callback -> assign new packet to connection
 */

#define TX_COLLISION_RETRY_COUNT 10

class Enc28J60Network : public MemoryPool
{

private:
  static uint16_t nextPacketPtr;
  static uint8_t bank;
  static uint8_t erevid;

  static struct memblock receivePkt;

  static bool broadcast_enabled; //!< True if broadcasts enabled (used to allow temporary disable of broadcast for DHCP or other internal functions)

  static uint8_t readOp(uint8_t op, uint8_t address);
  static void writeOp(uint8_t op, uint8_t address, uint8_t data);
  static uint16_t setReadPtr(memhandle handle, memaddress position, uint16_t len);
  static void setERXRDPT(void);
  static void readBuffer(uint16_t len, uint8_t* data);
  static void writeBuffer(uint16_t len, uint8_t* data);
  static uint8_t readByte(uint16_t addr);
  static void writeByte(uint16_t addr, uint8_t data);
  static void setBank(uint8_t address);
  static uint8_t readReg(uint8_t address);
  static void writeReg(uint8_t address, uint8_t data);
  static void writeRegPair(uint8_t address, uint16_t data);
  static void phyWrite(uint8_t address, uint16_t data);
  static uint16_t phyRead(uint8_t address);
  static void clkout(uint8_t clk);

  static void enableBroadcast (bool temporary);
  static void disableBroadcast (bool temporary);
  static void enableMulticast (void);
  static void disableMulticast (void);

  static uint8_t readRegByte (uint8_t address);
  static void writeRegByte (uint8_t address, uint8_t data);

  friend void enc28J60_mempool_block_move_callback(memaddress,memaddress,memaddress);

public:

  void powerOn(void);
  void powerOff(void);
  static uint8_t geterevid(void);
  uint16_t PhyStatus(void);
  static bool linkStatus(void);

  static void init(uint8_t* macaddr);
  static memhandle receivePacket(void);
  static void freePacket(void);
  static memaddress blockSize(memhandle handle);
  static void sendPacket(memhandle handle);
  static uint16_t readPacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len);
  static uint16_t writePacket(memhandle handle, memaddress position, uint8_t* buffer, uint16_t len);
  static void copyPacket(memhandle dest, memaddress dest_pos, memhandle src, memaddress src_pos, uint16_t len);
  static uint16_t chksum(uint16_t sum, memhandle handle, memaddress pos, uint16_t len);
};

extern Enc28J60Network Enc28J60;
#endif /* Enc28J60NetworkClass_H_ */
