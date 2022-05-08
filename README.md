# UIPEthernet
UIPEthernet library for Arduinos (Atmel AVR-s,Atmel SAM3X8E ARM Cortex-M3,STM32F series,ESP8266,Intel ARC32(Genuino101),Nordic nRF51(RFduino),Teensy boards,Realtek Ameba(RTL8195A,RTL8710)), ENC28j60 network chip compatible with Wiznet W5100 API

Original UIPEthernet writed by Norbert Truchsess.

For new projects with enc28j60 EthernetENC library is recommended.

You can find wiring diagram for more board in the hardware directory.

Modifications:
- Replaced import to include, because gcc say 'import is deprecated'.
- Added support for STM32F, and ESP8266 MCU-s.
- Merged martinayotte's modification (Correct s_dhcp ~40K more memory usage with STM32F MCU-s.)
- Moved htons,ntohs,htonl,ntohl definitions to uip.h.
- Corrected infinite loops.
- Set the version to 2.0.3
- Corrected ESP8266 exception(28).
- Added watchdog reset calls in functions for stable running on ESP8266.
- Added geterevid function to get ENC28j60 chip erevid (revision information).
- Changed linkStatus to static for outside call.
- Added functions bypass, if can't communicate with ethernet device.
- Changed debuging/logging. Remove individual debuging. Add global and scalable debuging feature.
You can setup debuging/logging level in utility/logging.h
You can use this header file in Your scetch too.
Add "LogObject" define for serial logging/debuging with board specific default setting.
- Added support to MBED/SMeshStudio IDE. (Compiled and tested on Nucleo-F302R8. (STM32F302R8))

- Added Abstract Print class to MBED for full compatibility (Can use print, println with uip objects.)
- Errata#12 corrected (by seydamir).
- Created v2.0.2 release.

If You use NodeMCU please check wiring first:
https://github.com/UIPEthernet/UIPEthernet/blob/master/hardware/NodeMCU_enc28j60_wiring.PNG

- You can save 5K flash if you disable UDP support.
- Correction code of Errata#12 modified.
- Added support for Intel ARC32(Genuino101), Nordic nRF51(RFduino), Teensy boards
- Issue#4 corrected
- Added support for Realtek Ameba(RTL8195A,RTL8710)
- Added direct broadcast support
- Issue#5 corrected: You can save 5K flash memory with disable UDP support.
- Issue#6 corrected: Added support Eclipse with arduino plugin
- Issue#8, and Issue#9 corrected: Modified DHCP code: Moved timeouts define to dhcp.h
- Issue#11 corrected: Changed ENC28J60_CONTROL_CS pin to 10 on Arduino Due
- New release:2.0.4

- Added support for Arduino_Core_STM32.
- Endianness configuration/detection changed.
- Added support for adafruit wiced feather.
- Arduino Mega2560 upload error (3 !!!) corrected.
- Added setup SS pin to output.
- New release:2.0.5

- Added support ESP32 and SAMD
- New release:2.0.6

- bug fixes (jandrassy)
- Ethernet lib compatibility: `init(cs_pin)` (by phd) and `linkStatus`
- ARP Table improvement by dimitar-kunchev
- UIPServer esp32 core 1.0.1 compilation compatibility (jandrassy)
- uipethernet-conf.h options can be specified as -D (jandrassy)
- New release:2.0.7

- Fix local UDP broadcast address evaluation with ~ operator on 32bit systems (pknoe3lh)
- Re-fix Errata 14 (Bigpet)
- ENC28J60_USE_SPILIB for ARDUINO_ARCH_AVR and ARDUINO_ARCH_MEGAAVR (jandrassy)
- Add support for Teensy4 boards (__IMXRT1062__) (rjongeneelen)
- `EthernetServer.accept()` implementation (jandrassy)
- compilation with logging enabled fixes for megaavr core (jandrassy)
- stm32 blue pill wiring picture corrected (jandrassy)
- New release:2.0.8

- major bug fix - wrong calculation of packet position wrapped on RXSTOP
- Enc28J60Network: reserve space for TSV. coded by N.Truchsess in ntruchsess/arduino_uip#103
- solved problem with STM32F3 define in ARDUINO_ARCH_STM32
- sendPacket: fixed Errata 13 workaround and send result returned
- UIPClient - fix of the 'overlay' struct uip_userdata_closed_t
- Enc28J60 object removed. all functions in Enc28J60Network were static
- uip_timer, uip_clock, clock-arch and uip_debug were not used. removed
- Ethernet.tick() - polling for active connection was too frequent. in UIP_CLIENT_TIMER mode (it is default) an active connection was polled every tick()
- UIPClient - added getters remoteIP() and remotePort()
- UIPClient - implementation of availableForWrite()
- UIPClient - sending next packet right after ACK boost the performance
- UIPClient - aborted was not handled
- UIPClient - _write() looped forever if connection timed out
- UIPClient - UIP_WRITE_TIMEOUT instead of UIP_ATTEMPTS_ON_WRITE
- UIPServer - added operator bool() and function end()
- UIPEthernet - added hardwareStatus() (Ethernet 2.00 compatibility)
- Ethernet.h file for super libraries including Ethernet.h
- New release:2.0.9

- make hardwareStatus() and linkStatus() work without Erhernet.begin(mac)
- UIPServer.end() - stop() all clients
- fixed library.json for platformio
- New release:2.0.10

- UIPUdp.cpp beginPacket fix IPAddress check for esp8266 core 3.x.x
- platformio library.json fixes by Ivan Kravets
- UIPClient._allocateData - fix data clear
- UIPClient - fix NULL data field checks
- New release:2.0.11

- Revert "UIPClient - fix NULL data field checks"
- UIPClient - process incoming data even when remote closed
- restart UDP connection also when logging is off (by Karl Õmblus)
- New release:2.0.12
 