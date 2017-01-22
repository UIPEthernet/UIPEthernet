# UIPEthernet
UIPEthernet library for Arduinos (Atmel AVR-s,Atmel SAM3X8E ARM Cortex-M3,STM32F series,ESP8266), ENC28j60 network chip compatible with Wiznet W5100 API

Original UIPEthernet writed by Norbert Truchsess.

You can find wiring diagram for more board in the hardware directory.

Modifications:
- Replaced import to include, because gcc say 'import is deprecated'.
- Added support for STM32F, and ESP8266 MCU-s.
- Merged martinayotte's modification (Correct s_dhcp ~40K more memory usage with STM32F MCU-s.)
- Moved htons,ntohs,htonl,ntohl definitions to uip.h.
- Corrected infinite loops.
- Set the version to 2.0.1
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
- Fxing errata 12 by seydamir.
