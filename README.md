# logicanalyzer

Hardware features:
- This is a DIY hobby project to build a simple logic analyzer.
- Firmware is done in C for an Atmel AVR Atmega32 controller.
- Cheap cache sram circuits from old 386 mainboards are used for fast sample snapshot storage.
- Prototype was build from scratch using wire wrap board with through-hole components.

Firmware features:
- Samplerate can be selected by user (divided from external crystal oscillator or manually switched).
- Pre-trigger and post-trigger length can be selected.
- Simple trigger conditions can be used.
- UART transport protection by CRC-16

Control software features:
- Serial communication library
- Set trigger and samplerate
- Get full snapshot ram content
- Verify correct serial transport by CRC-16
- Store logic analyter data in .VCD file for later viewing by e.g. GTKwave
