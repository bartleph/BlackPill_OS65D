// config.h - settings for the SBC65 emulator (Black Pill STM32F401CC / CE)
#pragma once

// ---------------- CPU speed ----------------
#define CPU_HZ            1000000UL  // emulated 65C02 clock (your SBC runs at 1 MHz)
#define START_TURBO       0          // 1 = start flat out instead of at CPU_HZ
#define SLICE_CYCLES      1000       // 65C02 cycles run between I/O polls (1 ms at 1 MHz)

// ---------------- Console ACIA ($8000) ----------------
// Output goes to USB serial AND the UART pins; input is taken from both.
// Set CONSOLE_UART_BAUD to match your VT100 / ESP32 bridge.
// While the UART is enabled, console output is paced to its baud rate,
// just like the real ACIA. Set it to 0 for USB only at full speed.
#define CONSOLE_UART_BAUD 19200
#define CONSOLE_UART_RX   PA10       // USART1 RX  (connect to terminal TX)
#define CONSOLE_UART_TX   PA9        // USART1 TX  (connect to terminal RX)
#define BOARD_HSE_HZ      25000000UL // crystal on the board: 25 MHz on the WeAct Black Pill F401/F411
                                     // (used to set the UART baud from the chip's real clock)

// Emulator command key: press Ctrl-] then
//   R = reset 65C02   N = NMI   T = toggle turbo   ? = status
//   Ctrl-] again sends a real Ctrl-] to the 65C02
#define ESC_CHAR          0x1D

// ---------------- Pins ----------------
#define RESET_PIN         PA0        // active low; the Black Pill KEY button is on PA0
                                     // (also wire your ESP32 remote-reset output here)
#define NMI_PIN           PA1        // active low, falling edge = NMI
#define LED_PIN           PC13       // on-board LED, active low

// ---------------- Memory map ----------------
// $0000-$7FFF RAM 32K
// $8000-$8FFF console ACIA (6850), mirrored on A0
// $9000-$900F VIA (6522); $9010-$9FFF unmapped
// $A000-$BFFF ROM (Forth)
// $C000-$C003 disk PIA (6821), $C010-$C011 disk ACIA: drives A-D on the SD card
// $D000-$FFFF ROM (DOS extension + DOS)
// The ROM image comes from rom_image.h (made by tools/bin2h from your EEPROM .bin)
#define OPEN_BUS          0xFF       // value read from unmapped addresses
#define CLEAR_RAM_AT_POWERUP 1

// ---------------- VIA / PIA idle inputs ----------------
#define VIA_PA_INPUT      0xFF       // what VIA port pins set as inputs read
#define VIA_PB_INPUT      0xFF

// ---------------- Disk drives (SD card on SPI1: CS PA4, SCK PA5, MISO PA6, MOSI PA7) ----------------
// Gotek HFE images in the card's root directory. SBC65.CFG on the card picks
// them (lines like  A=PBDOS.HFE ); without it the first four .HFE files go in A-D.
// Ctrl-] D changes them while running.
#define DISK_BIT_RATE     125000UL   // OSI 5.25" data rate
#define DISK_INDEX_BITS   250        // index pulse length in bit times (2 ms)
#define DISK_FLUSH_CYCLES 500000UL   // write a changed track back after 0.5 s untouched
// Which drive the 74LS138 selects for each PA6/PB5 combination,
// in the order {PA6=0 PB5=0, PA6=0 PB5=1, PA6=1 PB5=0, PA6=1 PB5=1}; 0=A 1=B 2=C 3=D.
// Stock OSI: PA6=1 is drive 1, PB5=0 picks the second pair.
#define DRIVE_SELECT_MAP  {3, 1, 2, 0}
