# SBC65_Emu — HobbyPCB.co.uk 65C02 SBC on a Black Pill (STM32F401CC / CE)

Emulates the SBC: 1 MHz 65C02, 32K RAM, console ACIA at $8000, VIA at $9000,
a full 32K EEPROM image (Forth at $A000, DOS extension at $D000, DOS at $E000),
and the OSI disk interface (PIA $C000, ACIA $C010) with four 5.25" drives A–D
that use Gotek HFE images on a micro SD card.
<img width="383" height="192" alt="image" src="https://github.com/user-attachments/assets/619c7e1f-90c1-4585-b026-508e6b95e9db" />

## Files

| File | What it is |
|---|---|
| `SBC65_Emu/SBC65_Emu.ino` | main loop, speed pacing, reset/NMI pins, Ctrl-] commands, disk menu |
| `SBC65_Emu/config.h` | all settings: baud, pins, speed, drive select map |
| `SBC65_Emu/sbc65.h` | memory map and devices (RAM, console ACIA, VIA, ROM) |
| `SBC65_Emu/disk470.h` | disk interface: PIA, disk ACIA, stepping, HFE track read/write |
| `SBC65_Emu/fat.h`, `sdcard.h` | SD card (bit-banged on PA4-PA7) and FAT16/FAT32 reader |
| `SBC65_Emu/cpu65c02.h` | WDC 65C02 core (passes Klaus Dormann's 6502 and 65C02 tests) |
| `SBC65_Emu/rom_image.h` | the ROM.  |


## 1. ROM

rom_image.h is built from the SBC's EEPROM.

The 32K file is placed at $8000-$FFFF, so file offset 0 = $8000

## 2. Arduino IDE

The sketch builds with either core:

* **Roger Clark's Arduino_STM32** (what you have): pick your F4 / Black Pill board.
  The console UART is that core's `Serial1` (USART1, PA9/PA10).
* **STM32duino** ("STM32 MCU based boards"): Board *Generic STM32F4 series*,
  part number *BlackPill F401CC*, USB support *CDC (generic 'Serial' supersede U(S)ART)*,
  Optimize *Faster (-O2)*.

The SD card is driven directly on its four pins, so no SD library is needed.

## 3. Wiring

| Black Pill | Use |
|---|---|
| USB-C | console as a USB serial port (PuTTY etc., any baud) |
| PA9 (TX) / PA10 (RX) / GND | TTL console UART, same as your SBC header |
| PA0 (KEY button) | 65C02 RESET, active low. Wire the ESP32 remote-reset output here too |
| PA1 | NMI, active low (falling edge) |
| PC13 LED | disk activity: lit while the DOS reads or writes disk data |
| **SD module** CS → PA4, CLK → PA5, DO (MISO) → PA6, DI (MOSI) → PA7 | the disk drives |
| SD module 3.3V → 3V3, GND → GND | leave its 5V and CD pins unconnected |

The KEY button resets only the 65C02, VIA and PIA, so RAM survives like the real
machine. The NRST button restarts the whole emulator and clears RAM.

## 4. The SD card

* Format it **FAT32** (or FAT16) and copy your `.HFE` images to the **root folder**.
  The same files work in the Gotek, and back again.
* Choose which image is in which drive with a text file `SBC65.CFG` in the root:
  ```
  A=PBdos.HFE
  B=Work disk.hfe
  ```
  Long file names are fine. Without `SBC65.CFG` the first four `.HFE` files go into A–D.
* **Ctrl-] D** opens a menu to swap images while running (and re-reads the card,
  so you can change cards first).
* When the DOS writes, the changed track is written back into the HFE file in place
  after the head is lifted, or 0.5 s after the last write. Files never change size.
* An image whose read-only attribute is set (or whose HFE header disallows writes)
  shows as write-protected to the DOS.
* If a mount says "too fragmented", copy that file to the card again (or onto a
  freshly formatted card).

## 5. Console command key: Ctrl-]

| Keys | Action |
|---|---|
| Ctrl-] R | reset the 65C02 |
| Ctrl-] N | NMI |
| Ctrl-] T | toggle turbo (flat out) / real 1 MHz |
| Ctrl-] D | disk menu: show drives, mount / eject images |
| Ctrl-] ? | status: registers, measured MHz, drives and head positions |
| Ctrl-] Ctrl-] | send a real Ctrl-] to the 65C02 |

## 6. Settings worth knowing (`config.h`)

* `CONSOLE_UART_BAUD 9600` — match your VT100 or ESP32 bridge; `0` = USB only, full speed.
* `CPU_HZ` / `START_TURBO` — disk timing follows the emulated clock, so disks
  also work in turbo mode.
* `DRIVE_SELECT_MAP {3, 1, 2, 0}` — which drive your 74LS138 selects for each
  PA6/PB5 combination. The default is the stock OSI arrangement (PA6=1 is drive A,
  PB5=0 picks the C/D pair). If `SE B` reaches the wrong image, run your `PX`
  command on the real machine and swap the numbers to match.

## How the disk works

The HFE tracks hold FM bit cells. When the head loads on a track, it is decoded
to the 125 kbit/s serial stream the disk ACIA sees (8E1, 8 µs per bit at 1 MHz).
The bits are delivered in step with the 65C02 clock, with index pulse, track-0,
ready and write-protect on the PIA as the OSI routines expect. Writes go into the
same bit stream while PB0 (write gate) is low and are encoded back to FM.

## Notes and limits

* Side 0 only (OSI 5.25" disks are single-sided); side 1 data in the file is kept.
* VIA: registers, T1 (one-shot and free-run) and T2 timed mode with IRQs are
  emulated. Ports read `VIA_PA_INPUT`/`VIA_PB_INPUT`; nothing is wired to pins.
* $9010-$9FFF reads `OPEN_BUS` ($FF). Writes to ROM areas are ignored.
