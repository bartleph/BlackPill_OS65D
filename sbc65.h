// sbc65.h - memory map and I/O devices of the 65C02 SBC
#pragma once
#include <Arduino.h>
#include "config.h"
#include "rom_image.h"

// ======================================================= RAM $0000-$7FFF
static uint8_t ram[0x8000];


// ======================================================= console I/O
// Works with either Arduino core:
//  * STM32duino ("STM32 MCU based boards", recommended): USART1 on the pins in config.h
//  * Roger Clark's Arduino_STM32 / libmaple: uses its built-in Serial1 (USART1, PA9/PA10)
#if defined(ARDUINO_ARCH_STM32)
  #if CONSOLE_UART_BAUD
  static HardwareSerial ConsoleUart(CONSOLE_UART_RX, CONSOLE_UART_TX);
  #define UART_TX_ROOM()  (ConsoleUart.availableForWrite() > 0)
  #endif
  #define USB_READY()     ((bool)Serial)
#else
  #if CONSOLE_UART_BAUD
  #define ConsoleUart     Serial1
  #define UART_TX_ROOM()  true          // libmaple write() waits for buffer space itself
  #endif
  #define USB_READY()     true
#endif

static uint8_t rxbuf[256];
static uint8_t rx_head, rx_tail;
static inline bool    rx_ready(void) { return rx_head != rx_tail; }
static inline bool    rx_full(void)  { return (uint8_t)(rx_head + 1) == rx_tail; }

// requests raised by the Ctrl-] command key, handled in the main loop
static volatile uint8_t req_reset, req_nmi, req_turbo, req_status, req_disk;
static uint8_t esc_armed;

static void console_rx(uint8_t c) {
  if (esc_armed) {
    esc_armed = 0;
    switch (c) {
      case 'r': case 'R': req_reset  = 1; return;
      case 'n': case 'N': req_nmi    = 1; return;
      case 't': case 'T': req_turbo  = 1; return;
      case '?': case 's': case 'S': req_status = 1; return;
      case 'd': case 'D': req_disk   = 1; return;
      case ESC_CHAR: break;               // send a literal Ctrl-]
      default: return;
    }
  } else if (c == ESC_CHAR) { esc_armed = 1; return; }
  if (!rx_full()) { rxbuf[rx_head] = c; rx_head++; }
}

// text from the emulator itself (status messages), bypasses the ACIA
static void console_print(const char *s) {
  size_t n = strlen(s);
  if (USB_READY()) Serial.write((const uint8_t *)s, n);
#if CONSOLE_UART_BAUD
  // one byte at a time: on some cores a block write drops whatever doesn't fit
  // in the transmit buffer, while a single-byte write waits for room
  for (size_t i = 0; i < n; i++) ConsoleUart.write((uint8_t)s[i]);
#endif
}

// ======================================================= ACIA 6850 at $8000
static uint8_t acia_ctrl = 0x03;
static bool    acia_tdre = true;     // transmit register empty

static inline bool acia_irq(void) {
  if ((acia_ctrl & 0x80) && rx_ready()) return true;                 // RX IRQ enabled
  if ((acia_ctrl & 0x60) == 0x20 && acia_tdre) return true;          // TX IRQ enabled
  return false;
}
static inline uint8_t acia_status(void) {
  uint8_t s = 0;
  if (rx_ready()) s |= 0x01;          // RDRF
  if (acia_tdre)  s |= 0x02;          // TDRE   (DCD, CTS low = OK)
  if (acia_irq()) s |= 0x80;
  return s;
}
static inline uint8_t acia_read_data(void) {
  if (!rx_ready()) return rxbuf[(uint8_t)(rx_tail - 1)];
  uint8_t c = rxbuf[rx_tail]; rx_tail++;
  return c;
}
static inline void acia_update_tdre(void) {
#if CONSOLE_UART_BAUD
  acia_tdre = UART_TX_ROOM();
#else
  acia_tdre = true;
#endif
}
static inline void acia_write_data(uint8_t v) {
  if (USB_READY()) Serial.write(v);
#if CONSOLE_UART_BAUD
  ConsoleUart.write(v);
#endif
  acia_update_tdre();
}
static inline void acia_write_ctrl(uint8_t v) { acia_ctrl = v; }

static void console_poll(void) {
  while (!rx_full() && Serial.available()) console_rx((uint8_t)Serial.read());
#if CONSOLE_UART_BAUD
  while (!rx_full() && ConsoleUart.available()) console_rx((uint8_t)ConsoleUart.read());
#endif
  acia_update_tdre();
}

// ======================================================= VIA 6522 at $9000
struct Via6522 {
  uint8_t orb, ora, ddrb, ddra, t1ll, t1lh, t2ll, sr, acr, pcr, ifr, ier;
  int32_t t1c, t2c;
  bool    t1armed, t2armed;
};
static Via6522 via;

static inline void via_update_ifr(void) {
  if (via.ifr & via.ier & 0x7F) via.ifr |= 0x80; else via.ifr &= 0x7F;
}
static inline bool via_irq(void) { return via.ifr & 0x80; }

static void via_reset(void) {
  memset(&via, 0, sizeof(via));
  via.t1c = via.t2c = 0xFFFF;
}

static inline void via_tick(int n) {
  via.t1c -= n;
  if (via.t1c < 0) {
    if (via.acr & 0x40) {                         // free-running
      via.ifr |= 0x40;
      int32_t period = ((via.t1lh << 8) | via.t1ll) + 2;
      while (via.t1c < 0) via.t1c += period;
    } else {                                      // one-shot
      if (via.t1armed) { via.ifr |= 0x40; via.t1armed = false; }
      via.t1c &= 0xFFFF;
    }
    via_update_ifr();
  }
  if (!(via.acr & 0x20)) {                        // T2 timed mode (pulse counting not emulated)
    via.t2c -= n;
    if (via.t2c < 0) {
      if (via.t2armed) { via.ifr |= 0x20; via.t2armed = false; via_update_ifr(); }
      via.t2c &= 0xFFFF;
    }
  }
}

static uint8_t via_read(uint8_t r) {
  uint8_t v;
  switch (r) {
    case 0x0: via.ifr &= ~0x18; via_update_ifr();
              return (via.orb & via.ddrb) | (VIA_PB_INPUT & ~via.ddrb);
    case 0x1: via.ifr &= ~0x03; via_update_ifr();   // fall through
    case 0xF: return (via.ora & via.ddra) | (VIA_PA_INPUT & ~via.ddra);
    case 0x2: return via.ddrb;
    case 0x3: return via.ddra;
    case 0x4: via.ifr &= ~0x40; via_update_ifr(); return via.t1c & 0xFF;
    case 0x5: return (via.t1c >> 8) & 0xFF;
    case 0x6: return via.t1ll;
    case 0x7: return via.t1lh;
    case 0x8: via.ifr &= ~0x20; via_update_ifr(); return via.t2c & 0xFF;
    case 0x9: return (via.t2c >> 8) & 0xFF;
    case 0xA: via.ifr &= ~0x04; via_update_ifr(); return via.sr;
    case 0xB: return via.acr;
    case 0xC: return via.pcr;
    case 0xD: return via.ifr;
    default:  v = via.ier | 0x80; return v;         // 0xE
  }
}

static void via_write(uint8_t r, uint8_t v) {
  switch (r) {
    case 0x0: via.orb = v; via.ifr &= ~0x18; break;
    case 0x1: via.ora = v; via.ifr &= ~0x03; break;
    case 0xF: via.ora = v; break;
    case 0x2: via.ddrb = v; break;
    case 0x3: via.ddra = v; break;
    case 0x4: case 0x6: via.t1ll = v; break;
    case 0x5: via.t1lh = v; via.t1c = (via.t1lh << 8) | via.t1ll;
              via.ifr &= ~0x40; via.t1armed = true; break;
    case 0x7: via.t1lh = v; via.ifr &= ~0x40; break;
    case 0x8: via.t2ll = v; break;
    case 0x9: via.t2c = (v << 8) | via.t2ll; via.ifr &= ~0x20; via.t2armed = true; break;
    case 0xA: via.sr = v; via.ifr &= ~0x04; break;
    case 0xB: via.acr = v; break;
    case 0xC: via.pcr = v; break;
    case 0xD: via.ifr &= ~(v & 0x7F); break;
    case 0xE: if (v & 0x80) via.ier |= (v & 0x7F); else via.ier &= ~(v & 0x7F); break;
  }
  via_update_ifr();
}

// ======================================================= disk interface $C000 / $C010
static uint32_t cpu_clock;          // 65C02 cycles since power-up (wraps, only differences used)
#include "disk470.h"

// ======================================================= the bus
static inline uint8_t bus_read(uint16_t a) {
  if (a < 0x8000) return ram[a];
  switch (a >> 12) {
    case 0x8: return (a & 1) ? acia_read_data() : acia_status();
    case 0x9: return ((a & 0x0FF0) == 0) ? via_read(a & 0x0F) : OPEN_BUS;
    case 0xC: return disk_read(a);
    default:                                                         // $A,$B,$D,$E,$F
      return (a >= ROM_BASE && (uint32_t)(a - ROM_BASE) < ROM_SIZE) ? rom_image[a - ROM_BASE] : OPEN_BUS;
  }
}

static inline void bus_write(uint16_t a, uint8_t v) {
  if (a < 0x8000) { ram[a] = v; return; }
  switch (a >> 12) {
    case 0x8: if (a & 1) acia_write_data(v); else acia_write_ctrl(v); break;
    case 0x9: if ((a & 0x0FF0) == 0) via_write(a & 0x0F, v); break;
    case 0xC: disk_write(a, v); break;
    default:  break;                                // ROM: writes ignored
  }
}

#include "cpu65c02.h"

// /RES on the real board resets the CPU, VIA and PIA (the ACIA has no reset pin)
static void machine_reset(void) {
  via_reset();
  disk_reset();
  cpu_reset();
}
