// SBC65_Emu - emulates the 65C02 SBC (OS-65D ROM DOS) on a Black Pill STM32F401
//
// Builds with Roger Clark's Arduino_STM32 (libmaple) core or the STM32duino core.
// See README.md for board settings and wiring.
//
// Console: USB serial (any baud) and USART1 on PA9 (TX) / PA10 (RX), TTL levels.
// Disks:   micro SD adapter on SPI1 (CS PA4, SCK PA5, MISO PA6, MOSI PA7),
//          Gotek HFE images in the card's root directory.
// Ctrl-] then R/N/T/D/? = reset / NMI / turbo / disk menu / status.
// KEY button (PA0) = 65C02 RESET, RAM is kept like the real machine.
//
// Put your 32K EEPROM image into rom_image.h with tools/bin2h first.

#include "config.h"
#include "sbc65.h"

static bool     turbo = START_TURBO;
static bool     held_in_reset;
static uint8_t  nmi_pin_last = 1;
static uint64_t pace_acc;                // cycles*1e6 not yet converted to microseconds
static uint32_t pace_t0;
static uint32_t stat_cycles, stat_t0, stat_hz;

// ---------------------------------------------------------------- console UART pins
// On the STM32F4, USB set-up can take PA10 as the USB "ID" pin, and some cores
// leave the USART RX pin as a plain input. Either way USART1 never sees the
// terminal's keystrokes. This hands PA9 / PA10 back to USART1 (alternate function 7).
#define UREG(a) (*(volatile uint32_t *)(a))
#define U_GPIOA_MODER UREG(0x40020000)
#define U_GPIOA_PUPDR UREG(0x4002000C)
#define U_GPIOA_IDR   UREG(0x40020010)
#define U_GPIOA_AFRH  UREG(0x40020024)
#define U_USART1_SR   UREG(0x40011000)
#define U_USART1_BRR  UREG(0x40011008)
#define U_USART1_CR1  UREG(0x4001100C)
#define U_RCC_PLLCFGR UREG(0x40023804)
#define U_RCC_CFGR    UREG(0x40023808)

// The chip's real clocks, worked out from the RCC registers (not from what the core assumes)
static uint32_t real_sysclk(void) {
  uint32_t sws = (U_RCC_CFGR >> 2) & 3;
  if (sws == 0) return 16000000UL;                                 // HSI
  if (sws == 1) return BOARD_HSE_HZ;                               // HSE
  uint32_t pll = U_RCC_PLLCFGR;
  uint32_t src = (pll & (1u << 22)) ? BOARD_HSE_HZ : 16000000UL;
  uint32_t m = pll & 0x3F, n = (pll >> 6) & 0x1FF, p = (((pll >> 16) & 3) + 1) * 2;
  if (!m) return 0;
  return (uint32_t)((uint64_t)src / m * n / p);
}
static uint32_t real_pclk2(void) {
  static const uint16_t ahb_div[16] = {1,1,1,1,1,1,1,1,2,4,8,16,64,128,256,512};
  uint32_t cfgr = U_RCC_CFGR;
  uint32_t hclk = real_sysclk() / ahb_div[(cfgr >> 4) & 15];
  uint32_t ppre2 = (cfgr >> 13) & 7;
  return ppre2 < 4 ? hclk : hclk >> (ppre2 - 3);
}
static void uart_set_baud(uint32_t baud) {
  uint32_t f = real_pclk2();
  if (!f || !baud) return;
  uint32_t cr1 = U_USART1_CR1;
  U_USART1_CR1 = cr1 & ~(1u << 13);                                // UE off while changing
  if (cr1 & (1u << 15)) {                                          // OVER8
    uint32_t d = (2 * f + baud / 2) / baud;
    U_USART1_BRR = (d & ~0xFu) | ((d & 0xFu) >> 1);
  } else {
    U_USART1_BRR = (f + baud / 2) / baud;
  }
  U_USART1_CR1 = cr1;
}

static void uart_claim_pins(void) {
#if CONSOLE_UART_BAUD && !defined(SD_HOST_TEST)
  U_GPIOA_AFRH  = (U_GPIOA_AFRH & ~(0xFFu << 4)) | (7u << 4) | (7u << 8);      // PA9, PA10 = AF7
  U_GPIOA_PUPDR = (U_GPIOA_PUPDR & ~(0xFu << 18)) | (1u << 20);                // pull-up on PA10
  U_GPIOA_MODER = (U_GPIOA_MODER & ~(0xFu << 18)) | (2u << 18) | (2u << 20);   // alternate function
  uart_set_baud(CONSOLE_UART_BAUD);
#endif
}

static void print_uart_pins(void) {
#if CONSOLE_UART_BAUD && !defined(SD_HOST_TEST)
  char buf[100];
  static const char *modes[4] = {"input", "output", "alt", "analog"};
  uint32_t m = (U_GPIOA_MODER >> 20) & 3, af = (U_GPIOA_AFRH >> 8) & 15;
  snprintf(buf, sizeof(buf), "  UART RX PA10: %s AF%lu, level %lu, USART1 SR %02lX%s\r\n",
           modes[m], (unsigned long)af, (unsigned long)((U_GPIOA_IDR >> 10) & 1),
           (unsigned long)(U_USART1_SR & 0xFF),
           (m == 2 && af == 7) ? "" : "  <-- not connected to USART1");
  console_print(buf);
  uint32_t brr = U_USART1_BRR & 0xFFFF, f = real_pclk2();
  uint32_t actual = brr ? (uint32_t)((uint64_t)f * ((U_USART1_CR1 & (1u << 15)) ? 2 : 1) /
                    (((U_USART1_CR1 & (1u << 15)) ? ((brr & ~0xFu) | ((brr & 7u) << 1)) : brr))) : 0;
  snprintf(buf, sizeof(buf), "  UART baud %lu (asked %lu), CPU %lu MHz, APB2 %lu MHz\r\n",
           (unsigned long)actual, (unsigned long)CONSOLE_UART_BAUD,
           (unsigned long)(real_sysclk() / 1000000UL), (unsigned long)(f / 1000000UL));
  console_print(buf);
#endif
}

static void print_drives(void) {
  char buf[96];
  for (int d = 0; d < NDRIVES; d++) {
    snprintf(buf, sizeof(buf), "  %c: %-30s %s track %u\r\n", 'A' + d,
             drv[d].mounted ? drv[d].name : "(empty)",
             drv[d].mounted && drv[d].wp ? "[protected]" : "           ",
             drv[d].head);
    console_print(buf);
  }
  if (disk_error) { console_print("  SD: "); console_print(disk_error); console_print("\r\n"); }
}

static void print_status(void) {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "\r\n[SBC65 PC=%04X A=%02X X=%02X Y=%02X S=%02X P=%02X  %lu.%02lu MHz  %s%s]\r\n",
           cpu.pc, cpu.a, cpu.x, cpu.y, cpu.s, cpu.p,
           (unsigned long)(stat_hz / 1000000UL), (unsigned long)((stat_hz / 10000UL) % 100),
           turbo ? "turbo" : "real speed",
           cpu.stopped ? "  STOPPED (STP)" : (cpu.waiting ? "  WAI" : ""));
  console_print(buf);
  print_uart_pins();
  print_drives();
}

// ---------------------------------------------------------------- disk menu
static int menu_key(void) {                        // blocking read from USB or UART
  for (;;) {
    if (Serial.available()) return Serial.read();
#if CONSOLE_UART_BAUD
    if (ConsoleUart.available()) return ConsoleUart.read();
#endif
  }
}
static void menu_echo(char c) { char s[2] = {c, 0}; console_print(s); }

struct MenuList { int n; char names[32][40]; };
static MenuList menu_list;
static bool menu_list_cb(const FatEntry &e, void *ctx) {
  MenuList *m = (MenuList *)ctx;
  if (!(e.attr & 0x10) && name_has_ext(e.name, ".HFE")) {
    strncpy(m->names[m->n], e.name, 39); m->names[m->n][39] = 0;
    m->n++;
  }
  return m->n < 32;
}

static void disk_menu(void) {
  char buf[96];
  disk_flush_all();
  if (!fat_mount()) disk_error = fat_fail;
  else disk_error = NULL;
  menu_list.n = 0;
  fat_scan_root(menu_list_cb, &menu_list);
  for (;;) {
    console_print("\r\n[SBC65 disks]\r\n");
    print_drives();
    console_print("Images on the SD card:\r\n");
    for (int i = 0; i < menu_list.n; i++) {
      snprintf(buf, sizeof(buf), "  %2d %s\r\n", i + 1, menu_list.names[i]);
      console_print(buf);
    }
    console_print("Drive letter to change (Enter = back to the 65C02): ");
    int c = menu_key();
    if (c == '\r' || c == '\n' || c == 0x1B) break;
    int d = toupper(c) - 'A';
    if (d < 0 || d >= NDRIVES) continue;
    menu_echo((char)toupper(c));
    console_print("\r\nImage number (0 = eject): ");
    int num = 0;
    for (;;) {
      c = menu_key();
      if (c >= '0' && c <= '9') { num = num * 10 + (c - '0'); menu_echo((char)c); }
      else if (c == '\r' || c == '\n') break;
      else if (c == 0x1B) { num = -1; break; }
    }
    if (num == 0) drive_eject(d);
    else if (num > 0 && num <= menu_list.n) {
      const char *why;
      if (!drive_mount(d, menu_list.names[num - 1], &why)) {
        console_print("\r\n  can't mount: "); console_print(why);
      }
    }
  }
  console_print("\r\n");
  pace_acc = 0; pace_t0 = micros();               // don't try to catch up the time spent here
}

// ---------------------------------------------------------------- main loop
static void handle_requests(void) {
  if (req_reset)  { req_reset = 0; machine_reset(); console_print("\r\n[SBC65 reset]\r\n"); }
  if (req_nmi)    { req_nmi = 0; cpu.nmi = 1; }
  if (req_turbo)  { req_turbo = 0; turbo = !turbo; pace_acc = 0; pace_t0 = micros();
                    console_print(turbo ? "\r\n[SBC65 turbo on]\r\n" : "\r\n[SBC65 real speed]\r\n"); }
  if (req_status) { req_status = 0; print_status(); }
  if (req_disk)   { req_disk = 0; disk_menu(); }
}

static void poll_pins(void) {
  bool rst = digitalRead(RESET_PIN) == LOW;
  if (rst) held_in_reset = true;
  else if (held_in_reset) { held_in_reset = false; machine_reset(); }

  uint8_t n = digitalRead(NMI_PIN);
  if (nmi_pin_last && !n) cpu.nmi = 1;           // falling edge
  nmi_pin_last = n;
}

// LED = disk activity: lit while the DOS is reading or writing disk data
static void update_led(void) {
  bool on = (uint32_t)(millis() - disk_activity_ms) < 50 && disk_activity_ms != 0;
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

void setup(void) {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(NMI_PIN, INPUT_PULLUP);
  Serial.begin(115200);                          // USB CDC, baud is ignored
#if CONSOLE_UART_BAUD
  ConsoleUart.begin(CONSOLE_UART_BAUD);
#endif
  uart_claim_pins();                             // after USB and UART set-up
#if CLEAR_RAM_AT_POWERUP
  memset(ram, 0, sizeof(ram));
#endif
  disk_mount_all();                              // SD card -> drives A-D
  machine_reset();
  pace_t0 = stat_t0 = micros();
}

void loop(void) {
  console_poll();
  poll_pins();
  handle_requests();
  update_led();

  if (held_in_reset) { delay(1); return; }

  int32_t budget = SLICE_CYCLES;
  while (budget > 0) {
    cpu.irq = acia_irq() || via_irq();
    int n = cpu_step();
    via_tick(n);
    cpu_clock += n;
    budget -= n;
  }
  disk_sync();
  uint32_t ran = SLICE_CYCLES - budget;
  stat_cycles += ran;

  uint32_t now = micros();
  if ((uint32_t)(now - stat_t0) >= 1000000UL) {
    stat_hz = (uint32_t)((uint64_t)stat_cycles * 1000000ULL / (uint32_t)(now - stat_t0));
    stat_cycles = 0; stat_t0 = now;
  }

  if (!turbo) {                                  // hold the 65C02 to CPU_HZ
    pace_acc += (uint64_t)ran * 1000000ULL;
    uint32_t owed = (uint32_t)(pace_acc / CPU_HZ);
    pace_acc -= (uint64_t)owed * CPU_HZ;
    pace_t0 += owed;
    if ((int32_t)(micros() - pace_t0) > 50000) pace_t0 = micros();   // too far behind: resync
    while ((int32_t)(micros() - pace_t0) < 0) console_poll();
  }
}
