// sdcard.h - micro SD card in SPI mode, bit-banged on GPIO pins
//
//   CS   PA4      SCK  PA5      MISO PA6 (module "DO")      MOSI PA7 (module "DI")
//
// The pins are set up with the core's pinMode(), so this works the same on
// Roger Clark's core and on STM32duino. Bits are then clocked through the
// GPIOA port registers (several MHz). If the card does not answer that way,
// sd_init() falls back to plain digitalWrite()/digitalRead(), which is slow
// but works on anything.
//
// Provides: sd_init(), sd_read(lba, buf), sd_write(lba, buf)  (512-byte blocks)
#pragma once
#include <Arduino.h>

#ifndef SD_HOST_TEST   // ------------------------------------------------ STM32F4

#define SD_PIN_CS   PA4
#define SD_PIN_SCK  PA5
#define SD_PIN_MISO PA6
#define SD_PIN_MOSI PA7
#define SDR_GPIOA_IDR  (*(volatile uint32_t *)0x40020010)
#define SDR_GPIOA_BSRR (*(volatile uint32_t *)0x40020018)
#define SDR_GPIOA_OSPEEDR (*(volatile uint32_t *)0x40020008)

static bool sd_fast_io = true;              // GPIOA registers (false = digitalWrite)
static bool sd_slow = true;                 // card initialisation: keep the clock < 400 kHz

static inline void sd_cs_high(void) {
  if (sd_fast_io) SDR_GPIOA_BSRR = 1u << 4; else digitalWrite(SD_PIN_CS, HIGH);
}
static inline void sd_cs_low(void) {
  if (sd_fast_io) SDR_GPIOA_BSRR = 1u << (4 + 16); else digitalWrite(SD_PIN_CS, LOW);
}

static void sd_spi_setup(void) {
  pinMode(SD_PIN_CS, OUTPUT);   digitalWrite(SD_PIN_CS, HIGH);
  pinMode(SD_PIN_SCK, OUTPUT);  digitalWrite(SD_PIN_SCK, LOW);
  pinMode(SD_PIN_MOSI, OUTPUT); digitalWrite(SD_PIN_MOSI, HIGH);
  pinMode(SD_PIN_MISO, INPUT_PULLUP);
  SDR_GPIOA_OSPEEDR |= 0xFFu << 8;                     // fast edges on PA4-PA7
  sd_slow = true;
}

static uint8_t sd_xfer(uint8_t v) {
  uint8_t r = 0;
  if (sd_fast_io && !sd_slow) {                        // full speed, mode 0, MSB first
    for (int i = 0; i < 8; i++) {
      SDR_GPIOA_BSRR = (v & 0x80) ? (1u << 7) : (1u << (7 + 16));
      v <<= 1;
      SDR_GPIOA_BSRR = 1u << 5;                        // SCK high: card samples MOSI
      __asm__ volatile("nop\n nop\n nop\n nop");
      r = (uint8_t)((r << 1) | ((SDR_GPIOA_IDR >> 6) & 1));
      SDR_GPIOA_BSRR = 1u << (5 + 16);                 // SCK low: card shifts out next bit
      __asm__ volatile("nop\n nop");
    }
  } else if (sd_fast_io) {                             // init speed (~150 kHz)
    for (int i = 0; i < 8; i++) {
      SDR_GPIOA_BSRR = (v & 0x80) ? (1u << 7) : (1u << (7 + 16));
      v <<= 1;
      delayMicroseconds(3);
      SDR_GPIOA_BSRR = 1u << 5;
      delayMicroseconds(3);
      r = (uint8_t)((r << 1) | ((SDR_GPIOA_IDR >> 6) & 1));
      SDR_GPIOA_BSRR = 1u << (5 + 16);
    }
  } else {                                             // fallback: core pin functions
    for (int i = 0; i < 8; i++) {
      digitalWrite(SD_PIN_MOSI, (v & 0x80) ? HIGH : LOW);
      v <<= 1;
      if (sd_slow) delayMicroseconds(3);
      digitalWrite(SD_PIN_SCK, HIGH);
      if (sd_slow) delayMicroseconds(3);
      r = (uint8_t)((r << 1) | (digitalRead(SD_PIN_MISO) ? 1 : 0));
      digitalWrite(SD_PIN_SCK, LOW);
    }
  }
  return r;
}

static bool sd_hc;                          // SDHC/SDXC: block addressing

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg) {
  uint8_t crc = (cmd == 0) ? 0x95 : (cmd == 8) ? 0x87 : 0x01;
  sd_cs_high(); sd_xfer(0xFF);
  sd_cs_low();  sd_xfer(0xFF);
  sd_xfer(0x40 | cmd);
  sd_xfer(arg >> 24); sd_xfer(arg >> 16); sd_xfer(arg >> 8); sd_xfer(arg);
  sd_xfer(crc);
  uint8_t r;
  for (int i = 0; i < 10; i++) { r = sd_xfer(0xFF); if (!(r & 0x80)) return r; }
  return 0xFF;
}

static const char *sd_fail = "";             // why the last sd_init failed

static bool sd_init_try(void) {
  static bool powered;
  if (!powered) { delay(300); powered = true; }        // let the card power up
  sd_spi_setup();
  sd_cs_high();
  for (int i = 0; i < 12; i++) sd_xfer(0xFF);         // >= 74 clocks with CS high
  uint8_t r = 0xFF;
  for (int i = 0; i < 20 && r != 0x01; i++) r = sd_cmd(0, 0);
  if (r != 0x01) {
    sd_cs_high();
    sd_fail = (r == 0xFF) ? "card not answering - check CS/SCK/MOSI/MISO wiring and VCC"
            : (r == 0x00) ? "MISO stuck low - check the MISO wire"
            : "card gave a bad reply to reset";
    return false;
  }
  bool v2 = false;
  if (sd_cmd(8, 0x1AA) == 0x01) {
    uint8_t b[4]; for (int i = 0; i < 4; i++) b[i] = sd_xfer(0xFF);
    v2 = (b[2] == 0x01 && b[3] == 0xAA);
  }
  uint32_t t0 = millis();
  do {
    sd_cmd(55, 0);
    r = sd_cmd(41, v2 ? 0x40000000UL : 0);
    if ((uint32_t)(millis() - t0) > 2000) { sd_cs_high(); sd_fail = "card answered but would not initialise"; return false; }
  } while (r != 0x00);
  sd_hc = false;
  if (v2 && sd_cmd(58, 0) == 0x00) {
    uint8_t ocr = sd_xfer(0xFF); sd_xfer(0xFF); sd_xfer(0xFF); sd_xfer(0xFF);
    sd_hc = (ocr & 0x40) != 0;
  }
  if (!sd_hc) sd_cmd(16, 512);
  sd_cs_high(); sd_xfer(0xFF);
  sd_slow = false;
  return true;
}

static bool sd_init(void) {
  sd_fast_io = true;
  if (sd_init_try()) return true;
  const char *why = sd_fail;
  sd_fast_io = false;                                  // try again with digitalWrite()
  if (sd_init_try()) return true;
  sd_fail = why;
  return false;
}

static bool sd_wait_ready(uint32_t ms) {
  uint32_t t0 = millis();
  while (sd_xfer(0xFF) != 0xFF) if ((uint32_t)(millis() - t0) > ms) return false;
  return true;
}

static bool sd_read(uint32_t lba, uint8_t *buf) {
  if (sd_cmd(17, sd_hc ? lba : lba * 512) != 0x00) { sd_cs_high(); return false; }
  uint32_t t0 = millis(); uint8_t tok;
  while ((tok = sd_xfer(0xFF)) == 0xFF) if ((uint32_t)(millis() - t0) > 300) break;
  if (tok != 0xFE) { sd_cs_high(); return false; }
  for (int i = 0; i < 512; i++) buf[i] = sd_xfer(0xFF);
  sd_xfer(0xFF); sd_xfer(0xFF);                         // CRC
  sd_cs_high(); sd_xfer(0xFF);
  return true;
}

static bool sd_write(uint32_t lba, const uint8_t *buf) {
  if (sd_cmd(24, sd_hc ? lba : lba * 512) != 0x00) { sd_cs_high(); return false; }
  sd_xfer(0xFF); sd_xfer(0xFE);
  for (int i = 0; i < 512; i++) sd_xfer(buf[i]);
  sd_xfer(0xFF); sd_xfer(0xFF);
  bool ok = (sd_xfer(0xFF) & 0x1F) == 0x05;
  ok = sd_wait_ready(500) && ok;
  sd_cs_high(); sd_xfer(0xFF);
  return ok;
}

#else   // ---------------------------------------------- PC test build: card = file
static FILE *sd_host_file;
static const char *sd_fail = "no card file";
static bool sd_init(void) { return sd_host_file != NULL; }
static bool sd_read(uint32_t lba, uint8_t *buf) {
  fseek(sd_host_file, (long)lba * 512, SEEK_SET); return fread(buf, 1, 512, sd_host_file) == 512;
}
static bool sd_write(uint32_t lba, const uint8_t *buf) {
  fseek(sd_host_file, (long)lba * 512, SEEK_SET); return fwrite(buf, 1, 512, sd_host_file) == 512;
}
#endif
