// disk470.h - OSI-style floppy interface: PIA at $C000, ACIA at $C010,
// up to four 5.25" drives backed by Gotek HFE images on the SD card.
//
// The HFE track holds FM bit cells (4 cells per data bit: clock, -, data, -).
// A track is decoded into a plain bit stream when the head lands on it;
// the bits feed the disk ACIA (8E1, 125 kbit/s) exactly in step with the
// emulated 65C02 clock, so the DOS's own read/write timing is unchanged.
// Written tracks are encoded back into the HFE file in place, so the
// images stay usable on the Gotek.
#pragma once
#include "fat.h"

#define NDRIVES        4
#define HFE_MAX_TRACKS 84
#define TRK_MAX_BITS   40960                      // >= cells per side / 4
#define DISK_BIT_CYCLES (CPU_HZ / DISK_BIT_RATE)  // 8 at 1 MHz
static const uint8_t drive_map[4] = DRIVE_SELECT_MAP;

struct Drive {
  bool     mounted, wp;
  char     name[40];
  FatFile  file;
  uint8_t  ntracks;
  uint16_t trk_blk[HFE_MAX_TRACKS];               // track start, in 512-byte blocks
  uint16_t trk_len[HFE_MAX_TRACKS];               // bytes, both sides
  uint8_t  head;                                  // head position (track)
};
static Drive drv[NDRIVES];

static uint8_t  trk_bits[TRK_MAX_BITS / 8];
static int8_t   trk_drive = -1;
static uint8_t  trk_track;
static uint32_t trk_nbits = 34000;
static bool     trk_dirty;
static uint32_t trk_last_write;                   // cpu_clock of last written bit
static uint32_t disk_pos;                         // bit position under the head
static uint32_t disk_clock_last, disk_frac;
static const char *disk_error;                    // last SD error, shown in status
static uint32_t disk_activity_ms;                 // millis() of the last disk data byte

static inline uint8_t trk_get(uint32_t i) { return (trk_bits[i >> 3] >> (i & 7)) & 1; }
static inline void    trk_put(uint32_t i, uint8_t v) {
  if (v) trk_bits[i >> 3] |= (uint8_t)(1 << (i & 7)); else trk_bits[i >> 3] &= (uint8_t)~(1 << (i & 7));
}

// ================================================================ HFE track I/O
struct FmDecoder { uint32_t nbits, lastclk; bool have_clk, expect_clock; };

static inline void fm_emit(FmDecoder &f, uint8_t b) { if (f.nbits < TRK_MAX_BITS) trk_put(f.nbits, b); f.nbits++; }
static inline void fm_pulse(FmDecoder &f, uint32_t p) {
  if (!f.have_clk) { f.have_clk = true; f.lastclk = p; for (uint32_t z = p / 4; z; z--) fm_emit(f, 0); return; }
  if (f.expect_clock) { f.lastclk = p; f.expect_clock = false; return; }
  uint32_t g = p - f.lastclk;
  if (g <= 3) { fm_emit(f, 1); f.expect_clock = true; }            // data pulse
  else { for (uint32_t z = (g + 2) / 4; z; z--) fm_emit(f, 0); f.lastclk = p; }
}

static bool trk_flush(void) {
  if (trk_drive < 0 || !trk_dirty) return true;
  Drive &v = drv[trk_drive];
  trk_dirty = false;
  if (!v.mounted || trk_track >= v.ntracks) return true;
  uint32_t rem = v.trk_len[trk_track] / 2, cell = 0;
  for (uint32_t b = 0; rem; b++) {
    uint32_t lba = fat_file_lba(v.file, v.trk_blk[trk_track] + b);
    if (!lba || !sd_read(lba, sd_buf)) { disk_error = "SD read failed"; return false; }
    uint32_t n = rem < 256 ? rem : 256;
    for (uint32_t j = 0; j < n; j++) {
      uint8_t out = 0;
      for (int k = 0; k < 8; k++, cell++) {
        uint32_t bit = cell >> 2;
        uint8_t ph = cell & 3, c = 0;
        if (ph == 0) c = 1;                                         // clock
        else if (ph == 2 && bit < trk_nbits) c = trk_get(bit);      // data
        out |= (uint8_t)(c << k);
      }
      sd_buf[j] = out;                                              // side 0 half only
    }
    if (!sd_write(lba, sd_buf)) { disk_error = "SD write failed"; return false; }
    rem -= n;
  }
  return true;
}

static void trk_load(int d, uint8_t t) {
  trk_flush();
  trk_drive = d; trk_track = t; trk_dirty = false;
  memset(trk_bits, 0, sizeof(trk_bits));
  trk_nbits = 34000;
  Drive &v = drv[d];
  if (!v.mounted || t >= v.ntracks) return;
  FmDecoder f = {0, 0, false, false};
  uint32_t rem = v.trk_len[t] / 2, cell = 0;
  for (uint32_t b = 0; rem; b++) {
    uint32_t lba = fat_file_lba(v.file, v.trk_blk[t] + b);
    if (!lba || !sd_read(lba, sd_buf)) { disk_error = "SD read failed"; break; }
    uint32_t n = rem < 256 ? rem : 256;
    for (uint32_t j = 0; j < n; j++) {
      uint8_t c = sd_buf[j];
      if (!c) { cell += 8; continue; }
      for (int k = 0; k < 8; k++, cell++) if (c & (1 << k)) fm_pulse(f, cell);
    }
    rem -= n;
  }
  while (f.nbits < cell / 4) fm_emit(f, 0);
  trk_nbits = f.nbits > TRK_MAX_BITS ? TRK_MAX_BITS : f.nbits;
  if (trk_nbits < 1000) trk_nbits = 34000;
  if (disk_pos >= trk_nbits) disk_pos %= trk_nbits;
}

// ================================================================ mounting
static bool drive_mount(int d, const char *name, const char **why) {
  if (trk_drive == d) { trk_flush(); trk_drive = -1; }
  Drive &v = drv[d];
  v.mounted = false;
  FatEntry e;
  if (!fat_ok && !fat_mount()) { *why = fat_fail; return false; }
  if (!fat_find(name, e))      { *why = "file not found"; return false; }
  if (!fat_open(e, v.file))    { *why = "file too fragmented - copy it to the card again"; return false; }
  uint32_t lba = fat_file_lba(v.file, 0);
  if (!lba || !sd_read(lba, sd_buf) || memcmp(sd_buf, "HXCPICFE", 8) != 0) { *why = "not an HFE (v1) image"; return false; }
  v.ntracks = sd_buf[9] > HFE_MAX_TRACKS ? HFE_MAX_TRACKS : sd_buf[9];
  uint16_t tlo = rd16le(sd_buf + 18);
  bool wr_allowed = sd_buf[20] != 0;
  lba = fat_file_lba(v.file, tlo);
  if (!lba || !sd_read(lba, sd_buf)) { *why = "SD read failed"; return false; }
  for (int t = 0; t < v.ntracks; t++) { v.trk_blk[t] = rd16le(sd_buf + 4 * t); v.trk_len[t] = rd16le(sd_buf + 4 * t + 2); }
  v.wp = v.file.readonly || !wr_allowed;
  strncpy(v.name, e.name, sizeof(v.name) - 1); v.name[sizeof(v.name) - 1] = 0;
  v.mounted = true;
  return true;
}

static void drive_eject(int d) {
  if (trk_drive == d) { trk_flush(); trk_drive = -1; }
  drv[d].mounted = false;
}

static void disk_flush_all(void) { trk_flush(); }

// SBC65.CFG lines like  A=PBDOS.HFE   (B, C, D likewise). Without it, the
// first four .HFE files in the card's root directory go into A-D.
struct AutoMount { int n; char names[NDRIVES][64]; };
static bool automount_cb(const FatEntry &e, void *ctx) {
  AutoMount *m = (AutoMount *)ctx;
  if (!(e.attr & 0x10) && name_has_ext(e.name, ".HFE")) strcpy(m->names[m->n++], e.name);
  return m->n < NDRIVES;
}

static void disk_mount_all(void) {
  for (int d = 0; d < NDRIVES; d++) drive_eject(d);
  disk_error = NULL;
  if (!fat_mount()) { disk_error = fat_fail; return; }
  char cfg[513];
  if (fat_read_small("SBC65.CFG", cfg, sizeof(cfg))) {
    char *p = cfg;
    while (*p) {
      char *line = p;
      while (*p && *p != '\n' && *p != '\r') p++;
      if (*p) *p++ = 0;
      while (*line == ' ') line++;
      int d = toupper((uint8_t)line[0]) - 'A';
      char *sep = line + 1;
      while (*sep == ' ') sep++;
      if (d >= 0 && d < NDRIVES && (*sep == '=' || *sep == ':')) {
        char *nm = sep + 1;
        while (*nm == ' ') nm++;
        char *end = nm + strlen(nm);
        while (end > nm && end[-1] == ' ') *--end = 0;
        const char *why;
        if (*nm && !drive_mount(d, nm, &why)) disk_error = why;
      }
    }
  } else {
    static AutoMount m;
    m.n = 0;
    fat_scan_root(automount_cb, &m);
    const char *why;
    for (int i = 0, d = 0; i < m.n; i++) if (drive_mount(d, m.names[i], &why)) d++;
  }
}

// ================================================================ PIA $C000-$C003
struct Pia6821 { uint8_t ora, ddra, cra, orb, ddrb, crb; };
static Pia6821 dpia;

static inline uint8_t pia_pa(void) { return (dpia.ora & dpia.ddra) | (uint8_t)~dpia.ddra; }
static inline uint8_t pia_pb(void) { return (dpia.orb & dpia.ddrb) | (uint8_t)~dpia.ddrb; }
static inline int  sel_drive(void)   { return drive_map[((pia_pa() >> 6) & 1) * 2 + ((pia_pb() >> 5) & 1)]; }
static inline bool head_loaded(void) { return !(pia_pb() & 0x80); }
static inline bool write_gate(void)  { return !(pia_pb() & 0x01); }

// ================================================================ ACIA $C010-$C011
struct DiskAcia {
  uint8_t  ctrl, rdr, st, tdr;
  bool     tdr_full;
  uint16_t txframe; uint8_t txleft;
  uint16_t rxshift; uint8_t rxcnt;
};
static DiskAcia dacia = {0x03};

static inline void dacia_format(uint8_t &nb, int8_t &par, uint8_t &stops) {
  uint8_t ws = (dacia.ctrl >> 2) & 7;
  nb = ws < 4 ? 7 : 8;
  par = (ws == 4 || ws == 5) ? -1 : (ws & 1);          // 0 even, 1 odd, -1 none
  stops = (ws == 0 || ws == 1 || ws == 4) ? 2 : 1;
}
static inline uint8_t parity_bit(uint8_t v, int8_t par) {
  v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
  return (v & 1) ^ (uint8_t)par;                         // makes the total even (or odd)
}
static inline void dacia_rx_bit(uint8_t line) {
  if (!dacia.rxcnt) { if (!line) { dacia.rxcnt = 1; dacia.rxshift = 0; } return; }
  dacia.rxshift |= (uint16_t)line << (dacia.rxcnt - 1);
  dacia.rxcnt++;
  uint8_t nb, stops; int8_t par;
  dacia_format(nb, par, stops);
  uint8_t need = nb + (par >= 0) + 1;
  if (dacia.rxcnt - 1 < need) return;
  dacia.rxcnt = 0;
  uint8_t data = dacia.rxshift & ((1u << nb) - 1);
  uint8_t st = 0x01;
  if (par >= 0 && ((dacia.rxshift >> nb) & 1) != parity_bit(data, par)) st |= 0x40;
  if (!((dacia.rxshift >> (need - 1)) & 1)) st |= 0x10;
  if (dacia.st & 0x01) { dacia.st |= 0x20; return; }    // overrun: new byte lost
  dacia.rdr = data;
  dacia.st = st;
}
static inline void dacia_tx_load(void) {
  uint8_t nb, stops; int8_t par;
  dacia_format(nb, par, stops);
  uint8_t data = dacia.tdr & ((1u << nb) - 1);
  uint16_t f = (uint16_t)data << 1;                       // start bit = 0
  uint8_t n = 1 + nb;
  if (par >= 0) { f |= (uint16_t)parity_bit(data, par) << n; n++; }
  for (uint8_t s = 0; s < stops; s++) { f |= (uint16_t)1 << n; n++; }
  dacia.txframe = f; dacia.txleft = n; dacia.tdr_full = false;
}

// ================================================================ the spinning disk
static inline bool disk_index(void) {
  int d = sel_drive();
  return drv[d].mounted && disk_pos < DISK_INDEX_BITS;
}

// Bring the disk up to the current CPU time, bit by bit.
static void disk_sync(void) {
  uint32_t el = cpu_clock - disk_clock_last;
  disk_clock_last = cpu_clock;
  disk_frac += el;
  uint32_t slots = disk_frac / DISK_BIT_CYCLES;
  disk_frac -= slots * DISK_BIT_CYCLES;
  if (!slots) return;

  int  d = sel_drive();
  bool active = drv[d].mounted && head_loaded();
  if (active && (trk_drive != d || trk_track != drv[d].head)) trk_load(d, drv[d].head);
  bool on_track = active && drv[d].head < drv[d].ntracks;
  bool wg = write_gate();
  bool reading = on_track && !wg;
  bool writing = on_track && wg && !drv[d].wp;
  bool in_reset = (dacia.ctrl & 3) == 3;

  if (!reading && !writing && !dacia.txleft && !dacia.tdr_full && !dacia.rxcnt) {
    disk_pos = (uint32_t)(((uint64_t)disk_pos + slots) % trk_nbits);
  } else {
    while (slots--) {
      uint8_t out = 1;
      if (!in_reset) {
        if (!dacia.txleft && dacia.tdr_full) dacia_tx_load();
        if (dacia.txleft) { out = dacia.txframe & 1; dacia.txframe >>= 1; dacia.txleft--; }
      }
      if (writing) { trk_put(disk_pos, !out); trk_dirty = true; }
      if (!in_reset) dacia_rx_bit(reading ? !trk_get(disk_pos) : 1);
      if (++disk_pos >= trk_nbits) disk_pos = 0;
    }
    if (writing) trk_last_write = cpu_clock;
  }
  // write a modified track back once it has been left alone for a while
  if (trk_dirty && !writing && (uint32_t)(cpu_clock - trk_last_write) > DISK_FLUSH_CYCLES) trk_flush();
}

static void disk_reset(void) {          // /RES resets the PIA (the ACIA has no reset pin)
  disk_sync();
  trk_flush();
  memset(&dpia, 0, sizeof(dpia));
}

static uint8_t disk_read(uint16_t a) {
  disk_sync();
  if ((a & 0x0FF0) == 0x0000) {                         // PIA (mirrored $C004-$C00F)
    switch (a & 3) {
      case 0: {
        if (!(dpia.cra & 0x04)) return dpia.ddra;
        uint8_t pb5 = (pia_pb() >> 5) & 1;
        int d1 = drive_map[2 + pb5], d2 = drive_map[pb5], d = sel_drive();
        uint8_t in = 0xFF;
        if (drv[d1].mounted) in &= ~0x01;              // drive 1 ready
        if (drv[d].head == 0) in &= ~0x02;             // track 0
        if (drv[d2].mounted) in &= ~0x10;              // drive 2 ready
        if (drv[d].mounted && drv[d].wp) in &= ~0x20;  // write protected
        if (disk_index()) in &= ~0x80;                 // index hole
        return (dpia.ora & dpia.ddra) | (in & ~dpia.ddra);
      }
      case 1: return dpia.cra & 0x3F;
      case 2: return (dpia.crb & 0x04) ? pia_pb() : dpia.ddrb;
      default: return dpia.crb & 0x3F;
    }
  }
  if ((a & 0x0FF0) == 0x0010) {                         // ACIA
    if (a & 1) { uint8_t v = dacia.rdr; dacia.st &= ~0x71; disk_activity_ms = millis(); return v; }
    return dacia.st | (dacia.tdr_full ? 0 : 0x02);
  }
  return OPEN_BUS;
}

static void disk_write(uint16_t a, uint8_t v) {
  disk_sync();
  if ((a & 0x0FF0) == 0x0000) {
    uint8_t old_pb = pia_pb();
    switch (a & 3) {
      case 0: if (dpia.cra & 0x04) dpia.ora = v; else dpia.ddra = v; break;
      case 1: dpia.cra = v; break;
      case 2: if (dpia.crb & 0x04) dpia.orb = v; else dpia.ddrb = v; break;
      default: dpia.crb = v; break;
    }
    uint8_t pb = pia_pb();
    if ((old_pb & 0x08) && !(pb & 0x08)) {              // step pulse (1 -> 0)
      Drive &s = drv[sel_drive()];
      if (pb & 0x04) { if (s.head) s.head--; }          // towards track 0
      else if (s.head < 79) s.head++;                   // towards track 39
    }
    if (!(old_pb & 0x80) && (pb & 0x80)) trk_flush();   // head lifted
  } else if ((a & 0x0FF0) == 0x0010) {
    if (a & 1) { dacia.tdr = v; dacia.tdr_full = true; disk_activity_ms = millis(); }
    else {
      dacia.ctrl = v;
      if ((v & 3) == 3) { dacia.st = 0; dacia.tdr_full = false; dacia.txleft = 0; dacia.rxcnt = 0; }
    }
  }
}
