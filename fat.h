// fat.h - small FAT16 / FAT32 reader for the SD card
//
// Root directory only, long file names supported. Files are read and written
// in place (sizes never change), so no cluster allocation is needed.
#pragma once
#include "sdcard.h"

static uint8_t  sd_buf[512];
static bool     fat_ok, fat32;
static uint8_t  fat_spc;                    // sectors per cluster
static uint32_t fat_start, fat_size, data_start, root_start, root_secs, root_clus;

static inline uint16_t rd16le(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t rd32le(const uint8_t *p) { return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static const char *fat_fail = "";

static bool fat_mount(void) {
  fat_ok = false;
  if (!sd_init()) { fat_fail = sd_fail; return false; }
  if (!sd_read(0, sd_buf)) { fat_fail = "card initialised but reading failed"; return false; }
  fat_fail = "card is not FAT16/FAT32 (cards over 32GB come as exFAT - reformat as FAT32)";
  if (sd_buf[510] != 0x55 || sd_buf[511] != 0xAA) return false;
  uint32_t part = 0;
  bool vbr = (sd_buf[0] == 0xEB || sd_buf[0] == 0xE9) && rd16le(sd_buf + 11) == 512;
  if (!vbr) {                                          // MBR: first partition
    part = rd32le(sd_buf + 0x1C6);
    if (!part || !sd_read(part, sd_buf) || rd16le(sd_buf + 11) != 512) return false;
    if (memcmp(sd_buf + 3, "EXFAT", 5) == 0) return false;
  }
  fat_spc = sd_buf[13];
  uint16_t rsvd = rd16le(sd_buf + 14);
  uint8_t  nfats = sd_buf[16];
  uint16_t rootent = rd16le(sd_buf + 17);
  fat_size = rd16le(sd_buf + 22);
  fat32 = (fat_size == 0);
  if (fat32) { fat_size = rd32le(sd_buf + 36); root_clus = rd32le(sd_buf + 44); }
  if (!fat_spc || !nfats) return false;
  fat_start  = part + rsvd;
  root_start = fat_start + nfats * fat_size;
  root_secs  = fat32 ? 0 : ((uint32_t)rootent * 32 + 511) / 512;
  data_start = root_start + root_secs;
  fat_ok = true;
  return true;
}

static inline uint32_t clus_lba(uint32_t c) { return data_start + (c - 2) * fat_spc; }

static uint32_t fat_next(uint32_t c) {
  uint32_t off = fat32 ? c * 4 : c * 2;
  if (!sd_read(fat_start + off / 512, sd_buf)) return 0x0FFFFFFF;
  if (fat32) return rd32le(sd_buf + off % 512) & 0x0FFFFFFF;
  uint16_t v = rd16le(sd_buf + off % 512);
  return v >= 0xFFF8 ? 0x0FFFFFFF : v;
}
static inline bool fat_eoc(uint32_t c) { return c < 2 || c >= 0x0FFFFFF8; }

// ------------------------------------------------ directory scan
struct FatEntry {
  char     name[64];
  uint32_t clus, size;
  uint8_t  attr;
};

// Calls fn(entry, ctx) for each file in the root directory; stops if fn returns false.
typedef bool (*fat_dir_fn)(const FatEntry &e, void *ctx);

static void fat_scan_root(fat_dir_fn fn, void *ctx) {
  if (!fat_ok) return;
  FatEntry e;
  char lfn[64]; bool lfn_ok = false;
  uint32_t clus = root_clus, sec = 0;
  for (;;) {
    uint32_t lba;
    if (fat32) {
      if (sec == fat_spc) { clus = fat_next(clus); sec = 0; if (fat_eoc(clus)) return; }
      lba = clus_lba(clus) + sec;
    } else {
      if (sec == root_secs) return;
      lba = root_start + sec;
    }
    sec++;
    if (!sd_read(lba, sd_buf)) return;
    for (int i = 0; i < 512; i += 32) {
      uint8_t *d = sd_buf + i;
      if (d[0] == 0x00) return;
      if (d[0] == 0xE5) { lfn_ok = false; continue; }
      if (d[11] == 0x0F) {                                     // long name piece
        int seq = d[0] & 0x1F;
        if (d[0] & 0x40) { memset(lfn, 0, sizeof(lfn)); lfn_ok = true; }
        static const uint8_t pos[13] = {1,3,5,7,9,14,16,18,20,22,24,28,30};
        for (int k = 0; k < 13; k++) {
          int idx = (seq - 1) * 13 + k;
          uint16_t ch = rd16le(d + pos[k]);
          if (idx < 63 && ch && ch != 0xFFFF) lfn[idx] = ch < 128 ? (char)ch : '?';
        }
        continue;
      }
      if (d[11] & 0x18) { lfn_ok = false; continue; }          // volume label / directory
      if (lfn_ok && lfn[0]) strcpy(e.name, lfn);
      else {                                                   // 8.3 name
        int n = 0;
        for (int k = 0; k < 8 && d[k] != ' '; k++) e.name[n++] = d[k];
        if (d[8] != ' ') { e.name[n++] = '.'; for (int k = 8; k < 11 && d[k] != ' '; k++) e.name[n++] = d[k]; }
        e.name[n] = 0;
      }
      lfn_ok = false;
      e.attr = d[11];
      e.clus = ((uint32_t)rd16le(d + 20) << 16) | rd16le(d + 26);
      e.size = rd32le(d + 28);
      uint8_t save[512];                                       // fn may use sd_buf
      memcpy(save, sd_buf, 512);
      if (!fn(e, ctx)) return;
      memcpy(sd_buf, save, 512);
    }
  }
}

static bool name_ieq(const char *a, const char *b) {
  while (*a && *b) { if (toupper((uint8_t)*a) != toupper((uint8_t)*b)) return false; a++; b++; }
  return *a == *b;
}
static bool name_has_ext(const char *a, const char *ext) {
  size_t n = strlen(a), m = strlen(ext);
  return n > m && name_ieq(a + n - m, ext);
}

struct FatFind { const char *want; FatEntry found; bool ok; };
static bool fat_find_cb(const FatEntry &e, void *ctx) {
  FatFind *f = (FatFind *)ctx;
  if (name_ieq(e.name, f->want)) { f->found = e; f->ok = true; return false; }
  return true;
}
static bool fat_find(const char *name, FatEntry &out) {
  FatFind f; f.want = name; f.ok = false;
  fat_scan_root(fat_find_cb, &f);
  if (f.ok) out = f.found;
  return f.ok;
}

// ------------------------------------------------ open file = list of extents
#define FAT_MAX_EXTENTS 24
struct FatFile {
  uint32_t size;
  uint8_t  next;
  uint8_t  readonly;
  uint32_t ext_lba[FAT_MAX_EXTENTS];      // first sector of each run of clusters
  uint32_t ext_secs[FAT_MAX_EXTENTS];     // length of each run in sectors
};

// returns false if the file is too fragmented (copy it to the card again)
static bool fat_open(const FatEntry &e, FatFile &f) {
  f.size = e.size; f.next = 0; f.readonly = e.attr & 0x01;
  uint32_t c = e.clus;
  while (!fat_eoc(c)) {
    uint32_t start = c, n = 1, nx;
    while (!fat_eoc(nx = fat_next(c)) && nx == c + 1) { c = nx; n++; }
    if (f.next == FAT_MAX_EXTENTS) return false;
    f.ext_lba[f.next] = clus_lba(start);
    f.ext_secs[f.next] = n * fat_spc;
    f.next++;
    c = nx;
  }
  return f.next > 0;
}

// sector number within the file -> card LBA (0 if past the end)
static uint32_t fat_file_lba(const FatFile &f, uint32_t fsec) {
  for (uint8_t i = 0; i < f.next; i++) {
    if (fsec < f.ext_secs[i]) return f.ext_lba[i] + fsec;
    fsec -= f.ext_secs[i];
  }
  return 0;
}

// read the first sector of a small text file (e.g. SBC65.CFG)
static bool fat_read_small(const char *name, char *out, int max) {
  FatEntry e;
  if (!fat_find(name, e) || fat_eoc(e.clus)) return false;
  if (!sd_read(clus_lba(e.clus), sd_buf)) return false;
  int n = e.size < (uint32_t)(max - 1) ? (int)e.size : max - 1;
  if (n > 512) n = 512;
  memcpy(out, sd_buf, n); out[n] = 0;
  return true;
}
