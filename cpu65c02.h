// cpu65c02.h - WDC 65C02 CPU core for the SBC65 emulator
//
// Complete WDC 65C02 instruction set:
//   * all NMOS documented opcodes (with the 65C02 fixes: JMP ($xxFF) works,
//     D flag cleared on interrupts, N/Z valid in decimal mode)
//   * 65C02 additions: BRA PHX PHY PLX PLY STZ TSB TRB, (zp) addressing,
//     BIT #/zp,x/abs,x, INC A, DEC A, JMP (abs,x)
//   * WDC/Rockwell extras: RMBn SMBn BBRn BBSn, WAI, STP
//   * unused opcodes behave as the WDC NOPs (correct byte length)
//
// Before including this file, define:
//   static inline uint8_t bus_read(uint16_t addr);
//   static inline void    bus_write(uint16_t addr, uint8_t val);
//
// Passes Klaus Dormann's 6502_functional_test and 65C02_extended_opcodes_test.

#pragma once
#include <stdint.h>

enum { FLAG_C = 0x01, FLAG_Z = 0x02, FLAG_I = 0x04, FLAG_D = 0x08, FLAG_B = 0x10, FLAG_U = 0x20, FLAG_V = 0x40, FLAG_N = 0x80 };

struct Cpu65C02 {
  uint16_t pc;
  uint8_t  a, x, y, s, p;
  uint8_t  waiting;   // WAI executed, waiting for IRQ/NMI
  uint8_t  stopped;   // STP executed, only RESET restarts
  uint8_t  irq;       // IRQ line level (non-zero = asserted), set by devices
  uint8_t  nmi;       // set to 1 to request an NMI (edge)
};
static Cpu65C02 cpu;

// Base cycle counts (WDC 65C02). Page-cross, branch and decimal extras added at run time.
static const uint8_t cpu_cycles[256] = {
// 0 1 2 3 4 5 6 7 8 9 A B C D E F
   7,6,2,1,5,3,5,5,3,2,2,1,6,4,6,5,  // 0
   2,5,5,1,5,4,6,5,2,4,2,1,6,4,6,5,  // 1
   6,6,2,1,3,3,5,5,4,2,2,1,4,4,6,5,  // 2
   2,5,5,1,4,4,6,5,2,4,2,1,4,4,6,5,  // 3
   6,6,2,1,3,3,5,5,3,2,2,1,3,4,6,5,  // 4
   2,5,5,1,4,4,6,5,2,4,3,1,8,4,6,5,  // 5
   6,6,2,1,3,3,5,5,4,2,2,1,6,4,6,5,  // 6
   2,5,5,1,4,4,6,5,2,4,4,1,6,4,6,5,  // 7
   3,6,2,1,3,3,3,5,2,2,2,1,4,4,4,5,  // 8
   2,6,5,1,4,4,4,5,2,5,2,1,4,5,5,5,  // 9
   2,6,2,1,3,3,3,5,2,2,2,1,4,4,4,5,  // A
   2,5,5,1,4,4,4,5,2,4,2,1,4,4,4,5,  // B
   2,6,2,1,3,3,5,5,2,2,2,3,4,4,6,5,  // C
   2,5,5,1,4,4,6,5,2,4,3,3,4,4,7,5,  // D
   2,6,2,1,3,3,5,5,2,2,2,1,4,4,6,5,  // E
   2,5,5,1,4,4,6,5,2,4,4,1,4,4,7,5   // F
};

// ---------------------------------------------------------------- helpers
static inline uint16_t c_rd16(uint16_t a) {
  return bus_read(a) | ((uint16_t)bus_read((uint16_t)(a + 1)) << 8);
}
static inline uint16_t c_rdzp16(uint8_t z) {
  return bus_read(z) | ((uint16_t)bus_read((uint8_t)(z + 1)) << 8);
}
static inline void    c_push(uint8_t v) { bus_write(0x0100 | cpu.s, v); cpu.s--; }
static inline uint8_t c_pop(void)       { cpu.s++; return bus_read(0x0100 | cpu.s); }
static inline void    c_nz(uint8_t v)   { cpu.p = (cpu.p & ~(FLAG_N | FLAG_Z)) | (v & FLAG_N) | (v ? 0 : FLAG_Z); }

// abs,X / abs,Y with page-cross penalty
static inline uint16_t c_abi(uint8_t i, int &cyc) {
  uint16_t b = c_rd16(cpu.pc); cpu.pc += 2;
  uint16_t e = (uint16_t)(b + i);
  if ((b ^ e) & 0xFF00) cyc++;
  return e;
}
// (zp),Y with page-cross penalty
static inline uint16_t c_izy(int &cyc) {
  uint16_t b = c_rdzp16(bus_read(cpu.pc++));
  uint16_t e = (uint16_t)(b + cpu.y);
  if ((b ^ e) & 0xFF00) cyc++;
  return e;
}

#define A_ZP()   ((uint16_t)bus_read(cpu.pc++))
#define A_ZPX()  ((uint16_t)(uint8_t)(bus_read(cpu.pc++) + cpu.x))
#define A_ZPY()  ((uint16_t)(uint8_t)(bus_read(cpu.pc++) + cpu.y))
#define A_ABS()  (cpu.pc += 2, c_rd16((uint16_t)(cpu.pc - 2)))
#define A_ABXN() (cpu.pc += 2, (uint16_t)(c_rd16((uint16_t)(cpu.pc - 2)) + cpu.x))  // no penalty
#define A_ABYN() (cpu.pc += 2, (uint16_t)(c_rd16((uint16_t)(cpu.pc - 2)) + cpu.y))
#define A_ABX()  c_abi(cpu.x, cyc)
#define A_ABY()  c_abi(cpu.y, cyc)
#define A_IZX()  c_rdzp16((uint8_t)(bus_read(cpu.pc++) + cpu.x))
#define A_IZY()  c_izy(cyc)
#define A_IZYN() ((uint16_t)(c_rdzp16(bus_read(cpu.pc++)) + cpu.y))
#define A_IZP()  c_rdzp16(bus_read(cpu.pc++))

#define V_IMM bus_read(cpu.pc++)
#define V_ZP  bus_read(A_ZP())
#define V_ZPX bus_read(A_ZPX())
#define V_ZPY bus_read(A_ZPY())
#define V_ABS bus_read(A_ABS())
#define V_ABX bus_read(A_ABX())
#define V_ABY bus_read(A_ABY())
#define V_IZX bus_read(A_IZX())
#define V_IZY bus_read(A_IZY())
#define V_IZP bus_read(A_IZP())

// ---------------------------------------------------------------- ALU
static inline void c_adc(uint8_t v, int &cyc) {
  uint8_t a = cpu.a;
  unsigned c = cpu.p & FLAG_C;
  cpu.p &= ~(FLAG_C | FLAG_V);
  if (cpu.p & FLAG_D) {
    int al = (a & 0x0F) + (v & 0x0F) + c;
    if (al >= 0x0A) al = ((al + 0x06) & 0x0F) + 0x10;
    int r = (a & 0xF0) + (v & 0xF0) + al;
    if ((~(a ^ v) & (a ^ r)) & 0x80) cpu.p |= FLAG_V;
    if (r >= 0xA0) r += 0x60;
    if (r >= 0x100) cpu.p |= FLAG_C;
    cpu.a = (uint8_t)r;
    cyc++;
  } else {
    unsigned r = a + v + c;
    if (r > 0xFF) cpu.p |= FLAG_C;
    if ((~(a ^ v) & (a ^ r)) & 0x80) cpu.p |= FLAG_V;
    cpu.a = (uint8_t)r;
  }
  c_nz(cpu.a);
}

static inline void c_sbc(uint8_t v, int &cyc) {
  uint8_t a = cpu.a;
  int c = cpu.p & FLAG_C;
  int bin = a - v - (1 - c);
  cpu.p &= ~(FLAG_C | FLAG_V);
  if (bin >= 0) cpu.p |= FLAG_C;
  if (((a ^ v) & (a ^ bin)) & 0x80) cpu.p |= FLAG_V;
  if (cpu.p & FLAG_D) {
    int al = (a & 0x0F) - (v & 0x0F) + c - 1;
    int r = bin;
    if (r < 0)  r -= 0x60;
    if (al < 0) r -= 0x06;
    cpu.a = (uint8_t)r;
    cyc++;
  } else {
    cpu.a = (uint8_t)bin;
  }
  c_nz(cpu.a);
}

static inline uint8_t c_asl(uint8_t v) { cpu.p = (cpu.p & ~FLAG_C) | (v >> 7); v <<= 1; c_nz(v); return v; }
static inline uint8_t c_lsr(uint8_t v) { cpu.p = (cpu.p & ~FLAG_C) | (v & 1);  v >>= 1; c_nz(v); return v; }
static inline uint8_t c_rol(uint8_t v) { uint8_t c = cpu.p & FLAG_C; cpu.p = (cpu.p & ~FLAG_C) | (v >> 7); v = (uint8_t)((v << 1) | c); c_nz(v); return v; }
static inline uint8_t c_ror(uint8_t v) { uint8_t c = cpu.p & FLAG_C; cpu.p = (cpu.p & ~FLAG_C) | (v & 1);  v = (uint8_t)((v >> 1) | (c << 7)); c_nz(v); return v; }
static inline uint8_t c_inc(uint8_t v) { v++; c_nz(v); return v; }
static inline uint8_t c_dec(uint8_t v) { v--; c_nz(v); return v; }
static inline void c_cmp(uint8_t r, uint8_t v) {
  uint8_t t = (uint8_t)(r - v);
  cpu.p = (cpu.p & ~(FLAG_C | FLAG_Z | FLAG_N)) | (r >= v ? FLAG_C : 0) | (t ? 0 : FLAG_Z) | (t & FLAG_N);
}
static inline void c_bit(uint8_t v) {
  cpu.p = (cpu.p & ~(FLAG_N | FLAG_V | FLAG_Z)) | (v & (FLAG_N | FLAG_V)) | ((cpu.a & v) ? 0 : FLAG_Z);
}
static inline void c_branch(bool cond, int &cyc) {
  int8_t off = (int8_t)bus_read(cpu.pc++);
  if (cond) {
    uint16_t t = (uint16_t)(cpu.pc + off);
    cyc += ((t ^ cpu.pc) & 0xFF00) ? 2 : 1;
    cpu.pc = t;
  }
}

#define ORA(v) { cpu.a |= (v); c_nz(cpu.a); }
#define AND(v) { cpu.a &= (v); c_nz(cpu.a); }
#define EOR(v) { cpu.a ^= (v); c_nz(cpu.a); }
#define LDA(v) { cpu.a = (v);  c_nz(cpu.a); }
#define LDX(v) { cpu.x = (v);  c_nz(cpu.x); }
#define LDY(v) { cpu.y = (v);  c_nz(cpu.y); }
#define RMW(ea, fn) { uint16_t e_ = (ea); bus_write(e_, fn(bus_read(e_))); }

// ---------------------------------------------------------------- control
static inline void c_interrupt(uint16_t vector) {
  c_push(cpu.pc >> 8);
  c_push(cpu.pc & 0xFF);
  c_push((cpu.p & ~FLAG_B) | FLAG_U);
  cpu.p = (cpu.p | FLAG_I) & ~FLAG_D;
  cpu.pc = c_rd16(vector);
}

static void cpu_reset(void) {
  cpu.s = 0xFD;
  cpu.p = FLAG_U | FLAG_I | FLAG_B;
  cpu.waiting = cpu.stopped = 0;
  cpu.nmi = 0;
  cpu.pc = c_rd16(0xFFFC);
}

// Execute one instruction (or service one interrupt). Returns clock cycles used.
static int cpu_step(void) {
  if (cpu.stopped) return 1;
  if (cpu.nmi) { cpu.nmi = 0; cpu.waiting = 0; c_interrupt(0xFFFA); return 7; }
  if (cpu.irq) {
    if (!(cpu.p & FLAG_I)) { cpu.waiting = 0; c_interrupt(0xFFFE); return 7; }
    cpu.waiting = 0;          // WAI with I set: just carry on
  }
  if (cpu.waiting) return 1;

  uint8_t op = bus_read(cpu.pc++);
  int cyc = cpu_cycles[op];
  uint16_t ea;
  uint8_t t;

  switch (op) {
  // ---- ORA
  case 0x09: ORA(V_IMM); break;  case 0x05: ORA(V_ZP);  break;
  case 0x15: ORA(V_ZPX); break;  case 0x0D: ORA(V_ABS); break;
  case 0x1D: ORA(V_ABX); break;  case 0x19: ORA(V_ABY); break;
  case 0x01: ORA(V_IZX); break;  case 0x11: ORA(V_IZY); break;
  case 0x12: ORA(V_IZP); break;
  // ---- AND
  case 0x29: AND(V_IMM); break;  case 0x25: AND(V_ZP);  break;
  case 0x35: AND(V_ZPX); break;  case 0x2D: AND(V_ABS); break;
  case 0x3D: AND(V_ABX); break;  case 0x39: AND(V_ABY); break;
  case 0x21: AND(V_IZX); break;  case 0x31: AND(V_IZY); break;
  case 0x32: AND(V_IZP); break;
  // ---- EOR
  case 0x49: EOR(V_IMM); break;  case 0x45: EOR(V_ZP);  break;
  case 0x55: EOR(V_ZPX); break;  case 0x4D: EOR(V_ABS); break;
  case 0x5D: EOR(V_ABX); break;  case 0x59: EOR(V_ABY); break;
  case 0x41: EOR(V_IZX); break;  case 0x51: EOR(V_IZY); break;
  case 0x52: EOR(V_IZP); break;
  // ---- ADC
  case 0x69: c_adc(V_IMM, cyc); break;  case 0x65: c_adc(V_ZP, cyc);  break;
  case 0x75: c_adc(V_ZPX, cyc); break;  case 0x6D: c_adc(V_ABS, cyc); break;
  case 0x7D: c_adc(V_ABX, cyc); break;  case 0x79: c_adc(V_ABY, cyc); break;
  case 0x61: c_adc(V_IZX, cyc); break;  case 0x71: c_adc(V_IZY, cyc); break;
  case 0x72: c_adc(V_IZP, cyc); break;
  // ---- SBC
  case 0xE9: c_sbc(V_IMM, cyc); break;  case 0xE5: c_sbc(V_ZP, cyc);  break;
  case 0xF5: c_sbc(V_ZPX, cyc); break;  case 0xED: c_sbc(V_ABS, cyc); break;
  case 0xFD: c_sbc(V_ABX, cyc); break;  case 0xF9: c_sbc(V_ABY, cyc); break;
  case 0xE1: c_sbc(V_IZX, cyc); break;  case 0xF1: c_sbc(V_IZY, cyc); break;
  case 0xF2: c_sbc(V_IZP, cyc); break;
  // ---- CMP / CPX / CPY
  case 0xC9: c_cmp(cpu.a, V_IMM); break;  case 0xC5: c_cmp(cpu.a, V_ZP);  break;
  case 0xD5: c_cmp(cpu.a, V_ZPX); break;  case 0xCD: c_cmp(cpu.a, V_ABS); break;
  case 0xDD: c_cmp(cpu.a, V_ABX); break;  case 0xD9: c_cmp(cpu.a, V_ABY); break;
  case 0xC1: c_cmp(cpu.a, V_IZX); break;  case 0xD1: c_cmp(cpu.a, V_IZY); break;
  case 0xD2: c_cmp(cpu.a, V_IZP); break;
  case 0xE0: c_cmp(cpu.x, V_IMM); break;  case 0xE4: c_cmp(cpu.x, V_ZP);  break;
  case 0xEC: c_cmp(cpu.x, V_ABS); break;
  case 0xC0: c_cmp(cpu.y, V_IMM); break;  case 0xC4: c_cmp(cpu.y, V_ZP);  break;
  case 0xCC: c_cmp(cpu.y, V_ABS); break;
  // ---- LDA / LDX / LDY
  case 0xA9: LDA(V_IMM); break;  case 0xA5: LDA(V_ZP);  break;
  case 0xB5: LDA(V_ZPX); break;  case 0xAD: LDA(V_ABS); break;
  case 0xBD: LDA(V_ABX); break;  case 0xB9: LDA(V_ABY); break;
  case 0xA1: LDA(V_IZX); break;  case 0xB1: LDA(V_IZY); break;
  case 0xB2: LDA(V_IZP); break;
  case 0xA2: LDX(V_IMM); break;  case 0xA6: LDX(V_ZP);  break;
  case 0xB6: LDX(V_ZPY); break;  case 0xAE: LDX(V_ABS); break;
  case 0xBE: LDX(V_ABY); break;
  case 0xA0: LDY(V_IMM); break;  case 0xA4: LDY(V_ZP);  break;
  case 0xB4: LDY(V_ZPX); break;  case 0xAC: LDY(V_ABS); break;
  case 0xBC: LDY(V_ABX); break;
  // ---- STA / STX / STY / STZ
  case 0x85: bus_write(A_ZP(),   cpu.a); break;
  case 0x95: bus_write(A_ZPX(),  cpu.a); break;
  case 0x8D: bus_write(A_ABS(),  cpu.a); break;
  case 0x9D: bus_write(A_ABXN(), cpu.a); break;
  case 0x99: bus_write(A_ABYN(), cpu.a); break;
  case 0x81: bus_write(A_IZX(),  cpu.a); break;
  case 0x91: bus_write(A_IZYN(), cpu.a); break;
  case 0x92: bus_write(A_IZP(),  cpu.a); break;
  case 0x86: bus_write(A_ZP(),   cpu.x); break;
  case 0x96: bus_write(A_ZPY(),  cpu.x); break;
  case 0x8E: bus_write(A_ABS(),  cpu.x); break;
  case 0x84: bus_write(A_ZP(),   cpu.y); break;
  case 0x94: bus_write(A_ZPX(),  cpu.y); break;
  case 0x8C: bus_write(A_ABS(),  cpu.y); break;
  case 0x64: bus_write(A_ZP(),   0); break;
  case 0x74: bus_write(A_ZPX(),  0); break;
  case 0x9C: bus_write(A_ABS(),  0); break;
  case 0x9E: bus_write(A_ABXN(), 0); break;
  // ---- shifts / rotates
  case 0x0A: cpu.a = c_asl(cpu.a); break;
  case 0x06: RMW(A_ZP(), c_asl);   break;  case 0x16: RMW(A_ZPX(), c_asl);  break;
  case 0x0E: RMW(A_ABS(), c_asl);  break;  case 0x1E: RMW(A_ABXN(), c_asl); break;
  case 0x4A: cpu.a = c_lsr(cpu.a); break;
  case 0x46: RMW(A_ZP(), c_lsr);   break;  case 0x56: RMW(A_ZPX(), c_lsr);  break;
  case 0x4E: RMW(A_ABS(), c_lsr);  break;  case 0x5E: RMW(A_ABXN(), c_lsr); break;
  case 0x2A: cpu.a = c_rol(cpu.a); break;
  case 0x26: RMW(A_ZP(), c_rol);   break;  case 0x36: RMW(A_ZPX(), c_rol);  break;
  case 0x2E: RMW(A_ABS(), c_rol);  break;  case 0x3E: RMW(A_ABXN(), c_rol); break;
  case 0x6A: cpu.a = c_ror(cpu.a); break;
  case 0x66: RMW(A_ZP(), c_ror);   break;  case 0x76: RMW(A_ZPX(), c_ror);  break;
  case 0x6E: RMW(A_ABS(), c_ror);  break;  case 0x7E: RMW(A_ABXN(), c_ror); break;
  // ---- INC / DEC
  case 0x1A: cpu.a = c_inc(cpu.a); break;
  case 0xE6: RMW(A_ZP(), c_inc);   break;  case 0xF6: RMW(A_ZPX(), c_inc);  break;
  case 0xEE: RMW(A_ABS(), c_inc);  break;  case 0xFE: RMW(A_ABXN(), c_inc); break;
  case 0x3A: cpu.a = c_dec(cpu.a); break;
  case 0xC6: RMW(A_ZP(), c_dec);   break;  case 0xD6: RMW(A_ZPX(), c_dec);  break;
  case 0xCE: RMW(A_ABS(), c_dec);  break;  case 0xDE: RMW(A_ABXN(), c_dec); break;
  case 0xE8: cpu.x++; c_nz(cpu.x); break;  case 0xC8: cpu.y++; c_nz(cpu.y); break;
  case 0xCA: cpu.x--; c_nz(cpu.x); break;  case 0x88: cpu.y--; c_nz(cpu.y); break;
  // ---- BIT / TSB / TRB
  case 0x89: t = V_IMM; cpu.p = (cpu.p & ~FLAG_Z) | ((cpu.a & t) ? 0 : FLAG_Z); break;
  case 0x24: c_bit(V_ZP);  break;  case 0x34: c_bit(V_ZPX); break;
  case 0x2C: c_bit(V_ABS); break;  case 0x3C: c_bit(V_ABX); break;
  case 0x04: ea = A_ZP();  goto do_tsb;
  case 0x0C: ea = A_ABS();
  do_tsb:
    t = bus_read(ea);
    cpu.p = (cpu.p & ~FLAG_Z) | ((cpu.a & t) ? 0 : FLAG_Z);
    bus_write(ea, t | cpu.a);
    break;
  case 0x14: ea = A_ZP();  goto do_trb;
  case 0x1C: ea = A_ABS();
  do_trb:
    t = bus_read(ea);
    cpu.p = (cpu.p & ~FLAG_Z) | ((cpu.a & t) ? 0 : FLAG_Z);
    bus_write(ea, t & ~cpu.a);
    break;
  // ---- branches
  case 0x10: c_branch(!(cpu.p & FLAG_N), cyc); break;
  case 0x30: c_branch( (cpu.p & FLAG_N), cyc); break;
  case 0x50: c_branch(!(cpu.p & FLAG_V), cyc); break;
  case 0x70: c_branch( (cpu.p & FLAG_V), cyc); break;
  case 0x90: c_branch(!(cpu.p & FLAG_C), cyc); break;
  case 0xB0: c_branch( (cpu.p & FLAG_C), cyc); break;
  case 0xD0: c_branch(!(cpu.p & FLAG_Z), cyc); break;
  case 0xF0: c_branch( (cpu.p & FLAG_Z), cyc); break;
  case 0x80: c_branch(true, cyc); cyc--; break;   // BRA: 3 base incl. taken
  // ---- flags
  case 0x18: cpu.p &= ~FLAG_C; break;  case 0x38: cpu.p |= FLAG_C; break;
  case 0x58: cpu.p &= ~FLAG_I; break;  case 0x78: cpu.p |= FLAG_I; break;
  case 0xB8: cpu.p &= ~FLAG_V; break;
  case 0xD8: cpu.p &= ~FLAG_D; break;  case 0xF8: cpu.p |= FLAG_D; break;
  // ---- transfers
  case 0xAA: cpu.x = cpu.a; c_nz(cpu.x); break;
  case 0xA8: cpu.y = cpu.a; c_nz(cpu.y); break;
  case 0x8A: cpu.a = cpu.x; c_nz(cpu.a); break;
  case 0x98: cpu.a = cpu.y; c_nz(cpu.a); break;
  case 0xBA: cpu.x = cpu.s; c_nz(cpu.x); break;
  case 0x9A: cpu.s = cpu.x; break;
  // ---- stack
  case 0x48: c_push(cpu.a); break;
  case 0xDA: c_push(cpu.x); break;
  case 0x5A: c_push(cpu.y); break;
  case 0x08: c_push(cpu.p | FLAG_B | FLAG_U); break;
  case 0x68: cpu.a = c_pop(); c_nz(cpu.a); break;
  case 0xFA: cpu.x = c_pop(); c_nz(cpu.x); break;
  case 0x7A: cpu.y = c_pop(); c_nz(cpu.y); break;
  case 0x28: cpu.p = c_pop() | FLAG_U | FLAG_B; break;
  // ---- jumps / subroutines / interrupts
  case 0x4C: cpu.pc = c_rd16(cpu.pc); break;
  case 0x6C: cpu.pc = c_rd16(c_rd16(cpu.pc)); break;                    // no page bug
  case 0x7C: cpu.pc = c_rd16((uint16_t)(c_rd16(cpu.pc) + cpu.x)); break;
  case 0x20: {
    uint16_t target = c_rd16(cpu.pc);
    uint16_t ret = (uint16_t)(cpu.pc + 1);
    c_push(ret >> 8); c_push(ret & 0xFF);
    cpu.pc = target;
    break;
  }
  case 0x60: { uint8_t lo = c_pop(); uint8_t hi = c_pop(); cpu.pc = (uint16_t)(((hi << 8) | lo) + 1); break; }
  case 0x40: {
    cpu.p = c_pop() | FLAG_U | FLAG_B;
    uint8_t lo = c_pop(); uint8_t hi = c_pop();
    cpu.pc = (uint16_t)((hi << 8) | lo);
    break;
  }
  case 0x00:
    cpu.pc++;                                     // skip signature byte
    c_push(cpu.pc >> 8); c_push(cpu.pc & 0xFF);
    c_push(cpu.p | FLAG_B | FLAG_U);
    cpu.p = (cpu.p | FLAG_I) & ~FLAG_D;
    cpu.pc = c_rd16(0xFFFE);
    break;
  case 0xEA: break;                               // NOP
  case 0xCB: cpu.waiting = 1; break;              // WAI
  case 0xDB: cpu.stopped = 1; break;              // STP
  // ---- 3-byte NOPs
  case 0x5C: case 0xDC: case 0xFC: cpu.pc += 2; break;
  // ---- 2-byte NOPs
  case 0x02: case 0x22: case 0x42: case 0x62: case 0x82: case 0xC2: case 0xE2:
  case 0x44: case 0x54: case 0xD4: case 0xF4:
    cpu.pc++; break;

  default:
    switch (op & 0x0F) {
    case 0x07: {                                  // RMBn / SMBn zp
      uint8_t bit = (uint8_t)(1 << ((op >> 4) & 7));
      ea = A_ZP();
      t = bus_read(ea);
      bus_write(ea, (op & 0x80) ? (t | bit) : (t & ~bit));
      break;
    }
    case 0x0F: {                                  // BBRn / BBSn zp,rel
      uint8_t bit = (uint8_t)(1 << ((op >> 4) & 7));
      t = bus_read(A_ZP());
      bool isset = (t & bit) != 0;
      c_branch(isset == ((op & 0x80) != 0), cyc);
      break;
    }
    default:                                      // x3 / xB : 1-byte NOPs
      break;
    }
    break;
  }
  return cyc;
}
