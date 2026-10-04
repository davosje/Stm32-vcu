/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2026 Dav <davos112@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "mcp2518fd.h"

#ifndef MCP2518FD_HOSTTEST
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>
#endif

/* SPI instructions. The command word is instruction << 12 | address. */
/* How long a single byte may take before the driver gives up on the SPI.
 * See Mcp2518Fd::Xfer: a byte is a few hundred cycles, so this is generous. */
#define XFER_GUARD 100000

#define INSTR_RESET 0x0000
#define INSTR_WRITE 0x2000
#define INSTR_READ 0x3000

/* Special function registers */
#define REG_OSC 0xE00
/* IOCON, 0xE04. Deliberately never written. In many MCP251863 designs the
 * transceiver's standby is tied to the controller's own pin and then IOCON
 * needs XSTBYEN set and TRIS0 cleared, but on this board pin 5 runs to PE12:
 * the microcontroller switches it, and the driver does that in Initialize. */
#define REG_IOCON 0xE04
#define REG_DEVID 0xE14

/* CAN controller registers */
#define REG_CON 0x000
#define REG_NBTCFG 0x004
#define REG_TSCON 0x014
#define REG_FIFOCON(m) (0x050 + 0x0C * (m))
#define REG_FIFOSTA(m) (0x054 + 0x0C * (m))
#define REG_FIFOUA(m) (0x058 + 0x0C * (m))
#define REG_FLTCON(m) (0x1D0 + 0x04 * (m))
#define REG_FLTOBJ(m) (0x1F0 + 0x08 * (m))
#define REG_FLTMASK(m) (0x1F4 + 0x08 * (m))
#define RAM_START 0x400

/* OSC */
#define OSC_OSCRDY (1 << 10)

/* What a register reads as when nobody is on the other end of the SPI: the
 * input held low, or left floating high. The Linux driver uses exactly this
 * test on OSC to decide there is no chip (mcp251xfd_reg_invalid). */
static inline bool RegInvalid(uint32_t value) {
  return value == 0 || value == 0xFFFFFFFF;
}

/* CON */
#define CON_REQOP_SHIFT 24
#define CON_REQOP_MASK (7 << 24)
#define CON_OPMOD_SHIFT 21
#define CON_OPMOD_MASK (7 << 21)
#define MODE_NORMAL_FD 0
#define MODE_CONFIG 4
#define MODE_NORMAL_20B 6

/* FIFOSTA */
#define FIFOSTA_NOTFULL_NOTEMPTY 1

/* FIFOCON, byte 1: UINC is bit 8, TXREQ is bit 9 of the 32 bit register */
#define FIFOCON_BYTE1 1
#define BYTE1_UINC 0x01
#define BYTE1_TXREQ 0x02

/* Our fixed layout: one transmit FIFO, one receive FIFO. */
#define FIFO_TX 1
#define FIFO_RX 2
#define FIFO_TX_DEPTH 8
#define FIFO_RX_DEPTH 16

/* Message object header words */
#define OBJ_DLC_MASK 0x0F
#define OBJ_IDE (1 << 4)
#define OBJ_RTR (1 << 5)

/* How many frames one Poll() may take off the chip in one tick.
 *
 * The limit is not the bus but the SPI. At the divider spi2_setup picks
 * (1.1 MHz) one frame costs about 235 us: status, user address, sixteen bytes
 * and the queue advance. The first version allowed eight, reasoning from what
 * a 500 kbit bus can deliver, and that is 1.9 ms in a 1 ms task. On the bench
 * on 2026-10-04 the MG charger's six frames, sent back to back, did exactly
 * that: cpuload stood at 238 %, which is one 1 ms run that took 2.38 ms.
 *
 * And an overrun costs more than the overrun. Stm32Scheduler clears a task's
 * compare flag after the task returns, so a run that outlasts its period
 * misses the next match and waits for the 16 bit counter to come round, 655
 * ms later - by which time the FIFO is full again and the next run overruns
 * too. The 1 ms task ends up running about once and a half per second.
 *
 * Two frames is 0.5 ms at worst and still drains 2000 frames a second, far
 * above what this bus will carry. A bus that delivers more loses frames out of
 * the FIFO instead of the timing of the 1 ms task, and that is the right way
 * round. */
#define POLL_BUDGET 2

/* The crystal at X3, read off the package: YXC 16.000. Everything about bit
 * timing hangs on this number.
 *
 * Worth knowing: the datasheet lists 40, 20 and 4 MHz crystals, plus an
 * external clock. 16 MHz is not on that menu, so this is Damien's choice and
 * not Microchip's. For classic CAN it is ample - 500 kbit comes out at 32 time
 * quanta - but it does mean the chip runs below the clock the manual
 * recommends for flexible data rate. The bench settles it in a minute:
 * Initialize() only reports success once the chip has actually reached normal
 * mode, and it cannot do that with a dead oscillator. */
#define XTAL_HZ 16000000

Mcp2518Fd::Mcp2518Fd(const Wiring &w)
    : wiring(w), ready(false), busy(false), filtersDirty(false),
      faulted(false), state(STATE_OFF), irqTim4Was(false), irqExtiWas(false),
      deviceId(0), oscSeen(0) {}

/** Bit timing word for NBTCFG.
 *
 * Every field in the register is one less than the real value, so a bit takes
 * 1 + (TSEG1 + 1) + (TSEG2 + 1) time quanta. With the 16 MHz crystal that
 * gives 32 quanta per bit up to 500 kbit, and a sample point at 81 %.
 */
uint32_t Mcp2518Fd::CalcBitTiming(enum baudrates baudrate) {
  uint32_t brp, tseg1, tseg2, sjw;

  switch (baudrate) {
  case Baud125: /* 4 MHz quanta, 32 per bit */
    brp = 3;
    tseg1 = 24;
    tseg2 = 5;
    break;
  case Baud250: /* 8 MHz quanta, 32 per bit */
    brp = 1;
    tseg1 = 24;
    tseg2 = 5;
    break;
  case Baud800: /* 16 MHz quanta, 20 per bit, sample point 80 % */
    brp = 0;
    tseg1 = 14;
    tseg2 = 3;
    break;
  case Baud1000: /* 16 MHz quanta, 16 per bit */
    brp = 0;
    tseg1 = 11;
    tseg2 = 2;
    break;
  case Baud500:
  default: /* 16 MHz quanta, 32 per bit */
    brp = 0;
    tseg1 = 24;
    tseg2 = 5;
    break;
  }

  sjw = tseg2; /* never wider than phase segment 2 */

  return (brp << 24) | (tseg1 << 16) | (tseg2 << 8) | sjw;
}

#ifndef MCP2518FD_HOSTTEST

static void ShortDelay(uint32_t loops) {
  for (volatile uint32_t i = 0; i < loops; i++)
    __asm__("nop");
}

/** THE TRAIL. A number in backup register DR1 that says where in this driver
 * the processor is. Backup registers survive a watchdog reset, so when the
 * board stops and the watchdog pulls it back up, the next boot can read where
 * it was standing. It costs one register write per step.
 *
 *   11..17  Initialize: start, reset sent, oscillator, config mode,
 *           read back and FIFOs, filters, normal mode requested
 *   21..26  Poll: start, FIFO status, user address, message read,
 *           queue advanced, handing the frame on
 *   31      Send
 *
 * Nested use - Poll in the 1 ms interrupt cutting into an Initialize in the
 * main loop - puts the outer number back on the way out, so the outer step is
 * not lost. Outside the driver it reads 0. */
static inline void Trail(uint16_t where) { BKP_DR1 = where; }

namespace {
struct TrailScope {
  uint16_t outer;
  TrailScope() : outer(BKP_DR1 & 0xFFFF) {}
  ~TrailScope() { BKP_DR1 = outer; }
};
} // namespace

uint16_t Mcp2518Fd::TakeTrail() {
  pwr_disable_backup_domain_write_protect();
  uint16_t last = BKP_DR1 & 0xFFFF;
  BKP_DR1 = 0;
  return last;
}

/** THE LOCK. Four contexts talk to this chip over one SPI, and they can cut
 * each other in half.
 *
 * Poll() and Send() run in the 1 ms task, which is the TIM4 interrupt at
 * priority 0, the highest on the board. ConfigureFilters() and SetBaudrate()
 * are reached through Param::Change, either from the main loop (terminal, or
 * the SDO handling in main) or from the CAN receive interrupt at 0xE0 - and
 * TIM4 cuts through both of those, while the CAN interrupt cuts through the
 * main loop.
 *
 * Two things go wrong without a lock. A transfer cut in half leaves the chip
 * select low and writes the tail of one command into the registers of the
 * next, which quietly disables a filter or gives it a mask that throws ids
 * away. And two Sends that interleave read the same free slot from FIFOUA,
 * write over each other in the chip's RAM and then advance the queue twice.
 *
 * Masking interrupts for a whole transfer would also work, but the longest one
 * is an 18 byte write to RAM: about 130 us at 1.125 MHz, which is a tenth of
 * the scheduler's tick. So only the test and set is masked - a handful of
 * instructions - and whoever finds the chip taken gives up its turn instead.
 * That costs a poll or a frame, never a corrupted register.
 */
bool Mcp2518Fd::Claim() {
  bool mine = false;

  cm_disable_interrupts();
  if (!busy) {
    busy = true;
    mine = true;
  }
  cm_enable_interrupts();

  return mine;
}

void Mcp2518Fd::Release() { busy = false; }

/** Chip select, and with it the guard for the bus.
 *
 * Measured on 2026-09-30: pin 9 of IC24 runs to pin 52 of the STM32, so this
 * chip sits on SPI2 - the same three wires as the MCP25625 of CAN3. That
 * driver has no lock of its own and transfers from two places: the EXTI15
 * interrupt, which is its receive line, and the 1 ms task, which is where the
 * Ampera heater sends. Both run at priority 0 and would cut straight through a
 * transfer of ours started in the main loop, leaving two chips with half a
 * command each.
 *
 * So both are held off for the length of one transfer. The longest is the
 * 18 byte write into the chip's RAM, about 130 us: a tick of the scheduler
 * arrives that much late, and CAN3 keeps its frame, because at 500 kbit one
 * takes 230 us and the MCP25625 buffers two.
 *
 * The other direction - CANSPI_Initialize in the main loop when CAN3Speed is
 * set, with our Poll cutting through it - is closed in MCP2515.cpp, by the
 * same two interrupts held off around its chip select. It was left open here
 * at first, and on 2026-10-04 it hung the main loop on every attempt to load
 * a parameter file.
 */
void Mcp2518Fd::Select() {
  irqTim4Was = nvic_get_irq_enabled(NVIC_TIM4_IRQ) != 0;
  irqExtiWas = nvic_get_irq_enabled(NVIC_EXTI15_10_IRQ) != 0;
  nvic_disable_irq(NVIC_TIM4_IRQ);
  nvic_disable_irq(NVIC_EXTI15_10_IRQ);

  /* CAN 3 shares this peripheral and leaves it as it pleases. A byte still
   * sitting in the receive register would shift every answer of ours one place
   * along, so the slate is wiped before chip select goes low. The register is
   * one deep on this part; the count is there so this loop can be read without
   * having to trust that. */
  for (int i = 0; i < 4 && (SPI_SR(wiring.spi) & SPI_SR_RXNE); i++)
    (void)SPI_DR(wiring.spi);

  gpio_clear(wiring.csPort, wiring.csPin);
}

void Mcp2518Fd::Deselect() {
  gpio_set(wiring.csPort, wiring.csPin);

  /* Put both interrupts back as they were found, not simply on. CAN 3 may
   * legitimately be switched off, and switching its receive line on behind its
   * back would arm an interrupt that nobody answers. */
  if (irqExtiWas)
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
  if (irqTim4Was)
    nvic_enable_irq(NVIC_TIM4_IRQ);

  /* A transfer that timed out takes the bus out of service. Poll() and Send()
   * then cost nothing and the rest of the board drives on. */
  if (faulted)
    ready = false;
}

/** One byte over the SPI, with a way out.
 *
 * This replaces libopencm3's spi_xfer, which waits on the receive flag in a
 * while loop with no escape. That is the one place in this driver where the
 * processor can stop for good, and it is not hypothetical: the peripheral is
 * shared with the MCP25625 of CAN 3, which transfers without a lock of its
 * own, and Select() holds off the scheduler for the length of a transfer. Lose
 * one byte there and the board stands still with every lamp still lit, and
 * since Initialize runs before the scheduler is started, the terminal and the
 * web interface go with it.
 *
 * That has not been seen to happen. A bench board that seemed to hang on
 * 2026-10-03 turned out to be fine: the setting had never been saved and the
 * web interface had moved to another address. This is here because the wait
 * had no way out, not because it was caught taking none.
 *
 * So both waits are counted out. One byte takes 32 SPI clocks, and at the
 * divider spi2_setup picks that is a few hundred processor cycles; the budget
 * below is many times that for a healthy chip, and over in a blink for a dead
 * one. The first time-out is final: faulted stays set, every later call
 * returns at once, and Deselect() clears ready.
 */
uint8_t Mcp2518Fd::Xfer(uint8_t out) {
  uint32_t guard;

  if (faulted)
    return 0xFF;

  for (guard = XFER_GUARD; !(SPI_SR(wiring.spi) & SPI_SR_TXE); guard--) {
    if (guard == 0) {
      faulted = true;
      return 0xFF;
    }
  }

  SPI_DR(wiring.spi) = out;

  for (guard = XFER_GUARD; !(SPI_SR(wiring.spi) & SPI_SR_RXNE); guard--) {
    if (guard == 0) {
      faulted = true;
      return 0xFF;
    }
  }

  return SPI_DR(wiring.spi) & 0xFF;
}

void Mcp2518Fd::WriteReg(uint16_t addr, uint32_t value) {
  uint16_t cmd = INSTR_WRITE | (addr & 0x0FFF);

  Select();
  Xfer(cmd >> 8);
  Xfer(cmd & 0xFF);
  Xfer(value & 0xFF);
  Xfer((value >> 8) & 0xFF);
  Xfer((value >> 16) & 0xFF);
  Xfer((value >> 24) & 0xFF);
  Deselect();
}

uint32_t Mcp2518Fd::ReadReg(uint16_t addr) {
  uint16_t cmd = INSTR_READ | (addr & 0x0FFF);
  uint32_t value = 0;

  Select();
  Xfer(cmd >> 8);
  Xfer(cmd & 0xFF);
  for (int i = 0; i < 4; i++)
    value |= ((uint32_t)Xfer(0)) << (8 * i);
  Deselect();

  return value;
}

void Mcp2518Fd::WriteByte(uint16_t addr, uint8_t value) {
  uint16_t cmd = INSTR_WRITE | (addr & 0x0FFF);

  Select();
  Xfer(cmd >> 8);
  Xfer(cmd & 0xFF);
  Xfer(value);
  Deselect();
}

void Mcp2518Fd::WriteBuf(uint16_t addr, const uint8_t *buf, uint8_t len) {
  uint16_t cmd = INSTR_WRITE | (addr & 0x0FFF);

  Select();
  Xfer(cmd >> 8);
  Xfer(cmd & 0xFF);
  for (uint8_t i = 0; i < len; i++)
    Xfer(buf[i]);
  Deselect();
}

void Mcp2518Fd::ReadBuf(uint16_t addr, uint8_t *buf, uint8_t len) {
  uint16_t cmd = INSTR_READ | (addr & 0x0FFF);

  Select();
  Xfer(cmd >> 8);
  Xfer(cmd & 0xFF);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Xfer(0);
  Deselect();
}

bool Mcp2518Fd::EnterMode(uint8_t mode) {
  uint32_t con = ReadReg(REG_CON);

  con &= ~CON_REQOP_MASK;
  con |= ((uint32_t)mode) << CON_REQOP_SHIFT;
  WriteReg(REG_CON, con);

  for (int i = 0; i < 50; i++) {
    uint32_t opmod = (ReadReg(REG_CON) & CON_OPMOD_MASK) >> CON_OPMOD_SHIFT;
    if (opmod == mode)
      return true;
    ShortDelay(1000);
  }

  return false;
}

bool Mcp2518Fd::Initialize(enum baudrates baudrate) {
  ready = false;
  TrailScope scope;
  Trail(11);

  if (!Claim())
    return false;

  /* A fresh attempt gets a fresh SPI. If the wiring was the problem and it has
   * been put right, switching the parameter off and on is enough. */
  faulted = false;

  /* Chip select idles high; the SPI clock and data pins were set up with the
   * peripheral itself, in hwinit. */
  gpio_set(wiring.csPort, wiring.csPin);
  gpio_set_mode(wiring.csPort, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, wiring.csPin);

  if (wiring.stbyPort != 0) {
    gpio_set_mode(wiring.stbyPort, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, wiring.stbyPin);
    /* Standby is active high on the ATA6563 half, so low is awake. If the
     * bus turns out to be dead while the controller itself answers, this is
     * the first line to doubt. */
    gpio_clear(wiring.stbyPort, wiring.stbyPin);
  }

  /* Reset: the command word is sixteen zero bits. */
  Select();
  Xfer(INSTR_RESET >> 8);
  Xfer(INSTR_RESET & 0xFF);
  Deselect();
  ShortDelay(20000);
  Trail(12);

  /* No PLL: the 16 MHz crystal is the system clock. */
  WriteReg(REG_OSC, 0);
  uint32_t osc = 0;
  for (int i = 0; i < 50; i++) {
    osc = ReadReg(REG_OSC);
    if (!RegInvalid(osc) && (osc & OSC_OSCRDY))
      break;
    ShortDelay(1000);
  }
  oscSeen = osc;
  Trail(13);
  deviceId = ReadReg(REG_DEVID) & 0xFF;

  /* Whether anyone is there is judged on OSC, the way the Linux driver does
   * it, and not on DEVID. Linux only prints DEVID, and working chips show up
   * in its log as "MCP2518FD rev0.0" - a DEVID of zero. The first version of
   * this check turned exactly that away, and reported a chip that was very
   * likely fine as NoChip (bench board, 2026-10-03). */
  if (RegInvalid(osc)) {
    state = STATE_NOCHIP;
    Release();
    return false; /* nothing answered on the bus */
  }

  if (!(osc & OSC_OSCRDY)) {
    state = STATE_NOMODE;
    Release();
    return false; /* it answers, but the crystal did not start */
  }

  if (!EnterMode(MODE_CONFIG)) {
    state = STATE_NOMODE;
    Release();
    return false;
  }

  Trail(14);
  /* Written and read back. A device id of its own is not proof: a floating
   * input can read as anything. Only a register that gives back the value we
   * just put in it says there is really a chip on the other end of those four
   * wires, in config mode, and listening. */
  uint32_t timing = CalcBitTiming(baudrate);
  WriteReg(REG_NBTCFG, timing);
  if (ReadReg(REG_NBTCFG) != timing) {
    state = STATE_NOMODE;
    Release();
    return false;
  }

  WriteReg(REG_TSCON, 0); /* no timestamps, so receive objects stay 8 bytes */
  Trail(15);

  /* One transmit FIFO and one receive FIFO, both with an 8 byte payload. */
  WriteReg(REG_FIFOCON(FIFO_TX),
           ((uint32_t)(FIFO_TX_DEPTH - 1) << 24) | (1 << 7));
  WriteReg(REG_FIFOCON(FIFO_RX), ((uint32_t)(FIFO_RX_DEPTH - 1) << 24));

  Trail(16);
  ConfigureFiltersLocked();
  filtersDirty = false;

  Trail(17);
  if (!EnterMode(MODE_NORMAL_20B)) {
    state = STATE_NOMODE;
    Release();
    return false;
  }

  state = STATE_RUN;
  ready = true;
  Release();
  return true;
}

void Mcp2518Fd::SetBaudrate(enum baudrates baudrate) {
  if (!ready || !Claim())
    return;

  ready = false;

  if (EnterMode(MODE_CONFIG)) {
    WriteReg(REG_NBTCFG, CalcBitTiming(baudrate));
    ready = EnterMode(MODE_NORMAL_20B);
  }
  state = ready ? STATE_RUN : STATE_NOMODE;

  Release();
}

/** Take everything and let the callbacks sort it out.
 *
 * The chip has 32 filters, so per-id filtering is possible later. It buys
 * little here: a 500 kbit bus cannot outrun the poll loop, and CanMap and the
 * device drivers already decide per id what they want.
 *
 * Called with the chip already claimed. See Claim() for who can interrupt
 * whom here.
 */
void Mcp2518Fd::ConfigureFiltersLocked() {
  WriteReg(REG_FLTCON(0), 0); /* disable while changing */
  WriteReg(REG_FLTOBJ(0), 0);
  WriteReg(REG_FLTMASK(0), 0); /* every bit "don't care" */
  WriteReg(REG_FLTCON(0), 0x80 | FIFO_RX);
}

/** Reached from RegisterUserMessage and ClearUserMessages, so from whichever
 * context changed a parameter. If the chip is busy this cannot wait and it
 * cannot spin either - the holder may be a lower priority context that will
 * not run again until we return - so it leaves a note for Poll(), which comes
 * round within the millisecond. */
void Mcp2518Fd::ConfigureFilters() {
  if (!Claim()) {
    filtersDirty = true;
    return;
  }

  ConfigureFiltersLocked();
  filtersDirty = false;
  Release();
}

void Mcp2518Fd::Send(uint32_t canId, uint32_t data[2], uint8_t len) {
  if (!ready)
    return;

  TrailScope scope;
  Trail(31);

  if (!Claim())
    return; /* someone else is mid sequence; dropping beats corrupting */

  if ((ReadReg(REG_FIFOSTA(FIFO_TX)) & FIFOSTA_NOTFULL_NOTEMPTY) == 0) {
    Release();
    return; /* transmit FIFO full, drop like the other interfaces do */
  }

  uint32_t ua = ReadReg(REG_FIFOUA(FIFO_TX));
  uint8_t obj[16];
  uint32_t id;
  uint32_t flags = len & OBJ_DLC_MASK;

  if (canId > 0x7FF) {
    uint32_t sid = (canId >> 18) & 0x7FF;
    uint32_t eid = canId & 0x3FFFF;
    id = sid | (eid << 11);
    flags |= OBJ_IDE;
  } else {
    id = canId & 0x7FF;
  }

  obj[0] = id & 0xFF;
  obj[1] = (id >> 8) & 0xFF;
  obj[2] = (id >> 16) & 0xFF;
  obj[3] = (id >> 24) & 0xFF;
  obj[4] = flags & 0xFF;
  obj[5] = (flags >> 8) & 0xFF;
  obj[6] = (flags >> 16) & 0xFF;
  obj[7] = (flags >> 24) & 0xFF;
  for (int i = 0; i < 8; i++)
    obj[8 + i] = (data[i / 4] >> (8 * (i % 4))) & 0xFF;

  WriteBuf(RAM_START + ua, obj, 16);
  WriteByte(REG_FIFOCON(FIFO_TX) + FIFOCON_BYTE1, BYTE1_UINC | BYTE1_TXREQ);
  Release();
}

void Mcp2518Fd::Poll() {
  if (!ready)
    return;

  TrailScope scope;
  Trail(21);

  /* A filter write that had to give way earlier is settled here first. */
  if (filtersDirty && Claim()) {
    ConfigureFiltersLocked();
    filtersDirty = false;
    Release();
  }

  for (int n = 0; n < POLL_BUDGET; n++) {
    uint8_t obj[16];

    /* Claim per message, not for the whole round: HandleRx below runs the
     * device callbacks, and one of those may want to answer over this very
     * bus. Holding the chip across that call would drop its reply. Taking and
     * giving back costs a handful of instructions; what has to stay in one
     * piece is reading the free slot, reading the message and advancing the
     * queue, and that is exactly what is wrapped here. */
    if (!Claim())
      return;

    if ((ReadReg(REG_FIFOSTA(FIFO_RX)) & FIFOSTA_NOTFULL_NOTEMPTY) == 0) {
      Release();
      return;
    }
    Trail(22);

    uint32_t ua = ReadReg(REG_FIFOUA(FIFO_RX));
    Trail(23);

    ReadBuf(RAM_START + ua, obj, 16);
    Trail(24);
    WriteByte(REG_FIFOCON(FIFO_RX) + FIFOCON_BYTE1, BYTE1_UINC);
    Release();
    Trail(25);

    uint32_t id = obj[0] | (obj[1] << 8) | (obj[2] << 16) |
                  ((uint32_t)obj[3] << 24);
    uint32_t flags = obj[4] | (obj[5] << 8);
    uint8_t dlc = flags & OBJ_DLC_MASK;

    if (flags & OBJ_RTR)
      continue; /* nothing here answers remote frames */

    if (flags & OBJ_IDE)
      id = ((id & 0x7FF) << 18) | ((id >> 11) & 0x3FFFF);
    else
      id &= 0x7FF;

    if (dlc > 8)
      dlc = 8;

    uint32_t data[2];
    data[0] = obj[8] | (obj[9] << 8) | (obj[10] << 16) |
              ((uint32_t)obj[11] << 24);
    data[1] = obj[12] | (obj[13] << 8) | (obj[14] << 16) |
              ((uint32_t)obj[15] << 24);

    Trail(26);
    HandleRx(id, data, dlc);
  }
}

#else // MCP2518FD_HOSTTEST

/* Built for the pc test runner: only the bit timing is worth testing without
 * the chip, but the class still needs bodies for its virtuals or the vtable
 * will not link. */
bool Mcp2518Fd::Initialize(enum baudrates) { return false; }
void Mcp2518Fd::SetBaudrate(enum baudrates) {}
void Mcp2518Fd::Send(uint32_t, uint32_t[2], uint8_t) {}
void Mcp2518Fd::Poll() {}
void Mcp2518Fd::ConfigureFilters() {}
void Mcp2518Fd::ConfigureFiltersLocked() {}
bool Mcp2518Fd::Claim() { return false; }
void Mcp2518Fd::Release() {}
uint8_t Mcp2518Fd::Xfer(uint8_t) { return 0xFF; }

#endif // MCP2518FD_HOSTTEST
