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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#endif

/* SPI instructions. The command word is instruction << 12 | address. */
#define INSTR_RESET 0x0000
#define INSTR_WRITE 0x2000
#define INSTR_READ 0x3000

/* Special function registers */
#define REG_OSC 0xE00
/* IOCON, 0xE04. Not written yet, and that may turn out to be wrong: the
 * ATA6563 transceiver inside the MCP251863 pulls its own standby pin up, so a
 * floating standby leaves the bus silent. Many designs tie that pin to the
 * controller's GPIO0/INT0 instead of to the microcontroller, and then IOCON
 * needs XSTBYEN set and TRIS0 cleared. Which it is here comes out of the same
 * measurement as the SPI bus and the chip select. */
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

/* How many frames one Poll() may take off the chip. A 500 kbit bus cannot
 * deliver more than about four frames per millisecond, so eight is headroom
 * without ever hogging the 1 ms task. */
#define POLL_BUDGET 8

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
    : wiring(w), ready(false), busy(false), filtersDirty(false), deviceId(0) {}

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

void Mcp2518Fd::Select() { gpio_clear(wiring.csPort, wiring.csPin); }

void Mcp2518Fd::Deselect() { gpio_set(wiring.csPort, wiring.csPin); }

void Mcp2518Fd::WriteReg(uint16_t addr, uint32_t value) {
  uint16_t cmd = INSTR_WRITE | (addr & 0x0FFF);

  Select();
  spi_xfer(wiring.spi, cmd >> 8);
  spi_xfer(wiring.spi, cmd & 0xFF);
  spi_xfer(wiring.spi, value & 0xFF);
  spi_xfer(wiring.spi, (value >> 8) & 0xFF);
  spi_xfer(wiring.spi, (value >> 16) & 0xFF);
  spi_xfer(wiring.spi, (value >> 24) & 0xFF);
  Deselect();
}

uint32_t Mcp2518Fd::ReadReg(uint16_t addr) {
  uint16_t cmd = INSTR_READ | (addr & 0x0FFF);
  uint32_t value = 0;

  Select();
  spi_xfer(wiring.spi, cmd >> 8);
  spi_xfer(wiring.spi, cmd & 0xFF);
  for (int i = 0; i < 4; i++)
    value |= ((uint32_t)spi_xfer(wiring.spi, 0)) << (8 * i);
  Deselect();

  return value;
}

void Mcp2518Fd::WriteByte(uint16_t addr, uint8_t value) {
  uint16_t cmd = INSTR_WRITE | (addr & 0x0FFF);

  Select();
  spi_xfer(wiring.spi, cmd >> 8);
  spi_xfer(wiring.spi, cmd & 0xFF);
  spi_xfer(wiring.spi, value);
  Deselect();
}

void Mcp2518Fd::WriteBuf(uint16_t addr, const uint8_t *buf, uint8_t len) {
  uint16_t cmd = INSTR_WRITE | (addr & 0x0FFF);

  Select();
  spi_xfer(wiring.spi, cmd >> 8);
  spi_xfer(wiring.spi, cmd & 0xFF);
  for (uint8_t i = 0; i < len; i++)
    spi_xfer(wiring.spi, buf[i]);
  Deselect();
}

void Mcp2518Fd::ReadBuf(uint16_t addr, uint8_t *buf, uint8_t len) {
  uint16_t cmd = INSTR_READ | (addr & 0x0FFF);

  Select();
  spi_xfer(wiring.spi, cmd >> 8);
  spi_xfer(wiring.spi, cmd & 0xFF);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = spi_xfer(wiring.spi, 0);
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

  if (!Claim())
    return false;

  /* Chip select idles high; the SPI clock and data pins were set up with the
   * peripheral itself, in hwinit. */
  gpio_set(wiring.csPort, wiring.csPin);
  gpio_set_mode(wiring.csPort, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, wiring.csPin);

  if (wiring.stbyPort != 0) {
    gpio_set_mode(wiring.stbyPort, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, wiring.stbyPin);
    gpio_clear(wiring.stbyPort, wiring.stbyPin); /* transceiver awake */
  }

  /* Reset: the command word is sixteen zero bits. */
  Select();
  spi_xfer(wiring.spi, INSTR_RESET >> 8);
  spi_xfer(wiring.spi, INSTR_RESET & 0xFF);
  Deselect();
  ShortDelay(20000);

  /* No PLL: the 16 MHz crystal is the system clock. */
  WriteReg(REG_OSC, 0);
  for (int i = 0; i < 50; i++) {
    if (ReadReg(REG_OSC) & OSC_OSCRDY)
      break;
    ShortDelay(1000);
  }

  deviceId = ReadReg(REG_DEVID) & 0xFF;
  if (deviceId == 0 || deviceId == 0xFF) {
    Release();
    return false; /* nothing answered on the bus */
  }

  if (!EnterMode(MODE_CONFIG)) {
    Release();
    return false;
  }

  WriteReg(REG_NBTCFG, CalcBitTiming(baudrate));
  WriteReg(REG_TSCON, 0); /* no timestamps, so receive objects stay 8 bytes */

  /* One transmit FIFO and one receive FIFO, both with an 8 byte payload. */
  WriteReg(REG_FIFOCON(FIFO_TX),
           ((uint32_t)(FIFO_TX_DEPTH - 1) << 24) | (1 << 7));
  WriteReg(REG_FIFOCON(FIFO_RX), ((uint32_t)(FIFO_RX_DEPTH - 1) << 24));

  ConfigureFiltersLocked();
  filtersDirty = false;

  if (!EnterMode(MODE_NORMAL_20B)) {
    Release();
    return false;
  }

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

    uint32_t ua = ReadReg(REG_FIFOUA(FIFO_RX));

    ReadBuf(RAM_START + ua, obj, 16);
    WriteByte(REG_FIFOCON(FIFO_RX) + FIFOCON_BYTE1, BYTE1_UINC);
    Release();

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

#endif // MCP2518FD_HOSTTEST
