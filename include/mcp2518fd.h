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
#ifndef MCP2518FD_H
#define MCP2518FD_H

#include "canhardware.h"
#include <stdint.h>

/** \brief Fourth CAN interface: the MCP2518FD inside IC24 (MCP251863)
 *
 * The ZombieVerter V1.3 carries an MCP251863 - an MCP2518FD controller and an
 * ATA6563 transceiver in one SSOP-28 package - on CONN3, clocked by X3, a
 * 16 MHz crystal. The firmware never used it. This driver makes it a normal
 * CanHardware, so every device that is mapped with a CAN_DEV parameter can be
 * put on it.
 *
 * The chip can do CAN FD, but we run it in classic CAN 2.0B: that is what the
 * chargers, LIMs and inverters on these projects speak. Flexible data rate is
 * a later job and only needs DBTCFG plus the FDF bit.
 *
 * Receiving is polled, not interrupt driven: Poll() drains the receive FIFO
 * and hands every frame to HandleRx(). Call it from the 1 ms task. That keeps
 * the interrupt line out of the first version - EXTI15 is already taken by
 * CAN3 - at the price of a millisecond of latency and a FIFO deep enough to
 * hold what arrives in between.
 */
class Mcp2518Fd : public CanHardware {
public:
  /** Pins and bus. Everything the board-specific part decides lives here. */
  struct Wiring {
    uint32_t spi;       //!< SPI peripheral base, already initialised in mode 0
    uint32_t csPort;    //!< chip select, driven low for every transfer
    uint16_t csPin;
    uint32_t stbyPort;  //!< transceiver standby, driven low to talk; 0 = unused
    uint16_t stbyPin;
  };

  Mcp2518Fd(const Wiring &wiring);

  /** Reset the chip, configure it and go to normal mode.
   * \return true if the chip answered and reached normal mode */
  bool Initialize(enum baudrates baudrate);
  bool IsReady() const { return ready; }

  void SetBaudrate(enum baudrates baudrate);
  void Send(uint32_t canId, uint32_t data[2], uint8_t len);

  /** Drain the receive FIFO. Call periodically. */
  void Poll();

  /** Device id of the chip, for diagnosis. 0 means nothing answered. */
  uint8_t GetDeviceId() const { return deviceId; }

  /** How far the chip got. The numbers are what the web interface shows as
   * CANFDState, so they must stay in step with CANFDSTATE in param_prj.h. */
  enum State {
    STATE_OFF = 0,     //!< never started
    STATE_RUN = 1,     //!< answered, read back what we wrote, normal mode
    STATE_NOCHIP = 2,  //!< nothing answered: wiring, power or chip select
    STATE_NOMODE = 3,  //!< it answered, but a register or a mode did not take
    STATE_SPIFAULT = 4 //!< a transfer timed out; out of service until reset
  };

  /** A fault outranks whatever Initialize last found, because it can come
   * later, in the middle of running. */
  State GetState() const {
    return faulted ? STATE_SPIFAULT : (State)state;
  }

  /** Bit timing for a baud rate, given the crystal on the board.
   * Split out so it can be tested on a pc without the hardware. */
  static uint32_t CalcBitTiming(enum baudrates baudrate);

private:
  void ConfigureFilters();
  void ConfigureFiltersLocked();

  /** Take the chip for the length of one sequence of transfers.
   * \return false when another context is already talking to it */
  bool Claim();
  void Release();

  bool EnterMode(uint8_t mode);
  void WriteReg(uint16_t addr, uint32_t value);
  uint32_t ReadReg(uint16_t addr);
  void WriteByte(uint16_t addr, uint8_t value);
  void WriteBuf(uint16_t addr, const uint8_t *buf, uint8_t len);
  void ReadBuf(uint16_t addr, uint8_t *buf, uint8_t len);
  void Select();
  void Deselect();

  /** One byte over the SPI, with a bounded wait.
   * \return the byte received, or 0xFF once the driver has faulted */
  uint8_t Xfer(uint8_t out);

  Wiring wiring;
  volatile bool ready;        //!< initialised and in normal mode
  volatile bool busy;         //!< a sequence of transfers is running
  volatile bool filtersDirty; //!< filters could not be written, retry in Poll
  volatile bool faulted;      //!< the SPI stopped answering, out of service
  volatile uint8_t state;     //!< a State, as far as Initialize got
  bool irqTim4Was;            //!< scheduler interrupt state before Select
  bool irqExtiWas;            //!< CAN3 receive state before Select
  uint8_t deviceId;
};

#endif // MCP2518FD_H
