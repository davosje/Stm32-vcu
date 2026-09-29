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

/* Bit timing is the one part of the MCP2518FD driver that can be checked
 * without the chip, and it is the part that quietly ruins a bus when it is
 * wrong: a sample point in the wrong place gives a bus that works on the bench
 * and drops frames in the vehicle. So decode the register back into a bit rate
 * and a sample point, and hold it against the crystal on the board.
 */

#include "mcp2518fd.h"
#include "test.h"
#include <stdio.h>

#define XTAL_HZ 16000000

static void Decode(uint32_t nbtcfg, unsigned &bitrate, unsigned &samplePoint) {
  unsigned brp = ((nbtcfg >> 24) & 0xFF) + 1;
  unsigned tseg1 = ((nbtcfg >> 16) & 0xFF) + 1;
  unsigned tseg2 = ((nbtcfg >> 8) & 0x7F) + 1;
  unsigned tq = 1 + tseg1 + tseg2;

  bitrate = XTAL_HZ / brp / tq;
  samplePoint = (1 + tseg1) * 1000 / tq; /* per mille */
}

static void CheckOne(enum CanHardware::baudrates baudrate, unsigned expected) {
  uint32_t nbtcfg = Mcp2518Fd::CalcBitTiming(baudrate);
  unsigned bitrate, samplePoint;

  Decode(nbtcfg, bitrate, samplePoint);

  ASSERT(bitrate == expected);
  /* CiA recommends 75 % to 90 %; anything outside that is a bug, not taste */
  ASSERT(samplePoint >= 750);
  ASSERT(samplePoint <= 900);

  /* The jump width may never exceed phase segment 2 */
  unsigned tseg2 = ((nbtcfg >> 8) & 0x7F) + 1;
  unsigned sjw = (nbtcfg & 0x7F) + 1;
  ASSERT(sjw <= tseg2);
}

void Mcp2518FdTest::RunTest() {
  CheckOne(CanHardware::Baud125, 125000);
  CheckOne(CanHardware::Baud250, 250000);
  CheckOne(CanHardware::Baud500, 500000);
  CheckOne(CanHardware::Baud800, 800000);
  CheckOne(CanHardware::Baud1000, 1000000);

  /* 500 kbit is what everything in this vehicle speaks, so pin it down
   * exactly: 32 quanta straight off the 16 MHz crystal, sampling at 81 %. */
  uint32_t nbtcfg = Mcp2518Fd::CalcBitTiming(CanHardware::Baud500);
  ASSERT(((nbtcfg >> 24) & 0xFF) == 0);  /* prescaler 1 */
  ASSERT(((nbtcfg >> 16) & 0xFF) == 24); /* phase 1 is 25 quanta */
  ASSERT(((nbtcfg >> 8) & 0x7F) == 5);   /* phase 2 is 6 quanta */
}
