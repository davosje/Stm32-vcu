/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 davosje
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

#ifndef MGGEN2DCDC_H
#define MGGEN2DCDC_H

/* DC-DC converter inside the MG ZS EV gen 2 on-board charger (PT CAN).
 *
 * Put this on the bus the charger's PT pins (5/6) are wired to (DCDCCan),
 * which must not be the bus of MGgen2Charger: see MGgen2Frames.h.
 *
 * The converter runs on one message, 0x19C every 10 ms. The PT copies of
 * 0x297 and 0x29B go along with it: in the captures the car kept both going
 * throughout, while every other PT message could drop out without the
 * converter noticing. It runs whenever high voltage is up, in RUN and in
 * CHARGE, and the bus falls quiet 100 s after high voltage goes down.
 *
 * Output voltage and current are read back from 0x39F into U12V and I12V.
 * The setpoint (DCSetPnt) is not used yet: 0x19C D3:D4 looks like one, but
 * that is not proven, so the value the car sends (13.44 V) is kept.
 */

#include "dcdc.h"
#include "params.h"
#include <stdint.h>

class MGgen2DCDC : public DCDC {
public:
  void DecodeCAN(int id, uint8_t *data) override;
  void DeInit() override;
  void Task10Ms() override;
  void Task100Ms() override;
  void SetCanInterface(CanHardware *c) override;

private:
  uint8_t HvState(int opmode) const;

  uint8_t counter = 0;
  uint8_t ticks10 = 0;
  uint16_t shutdownTicks = 0;
  uint16_t restTicks = 0;
  bool hvWasUp = false;
};

#endif // MGGEN2DCDC_H
