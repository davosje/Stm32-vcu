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
 *
 * Drives the VW MEB HV PTC coolant heater (J848, 1EE 963 231) over LIN.
 * Protocol: VWMebHeaterFrames.h.
 *
 * Heater = 7 (VWMeb). The heater runs while HeatReq is on, at HeatPercnt
 * duty, ramped up at 2 %/s like the J840 does. MebHtBits picks the bit
 * layout of the command frame; see VWMebHeaterFrames.h.
 *
 * Shown values: tmpheater (heater), tmpheatin, tmpheatout, udcheater (HV),
 * powerheater (HV voltage x current) and u12heater (12 V as the heater
 * sees it). u12heater reads 0 when the heater has not answered for a
 * second, so it doubles as a "LIN is alive" indicator on the bench.
 */
#ifndef VWMEBHEATER_H
#define VWMEBHEATER_H

#include "linbus.h"
#include <cstdint>
#include <heater.h>

class vwMebHeater : public Heater {
public:
  void SetTargetTemperature(float temp) { (void)temp; } // not used
  void SetPower(uint16_t power, bool HeatReq);
  void SetLinInterface(LinBus *l);
  void DeInit();

private:
  void ShowFeedback(const uint8_t *data);
  void ClearFeedback();

  LinBus *lin = nullptr;
  uint8_t tick = 0;
  bool readNext = true;
  uint16_t dutyTenths = 0;
  uint8_t silentSteps = 0;
};

#endif // VWMEBHEATER_H
