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

#include "MGgen2DCDC.h"
#include "MGgen2Frames.h"

using namespace MGgen2Protocol;

// PT CAN identifiers
static const int ID_CONTROL = 0x19C;
static const int ID_VEHICLE = 0x297;
static const int ID_PACK = 0x29B;
static const int ID_OUTPUT = 0x39F;

static const uint8_t TICKS_PER_50MS = 5;   // Task10Ms
static const uint16_t SHUTDOWN_TICKS = 60; // 6 s of HV_SHUTDOWN, Task100Ms
static const uint16_t REST_TICKS = 1000;   // 100 s, then quiet, Task100Ms

static bool HvUp(int opmode) {
  return opmode == MOD_RUN || opmode == MOD_CHARGE;
}

void MGgen2DCDC::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(ID_OUTPUT);
}

void MGgen2DCDC::DeInit() {
  counter = 0;
  ticks10 = 0;
  shutdownTicks = 0;
  restTicks = 0;
  hvWasUp = false;
}

void MGgen2DCDC::DecodeCAN(int id, uint8_t *data) {
  if (id == ID_OUTPUT) {
    Param::SetFloat(Param::U12V, LvVolts(data));
    Param::SetFloat(Param::I12V, LvAmps(data));
  }
}

void MGgen2DCDC::Task100Ms() {
  int opmode = Param::GetInt(Param::opmode);
  bool hvUp = HvUp(opmode);

  if (hvWasUp && !hvUp)
    shutdownTicks = SHUTDOWN_TICKS;
  else if (shutdownTicks > 0)
    shutdownTicks--;
  hvWasUp = hvUp;

  if (hvUp || opmode == MOD_PRECHARGE)
    restTicks = REST_TICKS;
  else if (restTicks > 0)
    restTicks--;
}

void MGgen2DCDC::Task10Ms() {
  if (restTicks == 0)
    return; // nothing to supply, keep the bus quiet

  int opmode = Param::GetInt(Param::opmode);
  uint8_t bytes[8];
  Build19C(bytes, HvUp(opmode), counter++);
  can->Send(ID_CONTROL, bytes, 8);

  if (++ticks10 < TICKS_PER_50MS)
    return;
  ticks10 = 0;

  uint8_t hv = HvState(opmode);
  Build297PT(bytes, hv);
  can->Send(ID_VEHICLE, bytes, 8);
  Build29BPT(bytes,
             (HvActive(hv) || hv == HV_PRECHARGE) ? PACK_HV : PACK_AWAKE);
  can->Send(ID_PACK, bytes, 8);
}

uint8_t MGgen2DCDC::HvState(int opmode) const {
  switch (opmode) {
  case MOD_RUN:
    return HV_ON_IGNITION_ON;
  case MOD_PRECHARGE:
    return HV_PRECHARGE;
  case MOD_CHARGE:
    return HV_ON_IGNITION_OFF;
  default:
    return shutdownTicks > 0 ? HV_SHUTDOWN : HV_STANDBY;
  }
}
