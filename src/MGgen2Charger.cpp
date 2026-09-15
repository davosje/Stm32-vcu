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

#include "MGgen2Charger.h"
#include "MGgen2Frames.h"

using namespace MGgen2Protocol;

// Hybrid CAN identifiers, sent by us
static const int ID_VEHICLE = 0x297;
static const int ID_PACK = 0x29B;
static const int ID_COMMAND = 0x29C;
static const int ID_MODE = 0x33F;
static const int ID_394 = 0x394;
static const int ID_396 = 0x396;
static const int ID_39A = 0x39A;
static const int ID_39B = 0x39B;
// Hybrid CAN identifiers, sent by the charger
static const int ID_STATUS = 0x324;
static const int ID_PILOT = 0x33B;
static const int ID_DC = 0x33D;
static const int ID_AC = 0x491;

// Timing, in ticks of the task that counts them
static const uint8_t TICKS_PER_50MS = 5;      // Task10Ms
static const uint8_t ALIVE_TICKS = 10;        // 1 s of silence: charger gone
static const uint8_t WAKING_TICKS = 7;        // 0.7 s before standby
static const uint8_t WAKE_PULSE_TICKS = 5;    // 0.5 s wake pulse
static const uint8_t WAKE_STATE_TICKS = 3;    // 0.3 s of HV_WAKE first
static const uint8_t STOP_PULSE_TICKS = 9;    // 0.9 s stop request
static const uint8_t STOP_TIMEOUT_TICKS = 30; // stop waiting after 3 s
static const uint8_t SHUTDOWN_TICKS = 60;     // 6 s of HV_SHUTDOWN
static const uint16_t REST_TICKS = 1000;      // 100 s of rest frames

static const float MAX_DC_AMPS = 51.1f;    // what the car always asks for
static const float STOPPED_AMPS = 1.0f;    // below this, contactors may open
static const float MIN_PACK_VOLTS = 50.0f; // no current without pack voltage

void MGgen2Charger::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(ID_STATUS);
  can->RegisterUserMessage(ID_PILOT);
  can->RegisterUserMessage(ID_DC);
  can->RegisterUserMessage(ID_AC);
}

void MGgen2Charger::DeInit() {
  phase = Phase::Asleep;
  phaseTicks = 0;
  restTicks = 0;
  silentTicks = UINT8_MAX;
  wantCharge = false;
  currentCounts = 0;
  modeRamp = 0;
}

void MGgen2Charger::DecodeCAN(int id, uint32_t data[2]) {
  const uint8_t *bytes = (const uint8_t *)data;
  switch (id) {
  case ID_STATUS:
    silentTicks = 0;
    break;
  case ID_PILOT:
    silentTicks = 0;
    pilotReady = PilotReady(bytes);
    pilotDuty = PilotDuty(bytes);
    break;
  case ID_DC:
    dcAmps = DcAmps(bytes);
    break;
  case ID_AC:
    AcPhases(bytes, acAmps, acVolts);
    break;
  }
}

bool MGgen2Charger::ControlCharge(bool RunCh, bool ACReq) {
  // The MG owns the AC pilot, so the charge interface's view is not used.
  (void)ACReq;
  wantCharge = RunCh && ChargerReady();
  bool hvUp = Param::GetInt(Param::opmode) == MOD_CHARGE;

  // Decide here and not in Task100Ms: the moment this returns false, the VCU
  // leaves charge mode and opens the contactors, current or no current.
  if (phase == Phase::Charging && !wantCharge)
    Enter(Phase::Stopping);

  // While stopping, hold charge mode as long as high voltage is up. Only
  // ever hold it: this never starts charge mode on its own.
  if (phase == Phase::Stopping)
    return hvUp;
  // And never start a new charge while the last one is still shutting down.
  if (phase == Phase::ShuttingDown)
    return false;
  return wantCharge;
}

void MGgen2Charger::Task10Ms() {
  if (++ticks10 < TICKS_PER_50MS)
    return;
  ticks10 = 0;
  if (Quiet())
    return;

  int opmode = Param::GetInt(Param::opmode);
  uint8_t hv = HvState(opmode);
  if (phase == Phase::Waking && phaseTicks < WAKE_PULSE_TICKS)
    hv |= HV_WAKE_PULSE;

  uint8_t bytes[8];
  Build297Hybrid(bytes, Vehicle(), hv, counter297++);
  can->Send(ID_VEHICLE, bytes, 8);

  PackStage pack = PACK_AWAKE;
  if (phase == Phase::Asleep)
    pack = PACK_SLEEP;
  else if (opmode == MOD_PRECHARGE || opmode == MOD_CHARGE ||
           phase == Phase::ShuttingDown)
    pack = PACK_HV;
  Build29BHybrid(bytes, pack);
  can->Send(ID_PACK, bytes, 8);
}

void MGgen2Charger::Task100Ms() {
  if (silentTicks < UINT8_MAX)
    silentTicks++;
  int opmode = Param::GetInt(Param::opmode);
  UpdatePhase(opmode);
  Publish();
  if (Quiet())
    return;

  uint16_t volts = VoltsToCounts(Param::GetFloat(Param::Voltspnt));
  Command command = CMD_SLEEP;
  Command mode = CMD_SLEEP;

  switch (phase) {
  case Phase::Asleep:
    break;
  case Phase::Waking:
    command = CMD_STANDBY;
    mode = CMD_AWAKE;
    break;
  case Phase::Standby:
    command = CMD_STANDBY;
    mode = CMD_STANDBY;
    break;
  case Phase::Charging:
    currentCounts =
        Ramp(currentCounts, AmpsToCounts(TargetAmps()), CURRENT_RAMP_STEP);
    modeRamp = Ramp(modeRamp, MODE_RAMP_MAX, MODE_RAMP_STEP);
    command = CMD_CHARGE;
    mode = CMD_CHARGE;
    break;
  case Phase::Stopping:
    currentCounts = Ramp(currentCounts, 0, CURRENT_RAMP_STEP);
    command = CMD_STOPPING;
    // The car kept reporting "charging" for as long as the stop request.
    mode = phaseTicks < STOP_PULSE_TICKS ? CMD_CHARGE : CMD_STOPPING;
    break;
  case Phase::ShuttingDown:
    command = CMD_SHUTDOWN;
    mode = CMD_SHUTDOWN;
    break;
  }

  bool awake = phase != Phase::Asleep;
  bool hvActive = HvActive(HvState(opmode)) || opmode == MOD_PRECHARGE;
  uint8_t bytes[8];

  Build29C(bytes, command, currentCounts, volts);
  can->Send(ID_COMMAND, bytes, 8);
  Build33F(bytes, mode, modeRamp, volts);
  can->Send(ID_MODE, bytes, 8);
  Build394(bytes, awake, counter394++);
  can->Send(ID_394, bytes, 8);
  Build396(bytes, hvActive);
  can->Send(ID_396, bytes, 8);
  roll += ROLL_STEP;
  Build39A(bytes, roll);
  can->Send(ID_39A, bytes, 8);
  Build39BHybrid(bytes, counter39B++);
  can->Send(ID_39B, bytes, 8);
}

void MGgen2Charger::Enter(Phase next) {
  phase = next;
  phaseTicks = 0;
}

void MGgen2Charger::UpdatePhase(int opmode) {
  bool hvRequested = opmode == MOD_PRECHARGE || opmode == MOD_CHARGE;

  if (opmode != MOD_PRECHARGE)
    prechargeTicks = 0;
  else if (prechargeTicks < UINT8_MAX)
    prechargeTicks++;
  if (phaseTicks < UINT16_MAX)
    phaseTicks++;

  switch (phase) {
  case Phase::Asleep:
    if (Alive() || hvRequested)
      Enter(Phase::Waking);
    else if (restTicks > 0)
      restTicks--;
    break;
  case Phase::Waking:
    if (phaseTicks >= WAKING_TICKS)
      Enter(Phase::Standby);
    break;
  case Phase::Standby:
    if (opmode == MOD_CHARGE) {
      currentCounts = 0;
      modeRamp = 0;
      Enter(Phase::Charging);
    } else if (!Alive() && !hvRequested) {
      restTicks = REST_TICKS;
      Enter(Phase::Asleep);
    }
    break;
  case Phase::Charging:
    // High voltage went away underneath us (fault, precharge, user).
    if (opmode != MOD_CHARGE)
      Enter(Phase::ShuttingDown);
    break;
  case Phase::Stopping:
    if (opmode != MOD_CHARGE)
      Enter(Phase::ShuttingDown);
    else if ((phaseTicks >= STOP_PULSE_TICKS && dcAmps < STOPPED_AMPS) ||
             phaseTicks >= STOP_TIMEOUT_TICKS)
      Enter(Phase::ShuttingDown);
    break;
  case Phase::ShuttingDown:
    if (phaseTicks >= SHUTDOWN_TICKS)
      Enter(Phase::Standby);
    break;
  }
}

void MGgen2Charger::Publish() {
  if (!Alive())
    return;
  // PilotLim is also written by charge interfaces (i3LIM, CPC, Foccci). Only
  // take it over while the MG itself sees a valid AC pilot: then the MG is the
  // better source, being the one wired to it.
  float pilotAmps = PilotAmps(pilotDuty);
  if (pilotAmps > 0)
    Param::SetFloat(Param::PilotLim, pilotAmps);
  // AC_Volts and AC_Amps are one value each; they show phase L1.
  Param::SetFloat(Param::AC_Volts, acVolts[0]);
  Param::SetFloat(Param::AC_Amps, acAmps[0]);
}

bool MGgen2Charger::Alive() const { return silentTicks < ALIVE_TICKS; }

bool MGgen2Charger::ChargerReady() const {
  return Alive() && pilotReady && PilotAmps(pilotDuty) > 0;
}

bool MGgen2Charger::Quiet() const {
  return phase == Phase::Asleep && restTicks == 0;
}

float MGgen2Charger::TargetAmps() const {
  float udc = Param::GetFloat(Param::udc);
  if (udc < MIN_PACK_VOLTS)
    return 0;
  float amps = Param::GetFloat(Param::Pwrspnt) / udc;
  if (amps > MAX_DC_AMPS)
    amps = MAX_DC_AMPS;
  float bms = Param::GetFloat(Param::BMS_ChargeLim);
  if (bms > 0 && amps > bms)
    amps = bms;
  return amps;
}

uint8_t MGgen2Charger::HvState(int opmode) const {
  if (phase == Phase::ShuttingDown)
    return HV_SHUTDOWN;
  switch (opmode) {
  case MOD_PRECHARGE:
    return prechargeTicks <= WAKE_STATE_TICKS ? HV_WAKE : HV_PRECHARGE;
  case MOD_CHARGE:
    return HV_ON_IGNITION_OFF;
  case MOD_RUN:
    return HV_ON_IGNITION_ON;
  default:
    return HV_STANDBY;
  }
}

uint8_t MGgen2Charger::Vehicle() const {
  if (phase == Phase::Asleep)
    return VEHICLE_ASLEEP;
  if (phase == Phase::Stopping && phaseTicks < STOP_PULSE_TICKS)
    return VEHICLE_AWAKE | VEHICLE_STOP_REQUEST;
  return VEHICLE_AWAKE;
}
