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

#ifndef MGGEN2CHARGER_H
#define MGGEN2CHARGER_H

/* MG ZS EV gen 2 on-board charger, AC charging side (hybrid CAN).
 *
 * Put this charger on the bus wired to its hybrid CAN pins (BY247 A4/B4),
 * set as ChargerCan. The DC-DC converter in the
 * same unit is on the PT bus and is a separate choice: DCdc_Type = MGgen2.
 * Byte layouts and where they come from: MGgen2Frames.h.
 *
 * How a charge runs:
 *
 *   Asleep        silent, until the charger speaks (it wakes itself when a
 *                 plug goes in) or charge mode is requested
 *   Waking        0.7 s, with the wake pulse the car sends
 *   Standby       charger awake, charge port reported active
 *   Charging      after precharge: current request ramps up as in the car
 *   Stopping      stop request sent, current request ramps down; charge mode
 *                 is held until the charger has let go of its current, so the
 *                 contactors never open under load
 *   ShuttingDown  6 s of HV shutdown state, then back to Standby
 *
 * The MG runs the AC pilot itself. A charge starts on the charger's own start
 * signal (0x33B bit 7 with a valid PWM duty), not on a charge interface: with
 * a LIM fitted for DC charging, the LIM would be reporting on a pilot line
 * the MG owns.
 *
 * The DC current asked for is the lowest of: 51.1 A (what the car asks),
 * Pwrspnt / udc, and BMS_ChargeLim when a BMS reports one. The EVSE limits
 * the charger further through the pilot duty.
 */

#include "chargerhw.h"
#include "params.h"
#include <stdint.h>

class MGgen2Charger : public Chargerhw {
public:
  void DecodeCAN(int id, uint32_t data[2]) override;
  void Task10Ms() override;
  void Task100Ms() override;
  bool ControlCharge(bool RunCh, bool ACReq) override;
  void SetCanInterface(CanHardware *c) override;
  void DeInit() override;

private:
  enum class Phase {
    Asleep,
    Waking,
    Standby,
    Charging,
    Stopping,
    ShuttingDown
  };

  void Enter(Phase next);
  void UpdatePhase(int opmode);
  void Publish();
  bool Alive() const;
  bool ChargerReady() const;
  bool Quiet() const;
  float TargetAmps() const;
  uint8_t HvState(int opmode) const;
  uint8_t Vehicle() const;

  Phase phase = Phase::Asleep;
  uint16_t phaseTicks = 0;
  uint16_t restTicks = 0;
  uint8_t prechargeTicks = 0;
  uint8_t silentTicks = UINT8_MAX;
  uint8_t ticks10 = 0;
  bool wantCharge = false;

  bool pilotReady = false;
  uint8_t pilotDuty = 0;
  float dcAmps = 0;
  float acAmps[3] = {0, 0, 0};
  float acVolts[3] = {0, 0, 0};

  uint16_t currentCounts = 0;
  uint16_t modeRamp = 0;
  uint16_t roll = 0;
  uint8_t counter297 = 0;
  uint8_t counter394 = 0;
  uint8_t counter39B = 0;
};

#endif // MGGEN2CHARGER_H
