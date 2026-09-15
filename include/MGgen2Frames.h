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

#ifndef MGGEN2FRAMES_H
#define MGGEN2FRAMES_H

/* CAN frames of the MG ZS EV gen 2 on-board charger with built-in DC-DC
 * converter (SAIC EP2CCU1130A, MG part number 11236823).
 *
 * Nothing in here touches parameters, CAN hardware or timers. Each function
 * turns values into the eight bytes of one frame, or one frame back into
 * values. That is what makes the protocol testable on a PC: see
 * test/test_mggen2.cpp, which checks every builder against frames taken from
 * a real car.
 *
 * Source: SavvyCAN captures of an MG ZS EV by Lars (EVcreate), published in
 * https://github.com/damienmaguire/MG-EV-Charger under CANLogs/Gen2/Lars.
 *
 * The unit is connected to two buses and needs both:
 *
 *   Hybrid CAN  (LV connector BY247, A4/B4)  AC charging  -> MGgen2Charger
 *   PT CAN      (LV connector BY247, C1/C2)  DC-DC        -> MGgen2DCDC
 *
 * 0x297, 0x29B and 0x39B exist on both buses with different payloads, so the
 * two buses must not be joined.
 *
 * BY247 is the 32-way LV connector of the 11 kW unit (rows A to H). The
 * 12-way BY400 found in older notes belongs to the 6.6 kW charger without a
 * DC-DC converter and has a different pinout.
 */

#include <stdint.h>

namespace MGgen2Protocol {

/* Several fields are 16 bits wide: 6 fixed bits on top of a 10 bit value. */
const uint16_t VALUE_MASK = 0x3FF;

/* 0x29C D3:D4, the DC current request. 20 counts per amp; 1022 is the highest
 * valid value, 1023 means invalid. The car ramps it by 100 counts (5 A) every
 * 100 ms. */
const uint16_t CURRENT_COUNTS_PER_AMP = 20;
const uint16_t CURRENT_COUNTS_MAX = 1022;
const uint16_t CURRENT_RAMP_STEP = 100;

/* 0x33F D6:D7 ramps from 0 to 1023 in steps of 40 per 100 ms while a charge
 * starts. What it means is not known; the car always does it, so we do too. */
const uint16_t MODE_RAMP_MAX = 1023;
const uint16_t MODE_RAMP_STEP = 40;

/* 0x39A D5:D6: the upper 10 bits keep counting, 63 per 100 ms. */
const uint16_t ROLL_STEP = 63;

/* Voltage limit in 0x29C D7:D8 and 0x33F D1:D2. */
const float VOLTS_PER_COUNT = 0.02f;

/* Vehicle high voltage state, 0x297 D2 on both buses. */
enum HvState : uint8_t {
  HV_STANDBY = 0x01,
  HV_ON_IGNITION_ON = 0x03,
  HV_WAKE = 0x04,
  HV_ON_IGNITION_OFF = 0x06,
  HV_SHUTDOWN = 0x0A,
  HV_PRECHARGE = 0x0C
};

/* OR'ed into 0x297 D2 for about 0.5 s every time the vehicle wakes up. */
const uint8_t HV_WAKE_PULSE = 0x20;

/* 0x297 D1 on the hybrid bus. */
const uint8_t VEHICLE_ASLEEP = 0x00;
const uint8_t VEHICLE_AWAKE = 0x01;

/* OR'ed into 0x297 D1 when the VCU ends a charge. Seen once, in the 1-phase
 * capture: the charger dropped its current 120 ms later, while 0x29C still
 * asked for full current, and the EVSE only ended its PWM 4 s after that. */
const uint8_t VEHICLE_STOP_REQUEST = 0x20;

/* What the charge command (0x29C) and the mode message (0x33F) say, in the
 * order a charge goes through them. */
enum Command {
  CMD_SLEEP,
  CMD_AWAKE,
  CMD_STANDBY,
  CMD_CHARGE,
  CMD_STOPPING,
  CMD_SHUTDOWN
};

/* Pack status, 0x29B on both buses. */
enum PackStage { PACK_SLEEP, PACK_AWAKE, PACK_HV };

/* 0x324 D1, what the charger is doing. */
const uint8_t CHARGER_IDLE = 0x00;
const uint8_t CHARGER_AC = 0x30;
const uint8_t CHARGER_V2L = 0x70;

uint8_t Xor7(const uint8_t *frame);
bool HvActive(uint8_t hvState);
uint16_t VoltsToCounts(float volts);
uint16_t AmpsToCounts(float amps);
uint16_t Ramp(uint16_t now, uint16_t target, uint16_t step);

/* Hybrid CAN, sent by the VCU. */
void Build297Hybrid(uint8_t *out, uint8_t vehicle, uint8_t hvState,
                    uint8_t counter);
void Build29BHybrid(uint8_t *out, PackStage stage);
void Build29C(uint8_t *out, Command cmd, uint16_t currentCounts,
              uint16_t voltageCounts);
void Build33F(uint8_t *out, Command cmd, uint16_t rampCounts,
              uint16_t voltageCounts);
void Build394(uint8_t *out, bool awake, uint8_t counter);
void Build396(uint8_t *out, bool hvActive);
void Build39A(uint8_t *out, uint16_t roll);
void Build39BHybrid(uint8_t *out, uint8_t counter);

/* PT CAN, sent by the VCU. */
void Build19C(uint8_t *out, bool dcdcOn, uint8_t counter);
void Build297PT(uint8_t *out, uint8_t hvState);
void Build29BPT(uint8_t *out, PackStage stage);

/* Sent by the charger. */
uint8_t ChargerMode(const uint8_t *frame324);
float HvVolts(const uint8_t *frame324);
bool PilotReady(const uint8_t *frame33B);
uint8_t PilotDuty(const uint8_t *frame33B);
float PilotAmps(uint8_t dutyPercent);
float DcAmps(const uint8_t *frame33D);
uint8_t AcPhases(const uint8_t *frame491, float *amps, float *volts);
float LvVolts(const uint8_t *frame39F);
float LvAmps(const uint8_t *frame39F);

} // namespace MGgen2Protocol

#endif // MGGEN2FRAMES_H
