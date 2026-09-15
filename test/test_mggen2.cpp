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

/* Every expected frame below was copied from a capture of a real MG ZS EV:
 * github.com/damienmaguire/MG-EV-Charger, CANLogs/Gen2/Lars. File and time
 * stamp are noted with each one, so a failing test can be checked against
 * the car rather than argued about.
 *
 *   1149 = "1149_3-phase 16A charge session.csv"   (ignition on)
 *   1017 = "1017_1-phase 6A charge session.csv"    (ignition off)
 *
 * Frames marked "bench" come from the charger alone on 12 V instead:
 * Documentation/MGgen2/bench-2026-09-15-hybrid-a.log.
 */

#include "MGgen2Frames.h"
#include "test_list.h"
#include <cmath>
#include <cstring>

using namespace std;
using namespace MGgen2Protocol;

static bool Same(const uint8_t *got, const uint8_t expected[8]) {
  return memcmp(got, expected, 8) == 0;
}

static bool Near(float got, float expected) {
  return fabs(got - expected) < 0.01f;
}

// ------------------------------------------------------------ 0x297 hybrid

static void Test297AsleepMatchesCar() {
  const uint8_t car[8] = {0x00, 0x01, 0xE0, 0x40, 0xFF, 0xFF, 0x86, 0x27};
  uint8_t f[8];
  Build297Hybrid(f, VEHICLE_ASLEEP, HV_STANDBY, 6); // 1149 @ 0.035 s
  ASSERT(Same(f, car));
}

static void Test297AwakeMatchesCar() {
  const uint8_t car[8] = {0x01, 0x01, 0xE0, 0x40, 0xFF, 0xFF, 0x82, 0x22};
  uint8_t f[8];
  Build297Hybrid(f, VEHICLE_AWAKE, HV_STANDBY, 2); // 1149 @ 9.435 s
  ASSERT(Same(f, car));
}

static void Test297WakePulseMatchesCar() {
  const uint8_t car[8] = {0x01, 0x21, 0xE0, 0x40, 0xFF, 0xFF, 0x86, 0x06};
  uint8_t f[8];
  Build297Hybrid(f, VEHICLE_AWAKE, HV_STANDBY | HV_WAKE_PULSE,
                 6); // 1149 @ 9.635 s
  ASSERT(Same(f, car));
}

static void Test297PrechargeMatchesCar() {
  const uint8_t car[8] = {0x01, 0x0C, 0xE0, 0x40, 0xFF, 0xFF, 0x83, 0x2E};
  uint8_t f[8];
  Build297Hybrid(f, VEHICLE_AWAKE, HV_PRECHARGE, 3); // 1149 @ 41.485 s
  ASSERT(Same(f, car));
}

static void Test297HighVoltageFlagAndChecksum() {
  uint8_t f[8];
  Build297Hybrid(f, VEHICLE_AWAKE, HV_ON_IGNITION_OFF, 0x0F);
  ASSERT((f[6] & 0xF0) == 0xA0);
  ASSERT((f[6] & 0x0F) == 0x0F);
  ASSERT(f[7] == Xor7(f));
}

// ------------------------------------------------------------ 0x29C command

static void Test29CStatesMatchCar() {
  const uint8_t asleep[8] = {0x28, 0xFF, 0x83, 0xFF, 0x00, 0xFF, 0xFF, 0xFF};
  const uint8_t awake[8] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t start[8] = {0x28, 0xD7, 0x04, 0x00, 0x00, 0xDC, 0x57, 0x12};
  const uint8_t ramp[8] = {0x28, 0xD7, 0x05, 0x2C, 0x00, 0xDC, 0x57, 0x12};
  const uint8_t full[8] = {0x28, 0xD7, 0x07, 0xFE, 0x00, 0xDC, 0x57, 0x12};
  uint8_t f[8];
  Build29C(f, CMD_SLEEP, 0, 0); // 1149 @ 0.035 s
  ASSERT(Same(f, asleep));
  Build29C(f, CMD_STANDBY, 0, 0); // 1149 @ 9.435 s
  ASSERT(Same(f, awake));
  Build29C(f, CMD_CHARGE, 0, 0x5712); // 1149 @ 42.436 s
  ASSERT(Same(f, start));
  Build29C(f, CMD_CHARGE, 300, 0x5712); // 1149 @ 42.736 s
  ASSERT(Same(f, ramp));
  Build29C(f, CMD_CHARGE, 1022, 0x5712); // 1149 @ 43.536 s
  ASSERT(Same(f, full));
}

static void Test29CNeverSendsTheInvalidCurrent() {
  uint8_t f[8];
  Build29C(f, CMD_CHARGE, 1023, 0x5712);
  ASSERT((((f[2] & 0x03) << 8) | f[3]) == CURRENT_COUNTS_MAX);
}

static void TestCurrentRampTakesTheCarsElevenSteps() {
  // 1149 @ 42.436 .. 43.536 s: 0, 100, 200 ... 1000, 1022
  uint16_t now = 0;
  int steps = 0;
  while (now != CURRENT_COUNTS_MAX && steps < 100) {
    now = Ramp(now, CURRENT_COUNTS_MAX, CURRENT_RAMP_STEP);
    steps++;
  }
  ASSERT(steps == 11);
}

static void TestRampAlsoComesDown() {
  ASSERT(Ramp(1022, 0, CURRENT_RAMP_STEP) == 922);
  ASSERT(Ramp(50, 0, CURRENT_RAMP_STEP) == 0);
  ASSERT(Ramp(500, 500, CURRENT_RAMP_STEP) == 500);
}

// --------------------------------------------------------------- 0x33F mode

static void Test33FStatesMatchCar() {
  const uint8_t asleep[8] = {0x7F, 0xFF, 0x00, 0xFF, 0x28, 0x03, 0xFF, 0x00};
  const uint8_t awake[8] = {0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00};
  const uint8_t standby[8] = {0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x40};
  const uint8_t start[8] = {0x57, 0x12, 0x00, 0xDC, 0x28, 0x20, 0x00, 0x41};
  const uint8_t ramp[8] = {0x57, 0x12, 0x00, 0xDC, 0x28, 0x21, 0x18, 0x41};
  const uint8_t full[8] = {0x57, 0x12, 0x00, 0xDC, 0x28, 0x23, 0xFF, 0x41};
  uint8_t f[8];
  Build33F(f, CMD_SLEEP, 0, 0); // 1149 @ 0.075 s
  ASSERT(Same(f, asleep));
  Build33F(f, CMD_AWAKE, 0, 0); // 1149 @ 9.475 s
  ASSERT(Same(f, awake));
  Build33F(f, CMD_STANDBY, 0, 0); // 1149 @ 10.175 s
  ASSERT(Same(f, standby));
  Build33F(f, CMD_CHARGE, 0, 0x5712); // 1149 @ 42.475 s
  ASSERT(Same(f, start));
  Build33F(f, CMD_CHARGE, 280, 0x5712); // 1149 @ 43.175 s
  ASSERT(Same(f, ramp));
  Build33F(f, CMD_CHARGE, MODE_RAMP_MAX, 0x5712);
  ASSERT(Same(f, full));
}

static void TestModeRampTakesTheCarsSteps() {
  // 1149: 25 steps of 40, then one of 23, from 0x2000 to 0x23FF
  uint16_t now = 0;
  int steps = 0;
  while (now != MODE_RAMP_MAX && steps < 100) {
    now = Ramp(now, MODE_RAMP_MAX, MODE_RAMP_STEP);
    steps++;
  }
  ASSERT(steps == 26);
}

// ------------------------------------------------------- other hybrid frames

static void Test29BHybridMatchesCar() {
  const uint8_t asleep[8] = {0xFB, 0x96, 0x70, 0x03, 0xFD, 0x10, 0x00, 0x00};
  const uint8_t awake[8] = {0xBB, 0x96, 0x70, 0x02, 0xFD, 0x10, 0x00, 0x00};
  const uint8_t hv[8] = {0x3B, 0x96, 0x70, 0x00, 0xFD, 0x10, 0x00, 0x00};
  uint8_t f[8];
  Build29BHybrid(f, PACK_SLEEP); // 1149 @ 0.040 s
  ASSERT(Same(f, asleep));
  Build29BHybrid(f, PACK_AWAKE); // 1149 @ 2.640 s
  ASSERT(Same(f, awake));
  Build29BHybrid(f, PACK_HV); // 1149 @ 41.540 s
  ASSERT(Same(f, hv));
}

static void Test394MatchesCar() {
  const uint8_t asleep[8] = {0x00, 0x36, 0x00, 0x00, 0x00, 0x37, 0x09, 0x08};
  const uint8_t awake[8] = {0x00, 0x36, 0x00, 0x00, 0x10, 0x37, 0x0A, 0x1B};
  uint8_t f[8];
  Build394(f, false, 0x09); // 1149 @ 0.094 s
  ASSERT(Same(f, asleep));
  Build394(f, true, 0x0A); // 1149 @ 9.795 s
  ASSERT(Same(f, awake));
}

static void Test396MatchesCar() {
  const uint8_t before[8] = {0x3B, 0x6D, 0x4E, 0x28, 0x00, 0x45, 0x45, 0x45};
  const uint8_t hv[8] = {0x3B, 0x6D, 0x4E, 0x28, 0x80, 0x45, 0x45, 0x45};
  uint8_t f[8];
  Build396(f, false); // 1149 @ 0.043 s
  ASSERT(Same(f, before));
  Build396(f, true); // 1149 @ 41.844 s
  ASSERT(Same(f, hv));
}

static void Test39AMatchesCar() {
  const uint8_t car[8] = {0x00, 0x36, 0x02, 0x00, 0xAD, 0x80, 0x00, 0x00};
  uint8_t f[8];
  Build39A(f, 694); // 1149 @ 0.096 s
  ASSERT(Same(f, car));
}

static void Test39BHybridMatchesCar() {
  const uint8_t car[8] = {0x3C, 0x3A, 0x82, 0x00, 0x00, 0x00, 0x20, 0xA4};
  uint8_t f[8];
  Build39BHybrid(f, 8); // 1149, counter 8 shows as 0x20
  ASSERT(Same(f, car));
}

// ------------------------------------------------------------------- PT bus

static void Test19CMatchesCar() {
  const uint8_t off[8] = {0x06, 0xA0, 0x06, 0xA0, 0x7F, 0xFF, 0x07, 0x7F};
  const uint8_t on[8] = {0x06, 0xA0, 0x26, 0xA0, 0x7F, 0xFF, 0x04, 0x7F};
  uint8_t f[8];
  Build19C(f, false, 7); // 1149 @ 0.113 s
  ASSERT(Same(f, off));
  Build19C(f, true, 4); // 1149, while charging
  ASSERT(Same(f, on));
}

static void Test297PTMatchesCar() {
  const uint8_t standby[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t running[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00};
  const uint8_t charging[8] = {0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00};
  const uint8_t shutdown[8] = {0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00};
  const uint8_t precharge[8] = {0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t f[8];
  Build297PT(f, HV_STANDBY); // 1149
  ASSERT(Same(f, standby));
  Build297PT(f, HV_ON_IGNITION_ON); // 1149
  ASSERT(Same(f, running));
  Build297PT(f, HV_ON_IGNITION_OFF); // 1017
  ASSERT(Same(f, charging));
  Build297PT(f, HV_SHUTDOWN); // 1017
  ASSERT(Same(f, shutdown));
  Build297PT(f, HV_PRECHARGE); // 1149
  ASSERT(Same(f, precharge));
  Build297PT(f, HV_STANDBY | HV_WAKE_PULSE); // the PT copy has no pulse
  ASSERT(Same(f, standby));
}

static void Test29BPTMatchesCar() {
  uint8_t f[8];
  Build29BPT(f, PACK_SLEEP);
  ASSERT(f[0] == 0xC0);
  Build29BPT(f, PACK_AWAKE);
  ASSERT(f[0] == 0x80);
  Build29BPT(f, PACK_HV);
  ASSERT(f[0] == 0x00);
}

// ----------------------------------------------------- charger to VCU

static void TestChargerStatusDecodes() {
  const uint8_t f324[8] = {0x30, 0x4F, 0xB0, 0x4B, 0x48, 0x00, 0x00, 0x1E};
  ASSERT(ChargerMode(f324) == CHARGER_AC); // 1149, 3 x 16 A
  ASSERT(Near(HvVolts(f324), 408.0f));
  ASSERT(Near(ChargerTemp(f324), 35.0f)); // D4 0x4B, D5 0x48
}

static void TestChargerTemperatureOffset() {
  const uint8_t settled[8] = {0x00, 0x00, 0x00, 0x3F, 0x3F, 0x00, 0x00, 0x00};
  const uint8_t overshoot[8] = {0x00, 0x00, 0x00, 0x28, 0x51, 0x00, 0x00, 0x00};
  ASSERT(Near(ChargerTemp(settled), 23.0f));   // bench @ 3.3 s, room temp
  ASSERT(Near(ChargerTemp(overshoot), 41.0f)); // bench @ 0.1 s, the hotter
}

static void TestPilotDecodes() {
  const uint8_t ready[8] = {0x99, 0x00, 0x00, 0x49, 0x00, 0x00, 0x00, 0x00};
  const uint8_t noPwm[8] = {0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ASSERT(PilotReady(ready)); // 1149, charging
  ASSERT(PilotDuty(ready) == 25);
  ASSERT(Near(PilotAmps(PilotDuty(ready)), 15.0f));
  ASSERT(!PilotReady(noPwm)); // 1149 @ 2.19 s, static pilot
  ASSERT(PilotAmps(PilotDuty(noPwm)) == 0);
}

static void TestPilotAmpsFollowsIec61851() {
  ASSERT(PilotAmps(5) == 0); // digital communication, no AC limit
  ASSERT(Near(PilotAmps(10), 6.0f));
  ASSERT(Near(PilotAmps(50), 30.0f));
  ASSERT(Near(PilotAmps(90), 65.0f));
  ASSERT(PilotAmps(100) == 0); // no PWM
}

static void TestDcCurrentDecodes() {
  const uint8_t f[8] = {0xE1, 0xCA, 0x50, 0x80, 0x00, 0x18, 0xEE, 0x20};
  ASSERT(Near(DcAmps(f), 23.8f)); // 1149, 3 x 16 A
}

static void TestAcPhasesDecode() {
  const uint8_t three[8] = {0x4A, 0x71, 0x4B, 0x71, 0x4B, 0x73, 0x00, 0x00};
  const uint8_t one[8] = {0x1D, 0x6C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  float amps[3], volts[3];
  ASSERT(AcPhases(three, amps, volts) == 3); // 1149
  ASSERT(Near(amps[0], 14.8f));
  ASSERT(Near(volts[2], 230.0f));
  ASSERT(AcPhases(one, amps, volts) == 1); // 1017
  ASSERT(Near(volts[0], 216.0f));
}

static void TestLowVoltageDecodes() {
  const uint8_t f[8] = {0x3B, 0x6D, 0x00, 0xA9, 0x0F, 0x57, 0x02, 0xA5};
  ASSERT(Near(LvVolts(f), 13.625f)); // 1149, DC-DC running
  ASSERT(Near(LvAmps(f), 15.0f));
}

// ------------------------------------------------------------ conversions

static void TestConversions() {
  // 108 cells x 4.15 V = 448.2 V, the limit this build was written for
  ASSERT(VoltsToCounts(448.2f) == 0x578A);
  ASSERT(VoltsToCounts(445.8f) == 0x5712); // what the MG car sends
  ASSERT(VoltsToCounts(-1) == 0);
  ASSERT(AmpsToCounts(2.0f) == 40);
  ASSERT(AmpsToCounts(51.1f) == CURRENT_COUNTS_MAX);
  ASSERT(AmpsToCounts(80.0f) == CURRENT_COUNTS_MAX);
  ASSERT(AmpsToCounts(-3.0f) == 0);
}

// ------------------------------------------------------------------ stopping

static void TestStopFramesMatchCar() {
  const uint8_t stopping[8] = {0x57, 0x12, 0x00, 0xDC, 0x28, 0x23, 0xFF, 0x01};
  const uint8_t mode[8] = {0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x03};
  const uint8_t command[8] = {0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t f[8];
  Build33F(f, CMD_STOPPING, MODE_RAMP_MAX, 0x5712); // 1017 @ 122.08 s
  ASSERT(Same(f, stopping));
  Build33F(f, CMD_SHUTDOWN, 0, 0); // 1017 @ 126.68 s
  ASSERT(Same(f, mode));
  Build29C(f, CMD_SHUTDOWN, 0, 0); // 1017 @ 126.64 s
  ASSERT(Same(f, command));
}

static void Test297StopRequest() {
  // 1017 @ 121.19 s: D1 went from 0x03 to 0x23
  uint8_t f[8];
  Build297Hybrid(f, VEHICLE_AWAKE | VEHICLE_STOP_REQUEST, HV_ON_IGNITION_OFF,
                 7);
  ASSERT(f[0] == 0x21);
  ASSERT(f[7] == Xor7(f));
}

void MGgen2Test::RunTest() {
  Test297AsleepMatchesCar();
  Test297AwakeMatchesCar();
  Test297WakePulseMatchesCar();
  Test297PrechargeMatchesCar();
  Test297HighVoltageFlagAndChecksum();
  Test29CStatesMatchCar();
  Test29CNeverSendsTheInvalidCurrent();
  TestCurrentRampTakesTheCarsElevenSteps();
  TestRampAlsoComesDown();
  Test33FStatesMatchCar();
  TestModeRampTakesTheCarsSteps();
  Test29BHybridMatchesCar();
  Test394MatchesCar();
  Test396MatchesCar();
  Test39AMatchesCar();
  Test39BHybridMatchesCar();
  Test19CMatchesCar();
  Test297PTMatchesCar();
  Test29BPTMatchesCar();
  TestChargerStatusDecodes();
  TestChargerTemperatureOffset();
  TestPilotDecodes();
  TestPilotAmpsFollowsIec61851();
  TestDcCurrentDecodes();
  TestAcPhasesDecode();
  TestLowVoltageDecodes();
  TestConversions();
  TestStopFramesMatchCar();
  Test297StopRequest();
}
