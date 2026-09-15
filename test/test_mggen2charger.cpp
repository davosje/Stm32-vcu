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

/* The MG gen 2 charger's state machine, run against a fake CAN bus.
 *
 * test_mggen2.cpp proves the bytes. These tests prove the behaviour that
 * matters for safety: when charge mode is asked for, and above all that
 * charge mode, and with it the contactors, is held until the charger has let
 * go of its current. */

#include "MGgen2Charger.h"
#include "test_list.h"
#include <cstring>
#include <map>

using namespace std;

namespace {

class FakeCan : public CanHardware {
public:
  struct Frame {
    uint8_t bytes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int count = 0;
  };

  void SetBaudrate(enum baudrates) override {}
  void Send(uint32_t canId, uint32_t data[2], uint8_t len) override {
    Frame &frame = frames[canId];
    memcpy(frame.bytes, (uint8_t *)data, len < 8 ? len : 8);
    frame.count++;
  }

  int Count(uint32_t id) { return frames.count(id) ? frames[id].count : 0; }
  uint8_t Byte(uint32_t id, int index) { return frames[id].bytes[index]; }
  void Forget() { frames.clear(); }

private:
  void ConfigureFilters() override {}
  map<uint32_t, Frame> frames;
};

FakeCan bus;
MGgen2Charger charger;

// 0x33B D1 as captured
const uint8_t PILOT_STATIC = 0x64; // 100 %, no PWM yet   (1149 @ 2.19 s)
const uint8_t PILOT_READY = 0x9A;  // bit 7 + 26 %       (1149 @ 8.89 s)
const uint8_t PILOT_GONE = 0x80;   // bit 7, no duty     (1017 @ 125.41 s)

uint8_t pilot = 0;
float dcAmps = 0;

void ChargerSends() {
  uint32_t data[2] = {0, 0};
  uint8_t *bytes = (uint8_t *)data;
  bytes[0] = pilot;
  charger.DecodeCAN(0x33B, data);
  data[0] = data[1] = 0;
  bytes[6] = (uint8_t)(dcAmps * 10);
  charger.DecodeCAN(0x33D, data);
}

/* Run for a number of 100 ms steps, calling the tasks as the VCU does. */
void Run(int steps, bool chargerTalks = true) {
  for (int step = 0; step < steps; step++) {
    if (chargerTalks)
      ChargerSends();
    for (int i = 0; i < 10; i++)
      charger.Task10Ms();
    charger.Task100Ms();
  }
}

int RequestedCounts() {
  return ((bus.Byte(0x29C, 2) & 0x03) << 8) | bus.Byte(0x29C, 3);
}

void Reset() {
  charger.DeInit();
  bus.ClearUserMessages();
  charger.SetCanInterface(&bus);
  bus.Forget();
  pilot = 0;
  dcAmps = 0;
  Param::SetInt(Param::opmode, MOD_OFF);
  Param::SetFloat(Param::Voltspnt, 448.2f);
  Param::SetFloat(Param::Pwrspnt, 11000);
  Param::SetFloat(Param::udc, 400);
  Param::SetFloat(Param::BMS_ChargeLim, 0);
}

/* Plug in, precharge, and charge until the request has ramped up. */
void StartCharging() {
  pilot = PILOT_READY;
  Run(10);
  charger.ControlCharge(true, true);
  Param::SetInt(Param::opmode, MOD_PRECHARGE);
  Run(10);
  Param::SetInt(Param::opmode, MOD_CHARGE);
  dcAmps = 24;
  Run(20);
}

} // namespace

static void TestSilentUntilTheChargerSpeaks() {
  Reset();
  Run(20, false);
  ASSERT(bus.Count(0x297) == 0);
  ASSERT(bus.Count(0x29C) == 0);
}

static void TestWakesButDoesNotChargeWithoutPwm() {
  Reset();
  pilot = PILOT_STATIC;
  Run(10);
  ASSERT(bus.Byte(0x297, 0) == 0x01); // awake
  ASSERT(bus.Byte(0x33F, 7) == 0x40); // standby, charge port active
  ASSERT(!charger.ControlCharge(true, true));
}

static void TestStartsOnTheStartSignalAndRampsLikeTheCar() {
  Reset();
  pilot = PILOT_READY;
  Run(10);
  ASSERT(!charger.ControlCharge(false, true)); // not without RunCh
  ASSERT(charger.ControlCharge(true, true));

  Param::SetInt(Param::opmode, MOD_PRECHARGE);
  Run(2);
  ASSERT(bus.Byte(0x297, 1) == 0x04); // wake state first, as the car
  Run(3);
  ASSERT(bus.Byte(0x297, 1) == 0x0C); // then precharge

  Param::SetInt(Param::opmode, MOD_CHARGE);
  Run(1);
  ASSERT(bus.Byte(0x29C, 1) == 0xD7);
  ASSERT(bus.Byte(0x297, 1) == 0x06);
  ASSERT(bus.Byte(0x33F, 7) == 0x41);

  // 11 kW at 400 V is 27.5 A: 550 counts, never more than 100 per step
  int previous = 0;
  bool gentle = true;
  for (int i = 0; i < 10; i++) {
    int counts = RequestedCounts();
    if (counts - previous > 100 || counts > 550)
      gentle = false;
    previous = counts;
    Run(1);
  }
  ASSERT(gentle);
  ASSERT(previous == 550);
  // Voltspnt = 448.2 V. Parameters are fixed point in steps of 1/32 V, so it
  // is stored as 448.1875 V and sent as 0x5789 (448.18 V), not 0x578A: the
  // charger stops 0.02 V early, which is the safe side.
  ASSERT(bus.Byte(0x29C, 6) == 0x57);
  ASSERT(bus.Byte(0x29C, 7) == 0x89);
}

static void TestBmsLimitCapsTheRequest() {
  Reset();
  Param::SetFloat(Param::BMS_ChargeLim, 10);
  StartCharging();
  ASSERT(RequestedCounts() == 200); // 10 A
}

static void TestHoldsChargeModeUntilTheCurrentIsGone() {
  Reset();
  StartCharging();
  ASSERT(charger.ControlCharge(false, true)); // RunCh dropped, still held
  Run(1);
  ASSERT((bus.Byte(0x297, 0) & 0x20) != 0); // stop request is out
  dcAmps = 0;                               // the charger lets go
  Run(5);
  ASSERT(charger.ControlCharge(false, true)); // inside the 0.9 s pulse
  Run(5);
  ASSERT(!charger.ControlCharge(false, true)); // current gone: released
}

static void TestGivesUpHoldingAfterThreeSeconds() {
  Reset();
  StartCharging();
  charger.ControlCharge(false, true);
  Run(20); // the current never drops
  ASSERT(charger.ControlCharge(false, true));
  ASSERT(RequestedCounts() == 0); // but we stopped asking for it
  Run(10);
  ASSERT(!charger.ControlCharge(false, true));
}

static void TestDoesNotHoldWhenHighVoltageIsAlreadyDown() {
  Reset();
  StartCharging();
  charger.ControlCharge(false, true);    // stopping
  Param::SetInt(Param::opmode, MOD_OFF); // the VCU dropped HV anyway
  ASSERT(!charger.ControlCharge(false, true));
}

static void TestStopsWhenThePilotGoes() {
  Reset();
  StartCharging();
  pilot = PILOT_GONE;
  Run(1);
  ASSERT(charger.ControlCharge(true, true)); // stopping, and holding
  Run(1);
  ASSERT((bus.Byte(0x297, 0) & 0x20) != 0);
}

static void TestNoNewChargeWhileShuttingDown() {
  Reset();
  StartCharging();
  charger.ControlCharge(false, true);
  dcAmps = 0;
  Run(10); // stopped, now shutting down
  Param::SetInt(Param::opmode, MOD_OFF);
  Run(10);
  ASSERT(bus.Byte(0x297, 1) == 0x0A);
  ASSERT(!charger.ControlCharge(true, true)); // pilot still there: still no
  Run(60);
  ASSERT(charger.ControlCharge(true, true)); // 6 s later it may start again
}

void MGgen2ChargerTest::RunTest() {
  TestSilentUntilTheChargerSpeaks();
  TestWakesButDoesNotChargeWithoutPwm();
  TestStartsOnTheStartSignalAndRampsLikeTheCar();
  TestBmsLimitCapsTheRequest();
  TestHoldsChargeModeUntilTheCurrentIsGone();
  TestGivesUpHoldingAfterThreeSeconds();
  TestDoesNotHoldWhenHighVoltageIsAlreadyDown();
  TestStopsWhenThePilotGoes();
  TestNoNewChargeWhileShuttingDown();
}
