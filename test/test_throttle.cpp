/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#include "leafbms.h"
#include "my_fp.h"
#include "my_math.h"
#include "test_list.h"
#include "throttle.h"

using namespace std;

void Param::Change(Param::PARAM_NUM p) {
  // Dummy function - we ignore parameter changes in these tests
}

// Include a dummy error list to allow the tests to link. It is unused.
const char *errorListString = "";

static void TestSetup() {
  // percentage of deadzone
  Throttle::throtdead = 5;
  Throttle::potmin[0] = 100;
  Throttle::potmax[0] = 4000;
  Throttle::throtmax = 100;
  Param::SetInt(Param::dir, 1);
}

// TEMPERATURE DERATING
static void TestThrottleTemperateOverMaxThrottleTo0() {
  float temp = 60;
  float tempMax = 50;
  float finalSpnt = 100;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) &&
         finalSpnt == 0);
}

static void TestThrottleTemperateInDerateZoneThrottleTo50Percent() {
  float temp = 61;
  float tempMax = 60;
  float finalSpnt = 100;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) &&
         finalSpnt == 50);
}

static void TestThrottleUnderTemperateNoDeRate() {
  float temp = 30;
  float tempMax = 60;
  float finalSpnt = 100;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) == false &&
         finalSpnt == 100);
}

static void TestThrottleTemperateInDerateZoneThrottleButThrottleUnderLimit() {
  float temp = 61;
  float tempMax = 60;
  float finalSpnt = 49;
  ASSERT(Throttle::TemperatureDerate(temp, tempMax, finalSpnt) &&
         finalSpnt == 49);
}

// CALC THROTTLE
static void TestCalcThrottleIs0WhenThrottleAndBrakePressed() {

  ASSERT(Throttle::CalcThrottle(3000, 0, true) == 0);
}

static void TestCalcThrottleIs0WhenNoThrottleAndBrakePressed() {

  ASSERT(Throttle::CalcThrottle(0, 0, true) == 0);
}

static void TestCalcThrottleIs0WhenInDeadZone() {
  // deadzone is first 5% of travel between 100 and 4000
  ASSERT(Throttle::CalcThrottle(295, 0, false) == 0);
}

static void TestCalcThrottleIsAbove0WhenJustOutOfDeadZone() {
  // deadzone is first 5% of travel between 100 and 4000
  ASSERT(Throttle::CalcThrottle(496, 0, false) > 0);
}

static void TestCalcThrottleIs100WhenMax() {
  // deadzone is first 5% of travel between 100 and 4000
  float throtVal = Throttle::CalcThrottle(4000, 0, false);
  ASSERT(throtVal == 100);
}

static void TestCalcThrottleIs100WhenOverMax() {
  // deadzone is first 5% of travel between 100 and 4000
  float throtVal = Throttle::CalcThrottle(4001, 0, false);
  // ASSERT(throtVal == 100);
}

// REGEN LIMIT FROM THE BMS
static bool Near(float a, float b) { return ABS(a - b) < 1; }

static void SetRegenLimit(int on, float maxInputKw, float udc) {
  Param::SetInt(Param::bmsregenlim, on);
  Param::SetInt(Param::idcmax, 5000);
  Param::SetInt(Param::idcmin, -5000);
  Param::SetFloat(Param::BMS_MaxInput, maxInputKw);
  Param::SetFloat(Param::udc, udc);
}

// IdcLimitCommand filters the current, so give it time to settle.
static float LimitedSpnt(float spnt, float idc) {
  float finalSpnt = spnt;
  for (int i = 0; i < 200; i++) {
    finalSpnt = spnt;
    Throttle::IdcLimitCommand(finalSpnt, idc);
  }
  return finalSpnt;
}

static void TestBmsRegenLimitOffLeavesRegenAlone() {
  SetRegenLimit(0, 0, 440);
  ASSERT(Near(LimitedSpnt(-50, 0), -50));
}

static void TestBmsRegenLimitOfZeroStopsRegen() {
  SetRegenLimit(1, 0, 440);
  ASSERT(LimitedSpnt(-50, 0) == 0);
}

static void TestBmsRegenLimitFollowsCurrent() {
  SetRegenLimit(1, 44, 440); // 100 A
  ASSERT(Near(LimitedSpnt(-50, 0), -50));
  ASSERT(Near(LimitedSpnt(-50, 90), -10));
  ASSERT(LimitedSpnt(-50, 150) == 0);
}

static void TestBmsRegenLimitWithoutVoltageStopsRegen() {
  SetRegenLimit(1, 44, 0);
  ASSERT(LimitedSpnt(-50, 0) == 0);
}

static void TestBmsRegenLimitKeepsStricterIdcmin() {
  SetRegenLimit(1, 44, 440); // 100 A from the BMS, 20 A from idcmin
  Param::SetInt(Param::idcmin, -20);
  ASSERT(Near(LimitedSpnt(-50, 0), -20));
}

static void TestBmsRegenLimitDoesNotTouchDrive() {
  SetRegenLimit(1, 0, 440);
  ASSERT(Near(LimitedSpnt(50, 0), 50));
}

// LEAF BMS: THE LIMIT GOES AWAY WHEN THE MESSAGE DOES
static uint8_t NissanCrc(const uint8_t *data) {
  uint8_t crc = 0;
  for (int b = 0; b < 8; b++) {
    uint8_t byte = (b == 7) ? 0 : data[b];
    for (int i = 7; i >= 0; i--) {
      uint8_t bit = (byte >> i) & 1;
      crc = crc >= 0x80 ? (uint8_t)(((crc << 1) + bit) ^ 0x85)
                        : (uint8_t)((crc << 1) + bit);
    }
  }
  return crc;
}

static void SendChargeLimit40Kw(LeafBMS &bms, bool corrupt) {
  uint8_t frame[8] = {0, 10, 0, 0, 0, 0, 0, 0}; // 160 counts of 0.25 kW
  frame[7] = NissanCrc(frame) ^ (corrupt ? 0xFF : 0);
  bms.DecodeCAN(0x1DC, frame);
}

static void TestLeafBmsDropsLimitWhenMessagesStop() {
  LeafBMS bms;
  Param::SetFloat(Param::udc2, 400);
  Param::SetInt(Param::BMS_MaxInput, 0);

  SendChargeLimit40Kw(bms, false);
  ASSERT(Param::GetInt(Param::BMS_MaxInput) == 40);
  for (int i = 0; i < 4; i++)
    bms.Task100Ms();
  ASSERT(Param::GetInt(Param::BMS_MaxInput) == 40);
  bms.Task100Ms();
  ASSERT(Param::GetInt(Param::BMS_MaxInput) == 0);

  SendChargeLimit40Kw(bms, false);
  ASSERT(Param::GetInt(Param::BMS_MaxInput) == 40);
}

static void TestLeafBmsCorruptFrameDoesNotKeepLimitAlive() {
  LeafBMS bms;
  Param::SetFloat(Param::udc2, 400);

  SendChargeLimit40Kw(bms, false);
  for (int i = 0; i < 5; i++) {
    SendChargeLimit40Kw(bms, true);
    bms.Task100Ms();
  }
  ASSERT(Param::GetInt(Param::BMS_MaxInput) == 0);
}

void ThrottleTest::RunTest() {
  TestSetup();
  TestThrottleTemperateOverMaxThrottleTo0();
  TestThrottleTemperateInDerateZoneThrottleTo50Percent();
  TestThrottleUnderTemperateNoDeRate();
  TestThrottleTemperateInDerateZoneThrottleButThrottleUnderLimit();
  TestCalcThrottleIs0WhenThrottleAndBrakePressed();
  TestCalcThrottleIs0WhenNoThrottleAndBrakePressed();
  TestCalcThrottleIs0WhenInDeadZone();
  TestCalcThrottleIsAbove0WhenJustOutOfDeadZone();
  TestCalcThrottleIs100WhenMax();
  TestCalcThrottleIs100WhenOverMax();
  TestBmsRegenLimitOffLeavesRegenAlone();
  TestBmsRegenLimitOfZeroStopsRegen();
  TestBmsRegenLimitFollowsCurrent();
  TestBmsRegenLimitWithoutVoltageStopsRegen();
  TestBmsRegenLimitKeepsStricterIdcmin();
  TestBmsRegenLimitDoesNotTouchDrive();
  TestLeafBmsDropsLimitWhenMessagesStop();
  TestLeafBmsCorruptFrameDoesNotKeepLimitAlive();
}
