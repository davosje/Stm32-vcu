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

/* The expected frames below are worked out from the bit positions in
 * meb-preheat (J848.ldf and NOTES.md), not copied from a capture. They
 * check that the code does what those descriptions say. Whether the heater
 * agrees is for the bench. When a real capture exists, add it here.
 */

#include "VWMebHeaterFrames.h"
#include "test_list.h"
#include <cmath>
#include <cstring>

using namespace std;
using namespace VWMebHeaterProtocol;

static bool Same(const uint8_t *got, const uint8_t *expected, int len) {
  return memcmp(got, expected, len) == 0;
}

static bool Near(float got, float expected) {
  return fabs(got - expected) < 0.01f;
}

// ------------------------------------------------------------ 0x1C command

static void TestOffIsAllZero() {
  const uint8_t zero[4] = {0, 0, 0, 0};
  uint8_t f[4];
  BuildCommand(f, false, 0, LAYOUT_LDF);
  ASSERT(Same(f, zero, 4));
  BuildCommand(f, false, 0, LAYOUT_NOTES);
  ASSERT(Same(f, zero, 4));
}

static void TestLdfOnAtFiftyPercent() {
  // bit 18 on, 50 at bits 19..25: 0x01940000 little endian
  const uint8_t expected[4] = {0x00, 0x00, 0x94, 0x01};
  uint8_t f[4];
  BuildCommand(f, true, 50, LAYOUT_LDF);
  ASSERT(Same(f, expected, 4));
}

static void TestLdfOnBitAlone() {
  const uint8_t expected[4] = {0x00, 0x00, 0x04, 0x00};
  uint8_t f[4];
  BuildCommand(f, true, 0, LAYOUT_LDF);
  ASSERT(Same(f, expected, 4));
}

static void TestNotesOnAtFiftyPercent() {
  // byte 2: duty low 6 bits | on at bit 6; byte 3: duty bits 6..7
  const uint8_t expected[4] = {0x00, 0x00, 0x72, 0x00};
  uint8_t f[4];
  BuildCommand(f, true, 50, LAYOUT_NOTES);
  ASSERT(Same(f, expected, 4));
}

static void TestNotesHundredUsesByteThree() {
  const uint8_t expected[4] = {0x00, 0x00, 0x64, 0x01};
  uint8_t f[4];
  BuildCommand(f, true, 100, LAYOUT_NOTES);
  ASSERT(Same(f, expected, 4));
}

static void TestDutyAboveHundredIsLimited() {
  uint8_t a[4], b[4];
  BuildCommand(a, true, 250, LAYOUT_LDF);
  BuildCommand(b, true, 100, LAYOUT_LDF);
  ASSERT(Same(a, b, 4));
}

// ------------------------------------------------------------ 0x0F feedback

static void TestFeedbackDecodes() {
  const uint8_t f[8] = {0x28, 0xC8, 0x00, 0x00, 0x8D, 0x5A, 0x46, 0x4B};
  Feedback fb;
  DecodeFeedback(f, fb);
  ASSERT(Near(fb.hvCurrent, 10.0f));
  ASSERT(Near(fb.hvVoltage, 400.0f));
  ASSERT(Near(fb.lvVoltage, 14.1f));
  ASSERT(fb.tempHeater == 40);
  ASSERT(fb.tempIn == 20);
  ASSERT(fb.tempOut == 25);
  ASSERT(fb.status == 0);
}

static void TestFeedbackBelowZero() {
  const uint8_t f[8] = {0, 0, 0, 0, 0x7B, 0x28, 0x28, 0x28};
  Feedback fb;
  DecodeFeedback(f, fb);
  ASSERT(fb.tempIn == -10);
}

// ------------------------------------------------------------ ramp

static void TestRampTakesTwoPercentPerSecond() {
  uint16_t d = 0;
  for (int i = 0; i < 20; i++) // 20 steps of 50 ms
    d = StepDuty(d, 50);
  ASSERT(d == 20); // 2.0 %
}

static void TestRampDropsAtOnce() {
  ASSERT(StepDuty(300, 10) == 100);
  ASSERT(StepDuty(300, 0) == 0);
}

static void TestRampStopsAtNinetyNine() {
  uint16_t d = 980;
  for (int i = 0; i < 50; i++)
    d = StepDuty(d, 100);
  ASSERT(d == 990);
}

void VWMebHeaterTest::RunTest() {
  TestOffIsAllZero();
  TestLdfOnAtFiftyPercent();
  TestLdfOnBitAlone();
  TestNotesOnAtFiftyPercent();
  TestNotesHundredUsesByteThree();
  TestDutyAboveHundredIsLimited();
  TestFeedbackDecodes();
  TestFeedbackBelowZero();
  TestRampTakesTwoPercentPerSecond();
  TestRampDropsAtOnce();
  TestRampStopsAtNinetyNine();
}
