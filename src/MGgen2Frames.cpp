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

#include "MGgen2Frames.h"

namespace MGgen2Protocol {

/* Frame templates, byte for byte as captured. Only fields that carry a state
 * or a value are changed below; every other byte stays what the car sent.
 * Where a byte drifted a little in the car (temperatures, most likely), the
 * template holds one captured value. */
static const uint8_t HYB297[8] = {0x00, 0x01, 0xE0, 0x40,
                                  0xFF, 0xFF, 0x80, 0x00};
static const uint8_t HYB29B_SLEEP[8] = {0xFB, 0x96, 0x70, 0x03,
                                        0xFD, 0x10, 0x00, 0x00};
static const uint8_t HYB29B_AWAKE[8] = {0xBB, 0x96, 0x70, 0x02,
                                        0xFD, 0x10, 0x00, 0x00};
static const uint8_t HYB29B_HV[8] = {0x3B, 0x96, 0x70, 0x00,
                                     0xFD, 0x10, 0x00, 0x00};
static const uint8_t CMD29C_SLEEP[8] = {0x28, 0xFF, 0x83, 0xFF,
                                        0x00, 0xFF, 0xFF, 0xFF};
static const uint8_t CMD29C_AWAKE[8] = {0x28, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00};
static const uint8_t CMD29C_CHARGE[8] = {0x28, 0xD7, 0x04, 0x00,
                                         0x00, 0xDC, 0x00, 0x00};
static const uint8_t MODE33F_SLEEP[8] = {0x7F, 0xFF, 0x00, 0xFF,
                                         0x28, 0x03, 0xFF, 0x00};
static const uint8_t MODE33F_AWAKE[8] = {0x00, 0x00, 0x00, 0x00,
                                         0x28, 0x00, 0x00, 0x00};
static const uint8_t MODE33F_CHARGE[8] = {0x00, 0x00, 0x00, 0xDC,
                                          0x28, 0x20, 0x00, 0x41};
static const uint8_t HYB394[8] = {0x00, 0x36, 0x00, 0x00,
                                  0x00, 0x37, 0x00, 0x00};
static const uint8_t HYB396[8] = {0x3B, 0x6D, 0x4E, 0x28,
                                  0x00, 0x45, 0x45, 0x45};
static const uint8_t HYB39A[8] = {0x00, 0x36, 0x02, 0x00,
                                  0x00, 0x00, 0x00, 0x00};
static const uint8_t HYB39B[8] = {0x3C, 0x3A, 0x82, 0x00,
                                  0x00, 0x00, 0x00, 0x00};
static const uint8_t PT19C[8] = {0x06, 0xA0, 0x06, 0xA0,
                                 0x7F, 0xFF, 0x00, 0x7F};

/* Bits and bytes whose role is known. */
static const uint8_t COUNTER_MASK = 0x0F;
static const uint8_t STATE_MASK = 0x1F;  // 0x297 D2 without the pulse flags
static const uint8_t FLAGS_297 = 0x80;   // 0x297 D7, always set
static const uint8_t HV_FLAG_297 = 0x20; // 0x297 D7 (both buses), HV is up
static const uint8_t AWAKE_394 = 0x10;   // 0x394 D5
static const uint8_t HV_396 = 0x80;      // 0x396 D5
static const uint8_t CURRENT_FIXED_29C = 0x04; // 0x29C D3, fixed bits
static const uint8_t RAMP_FIXED_33F = 0x20;    // 0x33F D6, fixed bits
static const uint8_t STANDBY_33F = 0x40;       // 0x33F D8, charge port active
static const uint8_t STOPPING_33F = 0x01;      // 0x33F D8, charge ending
static const uint8_t SHUTDOWN_33F = 0x03;      // 0x33F D8, HV shutting down
static const uint8_t DCDC_ON_19C = 0x26;       // 0x19C D3
static const uint8_t DCDC_OFF_19C = 0x06;
static const uint8_t PT29B_SLEEP = 0xC0; // 0x29B D1 on the PT bus
static const uint8_t PT29B_AWAKE = 0x80;
static const uint8_t PT29B_HV = 0x00;
static const uint8_t PILOT_READY_BIT = 0x80; // 0x33B D1 bit 7
static const uint8_t PILOT_DUTY_MASK = 0x7F; // 0x33B D1 bits 0-6, percent

/* IEC 61851-1: only 8..97 % duty carries an AC current limit. 5 % means
 * digital communication, 100 % means no PWM at all. */
static const uint8_t PILOT_PWM_MIN = 8;
static const uint8_t PILOT_PWM_MAX = 97;

static const float HV_VOLTS_PER_COUNT = 0.02f; // 0x324 D2:D3
static const float AMPS_PER_COUNT_33D = 0.1f;  // 0x33D D7
static const float AMPS_PER_COUNT_491 = 0.2f;  // 0x491, derived, not calibrated
static const float VOLTS_PER_COUNT_491 = 2.0f; // 0x491
static const float LV_VOLTS_PER_COUNT = 0.125f; // 0x39F D2

static void Copy(uint8_t *out, const uint8_t *from) {
  for (int i = 0; i < 8; i++)
    out[i] = from[i];
}

static void Clear(uint8_t *out) {
  for (int i = 0; i < 8; i++)
    out[i] = 0;
}

static void PutWord(uint8_t *out, int index, uint16_t value) {
  out[index] = value >> 8;
  out[index + 1] = value & 0xFF;
}

uint8_t Xor7(const uint8_t *frame) {
  uint8_t x = 0;
  for (int i = 0; i < 7; i++)
    x ^= frame[i];
  return x;
}

bool HvActive(uint8_t hvState) {
  uint8_t state = hvState & STATE_MASK;
  return state == HV_ON_IGNITION_ON || state == HV_ON_IGNITION_OFF ||
         state == HV_SHUTDOWN;
}

uint16_t VoltsToCounts(float volts) {
  if (volts <= 0)
    return 0;
  float counts = volts / VOLTS_PER_COUNT + 0.5f;
  if (counts >= 65535.0f)
    return 0xFFFF;
  return (uint16_t)counts;
}

uint16_t AmpsToCounts(float amps) {
  if (amps <= 0)
    return 0;
  float counts = amps * CURRENT_COUNTS_PER_AMP + 0.5f;
  if (counts >= CURRENT_COUNTS_MAX)
    return CURRENT_COUNTS_MAX;
  return (uint16_t)counts;
}

uint16_t Ramp(uint16_t now, uint16_t target, uint16_t step) {
  if (now + step < target)
    return now + step;
  if (now > target + step)
    return now - step;
  return target;
}

// ------------------------------------------------------------ hybrid, sent

void Build297Hybrid(uint8_t *out, uint8_t vehicle, uint8_t hvState,
                    uint8_t counter) {
  Copy(out, HYB297);
  out[0] = vehicle;
  out[1] = hvState;
  // D5:D6 carry a value during charging that we could not decode (it follows
  // the charge current, but with no clean scale). 0xFFFF is what the car
  // sends before high voltage is up, so that is what we send throughout.
  out[6] = FLAGS_297 | (HvActive(hvState) ? HV_FLAG_297 : 0) |
           (counter & COUNTER_MASK);
  out[7] = Xor7(out);
}

void Build29BHybrid(uint8_t *out, PackStage stage) {
  switch (stage) {
  case PACK_SLEEP:
    Copy(out, HYB29B_SLEEP);
    break;
  case PACK_AWAKE:
    Copy(out, HYB29B_AWAKE);
    break;
  case PACK_HV:
    Copy(out, HYB29B_HV);
    break;
  }
}

void Build29C(uint8_t *out, Command cmd, uint16_t currentCounts,
              uint16_t voltageCounts) {
  switch (cmd) {
  case CMD_SLEEP:
    Copy(out, CMD29C_SLEEP);
    return;
  case CMD_AWAKE:
  case CMD_STANDBY:
  case CMD_SHUTDOWN:
    Copy(out, CMD29C_AWAKE);
    return;
  case CMD_CHARGE:
  case CMD_STOPPING:
    break;
  }
  if (currentCounts > CURRENT_COUNTS_MAX)
    currentCounts = CURRENT_COUNTS_MAX;
  Copy(out, CMD29C_CHARGE);
  out[2] = CURRENT_FIXED_29C | (currentCounts >> 8);
  out[3] = currentCounts & 0xFF;
  PutWord(out, 6, voltageCounts);
}

void Build33F(uint8_t *out, Command cmd, uint16_t rampCounts,
              uint16_t voltageCounts) {
  switch (cmd) {
  case CMD_SLEEP:
    Copy(out, MODE33F_SLEEP);
    return;
  case CMD_AWAKE:
    Copy(out, MODE33F_AWAKE);
    return;
  case CMD_STANDBY:
    Copy(out, MODE33F_AWAKE);
    out[7] = STANDBY_33F;
    return;
  case CMD_SHUTDOWN:
    Copy(out, MODE33F_AWAKE);
    out[7] = SHUTDOWN_33F;
    return;
  case CMD_CHARGE:
  case CMD_STOPPING:
    break;
  }
  if (rampCounts > MODE_RAMP_MAX)
    rampCounts = MODE_RAMP_MAX;
  Copy(out, MODE33F_CHARGE);
  PutWord(out, 0, voltageCounts);
  out[5] = RAMP_FIXED_33F | (rampCounts >> 8);
  out[6] = rampCounts & 0xFF;
  if (cmd == CMD_STOPPING)
    out[7] = STOPPING_33F;
}

void Build394(uint8_t *out, bool awake, uint8_t counter) {
  Copy(out, HYB394);
  if (awake)
    out[4] = AWAKE_394;
  out[6] = counter & COUNTER_MASK;
  out[7] = Xor7(out);
}

void Build396(uint8_t *out, bool hvActive) {
  Copy(out, HYB396);
  if (hvActive)
    out[4] = HV_396;
}

void Build39A(uint8_t *out, uint16_t roll) {
  Copy(out, HYB39A);
  PutWord(out, 4, (roll & VALUE_MASK) << 6);
}

void Build39BHybrid(uint8_t *out, uint8_t counter) {
  Copy(out, HYB39B);
  out[6] = (counter & COUNTER_MASK) << 2;
  out[7] = Xor7(out);
}

// ---------------------------------------------------------------- PT, sent

void Build19C(uint8_t *out, bool dcdcOn, uint8_t counter) {
  // D3:D4 looks like a 10 bit setpoint of 672 (13.44 V at 0.02 V per count)
  // with the enable bit on top. Not proven, so the captured value is kept.
  Copy(out, PT19C);
  out[2] = dcdcOn ? DCDC_ON_19C : DCDC_OFF_19C;
  out[6] = counter & COUNTER_MASK;
}

void Build297PT(uint8_t *out, uint8_t hvState) {
  // A stripped gateway copy: no counter, no checksum, no pulse flags.
  Clear(out);
  out[1] = hvState & STATE_MASK;
  if (HvActive(hvState))
    out[6] = HV_FLAG_297;
}

void Build29BPT(uint8_t *out, PackStage stage) {
  Clear(out);
  switch (stage) {
  case PACK_SLEEP:
    out[0] = PT29B_SLEEP;
    break;
  case PACK_AWAKE:
    out[0] = PT29B_AWAKE;
    break;
  case PACK_HV:
    out[0] = PT29B_HV;
    break;
  }
}

// ---------------------------------------------------- sent by the charger

uint8_t ChargerMode(const uint8_t *frame324) { return frame324[0]; }

float HvVolts(const uint8_t *frame324) {
  return ((frame324[1] << 8) | frame324[2]) * HV_VOLTS_PER_COUNT;
}

bool PilotReady(const uint8_t *frame33B) {
  return (frame33B[0] & PILOT_READY_BIT) != 0;
}

uint8_t PilotDuty(const uint8_t *frame33B) {
  return frame33B[0] & PILOT_DUTY_MASK;
}

float PilotAmps(uint8_t dutyPercent) {
  // IEC 61851-1 table A.8. The charger reports whole percents, truncated, so
  // this reads up to 0.6 A low.
  if (dutyPercent < PILOT_PWM_MIN || dutyPercent > PILOT_PWM_MAX)
    return 0;
  if (dutyPercent < 10)
    return 6.0f;
  if (dutyPercent <= 85)
    return dutyPercent * 0.6f;
  if (dutyPercent <= 96)
    return (dutyPercent - 64) * 2.5f;
  return 80.0f;
}

float DcAmps(const uint8_t *frame33D) {
  return frame33D[6] * AMPS_PER_COUNT_33D;
}

uint8_t AcPhases(const uint8_t *frame491, float *amps, float *volts) {
  uint8_t live = 0;
  for (int phase = 0; phase < 3; phase++) {
    amps[phase] = frame491[phase * 2] * AMPS_PER_COUNT_491;
    volts[phase] = frame491[phase * 2 + 1] * VOLTS_PER_COUNT_491;
    if (frame491[phase * 2 + 1] != 0)
      live++;
  }
  return live;
}

float LvVolts(const uint8_t *frame39F) {
  return frame39F[1] * LV_VOLTS_PER_COUNT;
}

float LvAmps(const uint8_t *frame39F) { return frame39F[4]; }

} // namespace MGgen2Protocol
