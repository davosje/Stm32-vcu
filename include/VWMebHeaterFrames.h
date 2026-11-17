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
#ifndef VWMEBHEATERFRAMES_H
#define VWMEBHEATERFRAMES_H

/* LIN frames of the VW MEB high-voltage PTC coolant heater, J848,
 * part number 1EE 963 231 (ID.3, ID.4 and the rest of the MEB family).
 *
 * This is NOT the heater that VWCoolantHeater drives (3Q0 963 231). Both
 * use frame 0x1C for the command, but the payload and the feedback frame
 * differ.
 *
 * Nothing in here touches parameters, LIN hardware or timers, so the
 * protocol can be tested on a PC: see test/test_vwmebheater.cpp.
 *
 * Source: the LIN traffic between the J840 battery controller and the
 * heater in an ID.4, as described in github.com/jagheterfredrik/meb-preheat
 * (NOTES.md and J848.ldf, read 2026-11-17). That repository has no licence,
 * so only the protocol facts are used here, no code.
 *
 *   LIN 2.1, 19.2 kbit/s, enhanced checksum, heater NAD 0x04
 *   0x0F  feedback, published by the heater, 8 bytes
 *   0x1C  command, published by the master, 4 bytes
 *   The J840 polls the two alternately every 50 ms.
 *
 *   Connector (black, 1J0973714): pin 1 +12 V, pin 2 ground, pin 4 LIN.
 *
 * Feedback 0x0F:
 *   byte 0  HV current, 0.25 A per bit
 *   byte 1  HV voltage at the connector, 2 V per bit
 *   byte 2  status, always 0 in the captures
 *   byte 3  unknown, always 0
 *   byte 4  12 V supply, 0.1 V per bit
 *   byte 5  heater temperature, offset -50 degC
 *   byte 6  coolant in, offset -50 degC
 *   byte 7  coolant out, offset -50 degC
 *
 * Command 0x1C: on/off and a duty of 0..100 %. The two sources disagree on
 * where those bits are, so both layouts are here and parameter MebHtBits
 * picks one. Find out on the bench which one the heater answers to.
 *
 *   LAYOUT_LDF    J848.ldf: on/off at bit 18, duty at bits 19..25
 *   LAYOUT_NOTES  NOTES.md: on/off at byte 2 bit 6, duty at byte 2 bits
 *                 0..5 plus byte 3 bits 0..1
 *
 * All other bits are unknown and sent as zero.
 *
 * The J840 raises the duty by 2 % per second and stops at 99 %.
 * StepDuty() does the same.
 */

#include <stdint.h>

namespace VWMebHeaterProtocol {

const uint8_t FRAME_FEEDBACK = 0x0F;
const uint8_t FRAME_COMMAND = 0x1C;
const uint8_t FEEDBACK_LEN = 8;
const uint8_t COMMAND_LEN = 4;

const uint8_t DUTY_MAX = 99;      // highest duty the J840 uses, in %
const uint16_t DUTY_STEP = 1;     // tenths of a % per 50 ms step = 2 %/s
const int TEMPERATURE_OFFSET = 50;

enum CommandLayout { LAYOUT_LDF = 0, LAYOUT_NOTES = 1 };

struct Feedback {
  float hvCurrent;   // A
  float hvVoltage;   // V
  float lvVoltage;   // V
  int tempHeater;    // degC
  int tempIn;        // degC
  int tempOut;       // degC
  uint8_t status;
};

// Fills the 4 bytes of command frame 0x1C. Duty above 100 is sent as 100.
void BuildCommand(uint8_t frame[COMMAND_LEN], bool on, uint8_t dutyPercent,
                  CommandLayout layout);

// Turns the 8 bytes of feedback frame 0x0F into values.
void DecodeFeedback(const uint8_t frame[FEEDBACK_LEN], Feedback &fb);

// One 50 ms step of the duty ramp, in tenths of a percent. Rises by
// DUTY_STEP towards the target, drops to the target at once. The target is
// limited to DUTY_MAX.
uint16_t StepDuty(uint16_t currentTenths, uint8_t targetPercent);

} // namespace VWMebHeaterProtocol

#endif // VWMEBHEATERFRAMES_H
