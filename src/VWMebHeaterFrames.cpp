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

#include "VWMebHeaterFrames.h"

namespace VWMebHeaterProtocol {

void BuildCommand(uint8_t frame[COMMAND_LEN], bool on, uint8_t dutyPercent,
                  CommandLayout layout) {
  if (dutyPercent > 100)
    dutyPercent = 100;

  for (int i = 0; i < COMMAND_LEN; i++)
    frame[i] = 0;

  if (layout == LAYOUT_NOTES) {
    frame[2] = (dutyPercent & 0x3F) | (on ? 0x40 : 0);
    frame[3] = (dutyPercent >> 6) & 0x03;
  } else {
    // LIN numbers signal bits from the least significant bit of byte 0.
    uint32_t word = ((uint32_t)(on ? 1 : 0) << 18) |
                    ((uint32_t)(dutyPercent & 0x7F) << 19);
    for (int i = 0; i < COMMAND_LEN; i++)
      frame[i] = (word >> (8 * i)) & 0xFF;
  }
}

void DecodeFeedback(const uint8_t frame[FEEDBACK_LEN], Feedback &fb) {
  fb.hvCurrent = frame[0] * 0.25f;
  fb.hvVoltage = frame[1] * 2.0f;
  fb.status = frame[2];
  fb.lvVoltage = frame[4] * 0.1f;
  fb.tempHeater = (int)frame[5] - TEMPERATURE_OFFSET;
  fb.tempIn = (int)frame[6] - TEMPERATURE_OFFSET;
  fb.tempOut = (int)frame[7] - TEMPERATURE_OFFSET;
}

uint16_t StepDuty(uint16_t currentTenths, uint8_t targetPercent) {
  if (targetPercent > DUTY_MAX)
    targetPercent = DUTY_MAX;

  uint16_t target = targetPercent * 10;

  if (currentTenths >= target)
    return target;

  uint16_t next = currentTenths + DUTY_STEP;
  return next > target ? target : next;
}

} // namespace VWMebHeaterProtocol
