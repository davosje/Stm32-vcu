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

#include "VWMebHeater.h"
#include "VWMebHeaterFrames.h"
#include "params.h"

using namespace VWMebHeaterProtocol;

// 20 steps of 50 ms without feedback = 1 s.
static const uint8_t SILENT_LIMIT = 20;

void vwMebHeater::SetLinInterface(LinBus *l) {
  lin = l;
  DigIo::lin_wake.Clear(); // not used on the TJA1027
  DigIo::lin_nslp.Set();   // transceiver out of sleep
}

void vwMebHeater::DeInit() {
  dutyTenths = 0;
  ClearFeedback();
}

// Called every 10 ms by ControlCabHeater(), with HeatReq off and power 0
// when the heater is not allowed to run. Power in watts is not used: like
// the J840, we command a duty, taken from HeatPercnt.
void vwMebHeater::SetPower(uint16_t power, bool HeatReq) {
  (void)power;

  if (lin == nullptr)
    return;

  if (++tick < 5) // run every 50 ms
    return;
  tick = 0;

  // The answer to the feedback request sent 50 ms ago, if it came.
  if (lin->HasReceived(FRAME_FEEDBACK, FEEDBACK_LEN)) {
    ShowFeedback(lin->GetReceivedBytes());
    silentSteps = 0;
  } else if (silentSteps < SILENT_LIMIT) {
    silentSteps++;
    if (silentSteps == SILENT_LIMIT)
      ClearFeedback();
  }

  uint8_t target = HeatReq ? Param::GetInt(Param::HeatPercnt) : 0;
  dutyTenths = StepDuty(dutyTenths, target);

  if (readNext) {
    lin->Request(FRAME_FEEDBACK, 0, 0);
  } else {
    uint8_t frame[COMMAND_LEN];
    uint8_t duty = dutyTenths / 10;
    CommandLayout layout = (CommandLayout)Param::GetInt(Param::MebHtBits);
    BuildCommand(frame, duty > 0, duty, layout);
    lin->Request(FRAME_COMMAND, frame, COMMAND_LEN);
  }

  readNext = !readNext;
}

void vwMebHeater::ShowFeedback(const uint8_t *data) {
  Feedback fb;
  DecodeFeedback(data, fb);

  Param::SetInt(Param::tmpheater, fb.tempHeater);
  Param::SetInt(Param::tmpheatin, fb.tempIn);
  Param::SetInt(Param::tmpheatout, fb.tempOut);
  Param::SetFloat(Param::udcheater, fb.hvVoltage);
  Param::SetFloat(Param::powerheater, fb.hvVoltage * fb.hvCurrent);
  Param::SetFloat(Param::u12heater, fb.lvVoltage);
}

void vwMebHeater::ClearFeedback() {
  Param::SetInt(Param::udcheater, 0);
  Param::SetInt(Param::powerheater, 0);
  Param::SetInt(Param::u12heater, 0);
}
