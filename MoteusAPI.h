// Copyright 2021 Sina Aghli, sinaaghli.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTEUSAPI_H__
#define MOTEUSAPI_H__

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus_protocol.h"

using namespace std;

enum class Fault {
  kNoFault = 0,
  kCalibrationFault = 32,
  kMotorDriverFault = 33,
  kOverVoltageFault = 34,
  kEncoderFault = 35,
  kMotorNotConfiguredFault = 36,
  kPwmCycleOverrunFault = 37,
  kOverTemperatureFault = 38,
  kOutsideLimitFault = 39,
};

struct State {
  double position = NAN;
  double velocity = NAN;
  double torque = NAN;
  double q_curr = NAN;
  double d_curr = NAN;
  double rezero_state = NAN;
  double voltage = NAN;
  double temperature = NAN;
  double fault = NAN;
  double mode = NAN;
  bool TrajectoryComplete = false;
  // flags
  bool mode_flag = false;
  bool position_flag = false;
  bool velocity_flag = false;
  bool torque_flag = false;
  bool q_curr_flag = false;
  bool d_curr_flag = false;
  bool abs_position_flag = false;
  bool motor_temperature_flag = false;
  bool TrajectoryComplete_flag = false;
  bool home_state_flag = false;
  bool voltage_flag = false;
  bool temperature_flag = false;
  bool fault_flag = false;

  State& EN_Position() {
    position_flag = true;
    return *this;
  }
  State& EN_Velocity() {
    velocity_flag = true;
    return *this;
  }
  State& EN_Torque() {
    torque_flag = true;
    return *this;
  }
  State& EN_QCurr() {
    q_curr_flag = true;
    return *this;
  }
  State& EN_DCurr() {
    d_curr_flag = true;
    return *this;
  }
  State& EN_abs_position() {
    abs_position_flag = true;
    return *this;
  }
  State& motor_temperature() {
    motor_temperature_flag = true;
    return *this;
  }
  State& EN_TrajectoryComplete() {
    TrajectoryComplete_flag = true;
    return *this;
  }
  State& EN_home_state() {
    home_state_flag = true;
    return *this;
  }
  State& EN_Voltage() {
    voltage_flag = true;
    return *this;
  }
  State& EN_Temp() {
    temperature_flag = true;
    return *this;
  }
  State& EN_Fault() {
    fault_flag = true;
    return *this;
  }
  State& EN_Mode() {
    mode_flag = true;
    return *this;
  }

  void Reset() {
    position_flag = false;
    velocity_flag = false;
    torque_flag = false;
    q_curr_flag = false;
    d_curr_flag = false;
    abs_position_flag = false;
    motor_temperature_flag = false;
    home_state_flag = false;
    voltage_flag = false;
    temperature_flag = false;
    fault_flag = false;
    mode_flag = false;
    TrajectoryComplete_flag = false;
  }
};

class MoteusAPI {
 public:
  MoteusAPI(const string dev_name, int moteus_id);
  ~MoteusAPI();

  bool SendPositionCommand(double position,
                           double velocity_limit,
                           double accel_limit,
                           double max_torque,
                           double feedforward_torque,
                           double kp_scale,
                           double kd_scale,
                           double velocity, // end velocity
                           double watchdog_timer = NAN) const;

  bool SendStopCommand();

  bool SendDiagnosticCommand(string msg);

  void ReadState(State& curr_state) const;

 private:
  // Open /dev/dev_name_
  int OpenDev();
  int CloseDev() const;
  bool WriteDev(const string& buff) const;
  int ReadUntilDev(char* buf, char until, int buf_max, int timeout) const;
  bool ExpectResponse(const string& exp_string, string& resp) const;
  const string dev_name_;
  const int moteus_id_;
  int fd_;
  const unsigned int readbuffsize = 500;
  // const unsigned long timeoutdelayus = 1000;
};

#endif  // MOTEUSAPI_H__
