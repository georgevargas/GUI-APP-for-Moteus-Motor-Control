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

#include "MoteusAPI.h"

#include <errno.h>   // Error number definitions
#include <fcntl.h>   // File control definitions
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <sys/ioctl.h>
#include <termios.h>  // POSIX terminal control definitions
#include <unistd.h>   // UNIX standard function definitions
#include "MoteusAPI.h"

#include <iostream>
#include "moteus.h"

using namespace mjbots;
using namespace moteus;

MoteusAPI::MoteusAPI(const string dev_name, int moteus_id)
    : dev_name_(dev_name), moteus_id_(moteus_id) {
  OpenDev();
}

MoteusAPI::~MoteusAPI() { CloseDev(); }

bool MoteusAPI::SendPositionCommand(double position,
                                    double velocity_limit,
                                    double accel_limit,
                                    double max_torque,
                                    double feedforward_torque,
                                    double kp_scale,
                                    double kd_scale,
                                    double velocity,
                                    double watchdog_timer) const {
    moteus::PositionMode::Command cmd;
    cmd.position = position;
    cmd.velocity = velocity;
    cmd.maximum_torque = max_torque;
    cmd.stop_position = std::numeric_limits<double>::quiet_NaN();
    cmd.kp_scale = kp_scale;
    cmd.kd_scale = kd_scale;
    cmd.feedforward_torque = feedforward_torque;
    cmd.velocity_limit = velocity_limit;
    cmd.accel_limit = accel_limit;
    cmd.watchdog_timeout = watchdog_timer;

    moteus::PositionMode::Format res;
    res.position = mjbots::moteus::Resolution::kFloat;
    res.velocity = mjbots::moteus::Resolution::kFloat;
    res.feedforward_torque = mjbots::moteus::Resolution::kFloat;
    res.kp_scale = mjbots::moteus::Resolution::kFloat;
    res.kd_scale = mjbots::moteus::Resolution::kFloat;
    res.maximum_torque = mjbots::moteus::Resolution::kFloat;
    res.stop_position = mjbots::moteus::Resolution::kFloat;
    res.watchdog_timeout = mjbots::moteus::Resolution::kFloat;
    res.velocity_limit = mjbots::moteus::Resolution::kFloat;
    res.accel_limit = mjbots::moteus::Resolution::kFloat;
    res.fixed_voltage_override = mjbots::moteus::Resolution::kIgnore;

    moteus::Controller::Options options;
    options.id = moteus_id_;
    options.default_query = false;
    moteus::Controller controller(options);

    controller.SetPosition(cmd,&res);
    return true;

}

bool MoteusAPI::SendStopCommand() {
    moteus::Controller::Options options;
    options.id = moteus_id_;
    options.default_query = false;
    moteus::Controller controller(options);

    // Command a stop to the controller in order to clear any faults.
    controller.SetStop();

  return true;
}

void MoteusAPI::ReadState(State& curr_state) const {
    moteus::Query::Format q_com;
    q_com.mode = mjbots::moteus::Resolution::kInt8;
    q_com.position = mjbots::moteus::Resolution::kFloat;
    q_com.velocity = mjbots::moteus::Resolution::kFloat;
    q_com.torque = mjbots::moteus::Resolution::kFloat;
    q_com.q_current = mjbots::moteus::Resolution::kFloat;
    q_com.d_current = mjbots::moteus::Resolution::kFloat;
    q_com.abs_position = mjbots::moteus::Resolution::kFloat;
    q_com.motor_temperature = mjbots::moteus::Resolution::kFloat;
    q_com.trajectory_complete = mjbots::moteus::Resolution::kInt16;
    q_com.home_state = mjbots::moteus::Resolution::kIgnore;
    q_com.voltage = mjbots::moteus::Resolution::kFloat;
    q_com.temperature = mjbots::moteus::Resolution::kFloat;
    q_com.fault = mjbots::moteus::Resolution::kInt8;

  if (!curr_state.position_flag)
    q_com.position = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.velocity_flag)
    q_com.velocity = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.torque_flag)
    q_com.torque = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.q_curr_flag)
    q_com.q_current = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.d_curr_flag)
    q_com.d_current = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.abs_position_flag)
    q_com.abs_position = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.motor_temperature_flag)
    q_com.motor_temperature = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.home_state_flag)
    q_com.home_state = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.TrajectoryComplete_flag)
    q_com.trajectory_complete = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.voltage_flag)
    q_com.voltage = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.temperature_flag)
    q_com.temperature = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.fault_flag)
    q_com.fault = mjbots::moteus::Resolution::kIgnore;
  if (!curr_state.mode_flag)
    q_com.mode = mjbots::moteus::Resolution::kIgnore;

  moteus::Controller::Options options;
  options.id = moteus_id_;
  options.default_query = false;
  moteus::Controller controller(options);

  const auto maybe_result = controller.SetQuery(&q_com);
  const auto qr = maybe_result->values;
  curr_state.position = qr.position;
  curr_state.velocity = qr.velocity;
  curr_state.torque = qr.torque;
  curr_state.q_curr = qr.q_current;
  curr_state.d_curr = qr.d_current;
  curr_state.voltage = qr.voltage;
  curr_state.temperature = qr.temperature;
  curr_state.fault = qr.fault;
  curr_state.mode = static_cast<float>(qr.mode);
  curr_state.TrajectoryComplete = qr.trajectory_complete;
}

bool MoteusAPI::ExpectResponse(const string& exp_string,
                               string& fullresp) const {
  char read_buff[readbuffsize];
  for (uint ii = 0; ii < readbuffsize; ii++) {
    read_buff[ii] = 0;
  }
  do {
    int res = ReadUntilDev(read_buff, '\n', readbuffsize, 1000);
    if (res) {
      cout << "Timeout: Expected response'" << exp_string
           << "' was not received" << endl;
      return false;
    }
    fullresp = string(read_buff);
  } while (fullresp.find(exp_string) == std::string::npos);
  return true;
}

int MoteusAPI::OpenDev() {
  struct termios toptions;
  int fd;

  fd = open(dev_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd == -1) {
    throw std::runtime_error("MoteusAPI: Unable to open port");
    exit(EXIT_FAILURE);
  }

  if (tcgetattr(fd, &toptions) < 0) {
    throw std::runtime_error("MoteusAPI: Couldn't get term attributes");
    exit(EXIT_FAILURE);
  }

  // set baud to arbitrary value, it will get ignored by dev
  speed_t brate = B115200;

  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag |= CREAD | CLOCAL;
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  toptions.c_oflag &= ~OPOST;

  toptions.c_cc[VMIN] = 0;
  toptions.c_cc[VTIME] = 0;

  tcsetattr(fd, TCSANOW, &toptions);
  if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
    throw std::runtime_error("MoteusAPI: Couldn't set term attributes");
  }

  fd_ = fd;
  return fd_;
}

bool MoteusAPI::WriteDev(const string& buff) const {
  uint n = write(fd_, buff.c_str(), buff.size());
  if (n != buff.size()) {
    return false;
  }
  return true;
}

int MoteusAPI::CloseDev() const { return close(fd_); }

int MoteusAPI::ReadUntilDev(char* buf, char until, int buf_max,
                            int timeout) const {
  char b[1];  // read expects an array, so we give it a 1-byte array
  int i = 0;
  do {
    int n = read(fd_, b, 1);  // read a char at a time
    if (n == -1) return -1;   // couldn't read
    if (n == 0) {
      usleep(1 * 1000);  // wait 1 msec try again
      timeout--;
      if (timeout == 0) return -2;
      continue;
    }
    buf[i] = b[0];
    i++;
  } while (b[0] != until && i < buf_max && timeout > 0);

  buf[i] = 0;  // null terminate the string
  return 0;
}
