#include "MoteusAPI.h"

#include <stdio.h>   // Standard input/output definitions
#include "MoteusAPI.h"
#include "moteus.h"

using namespace mjbots;
using namespace moteus;

MoteusAPI::MoteusAPI(int moteus_id)
    : moteus_id_(moteus_id) {
}

MoteusAPI::~MoteusAPI() {
}

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
bool MoteusAPI::TestCommand() {
    moteus::Controller::Options options;
    options.id = moteus_id_;
    options.default_query = false;
    moteus::Controller controller(options);

      moteus::Query::Format q_com;

      const auto maybe_result = controller.SetQuery(&q_com);
      if (maybe_result)
      {
        const auto r = maybe_result->values;
        printf("Mode\t=\t%3d\nPosition\t=\t%7.6f\nVelocity\t=\t%7.6f\nTorque\t=\t%7.6f\nVoltage\t= \t%5.1f\nTemperature\t= \t%5.1f\nFault\t= \t%3d\n",
               static_cast<int>(r.mode),
               r.position,
               r.velocity,
               r.torque,
               r.voltage,
               r.temperature,
               r.fault);
        ::fflush(stdout);
      }

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
