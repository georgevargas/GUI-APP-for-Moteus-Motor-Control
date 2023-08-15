// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

/// @file
///
/// This file contains primitives used to encode and decode CAN-FD
/// messages for the mjbots moteus brushless controller.  While C++,
/// it is designed to be usable in "minimal-C++" environments like
/// Arduino, in addition to fully standards conforming environments.

#pragma once

#include <string.h>

#ifndef ARDUINO

#include <limits>
#define NaN std::numeric_limits<double>::quiet_NaN();

#else

#define NaN (0.0 / 0.0)

#endif


#include "moteus_multiplex.h"

namespace mjbots {
namespace moteus {

/// This is a single CAN-FD frame, its headers, and other associated
/// metadata, like which bus the message was sent or received from in
/// multi-bus systems.
struct CanFdFrame {
  ///////////////////////////////////////////
  /// First are the raw data from the bus.

  // Non-zero if this transport supports multiple busses.
  int8_t bus = 0;

  uint32_t arbitration_id = 0;
  uint8_t data[64] = {};
  uint8_t size = 0;

  enum Toggle {
    kDefault,
    kForceOff,
    kForceOn,
  };

  Toggle brs = kDefault;
  Toggle fdcan_frame = kDefault;

  //////////////////////////////////////////
  /// Next are parsed items for moteus frames.  These are not required
  /// to be set when sending a frame to a transport, but they will be
  /// filled in on receipt.  Higher level layers than the transport
  /// layer may use them to populate the bus-level data.

  // If true, then the ID used is not calculated from destination and
  // source, but is instead determined directly from arbitration_id.
  bool raw = false;

  int8_t destination = 0;
  int8_t source = 0;
  bool reply_required = false;
  uint16_t can_prefix = 0x0000;  // A 13 bit CAN prefix
};


/// The expected version associated with register 0x102.  If it
/// differs from this, then semantics of one or more registers may
/// have changed.
enum {
  kCurrentRegisterMapVersion = 5,
};

enum Register : uint16_t {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTorque = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kAbsPosition = 0x006,

  kMotorTemperature = 0x00a,
  kTrajectoryComplete = 0x00b,
  kHomeState = 0x00c,
  kVoltage = 0x00d,
  kTemperature = 0x00e,
  kFault = 0x00f,

  kPwmPhaseA = 0x010,
  kPwmPhaseB = 0x011,
  kPwmPhaseC = 0x012,

  kVoltagePhaseA = 0x014,
  kVoltagePhaseB = 0x015,
  kVoltagePhaseC = 0x016,

  kVFocTheta = 0x018,
  kVFocVoltage = 0x019,
  kVoltageDqD = 0x01a,
  kVoltageDqQ = 0x01b,

  kCommandQCurrent = 0x01c,
  kCommandDCurrent = 0x01d,

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandFeedforwardTorque = 0x022,
  kCommandKpScale = 0x023,
  kCommandKdScale = 0x024,
  kCommandPositionMaxTorque = 0x025,
  kCommandStopPosition = 0x026,
  kCommandTimeout = 0x027,

  kPositionKp = 0x030,
  kPositionKi = 0x031,
  kPositionKd = 0x032,
  kPositionFeedforward = 0x033,
  kPositionCommand = 0x034,

  kControlPosition = 0x038,
  kControlVelocity = 0x039,
  kControlTorque = 0x03a,
  kControlPositionError = 0x03b,
  kControlVelocityError = 0x03c,
  kControlTorqueError = 0x03d,

  kCommandStayWithinLowerBound = 0x040,
  kCommandStayWithinUpperBound = 0x041,
  kCommandStayWithinFeedforwardTorque = 0x042,
  kCommandStayWithinKpScale = 0x043,
  kCommandStayWithinKdScale = 0x044,
  kCommandStayWithinPositionMaxTorque = 0x045,
  kCommandStayWithinTimeout = 0x046,

  kEncoder0Position = 0x050,
  kEncoder0Velocity = 0x051,
  kEncoder1Position = 0x052,
  kEncoder1Velocity = 0x053,
  kEncoder2Position = 0x054,
  kEncoder2Velocity = 0x055,

  kEncoderValidity = 0x058,

  kAux1GpioCommand = 0x05c,
  kAux2GpioCommand = 0x05d,
  kAux1GpioStatus = 0x05e,
  kAux2GpioStatus = 0x05f,

  kAux1AnalogIn1 = 0x060,
  kAux1AnalogIn2 = 0x061,
  kAux1AnalogIn3 = 0x062,
  kAux1AnalogIn4 = 0x063,
  kAux1AnalogIn5 = 0x064,

  kAux2AnalogIn1 = 0x068,
  kAux2AnalogIn2 = 0x069,
  kAux2AnalogIn3 = 0x06a,
  kAux2AnalogIn4 = 0x06b,
  kAux2AnalogIn5 = 0x06c,

  kMillisecondCounter = 0x070,
  kClockTrim = 0x071,

  kRegisterMapVersion = 0x102,
  kSerialNumber = 0x120,
  kSerialNumber1 = 0x120,
  kSerialNumber2 = 0x121,
  kSerialNumber3 = 0x122,

  kRezero = 0x130,
  kSetOutputNearest = 0x130,
  kSetOutputExact = 0x131,
  kRequireReindex = 0x132,

  kDriverFault1 = 0x140,
  kDriverFault2 = 0x141,
};

enum class Mode {
  kStopped = 0,
  kFault = 1,
  kEnabling = 2,
  kCalibrating = 3,
  kCalibrationComplete = 4,
  kPwm = 5,
  kVoltage = 6,
  kVoltageFoc = 7,
  kVoltageDq = 8,
  kCurrent = 9,
  kPosition = 10,
  kPositionTimeout = 11,
  kZeroVelocity = 12,
  kStayWithin = 13,
  kMeasureInd = 14,
  kBrake = 15,
  kNumModes,
};

enum class HomeState {
  kRelative = 0,
  kRotor = 1,
  kOutput = 2,
};


////////////////////////////////////////////////////////////////
// Each command that can be issued is represented by a structure below
// with a few different sub-structs.  Possibilities are:
//
//  'Command' items that are serialized in an outgoing message
//  'Format'  the corresponding resolution for each item in 'Command'
//  'Result'  the deserialized version of a response
//
// Additionally, there are two static methods possible:
//
//  'Make' - take a Command and Format, and serialize it
//  'Parse' - deserialize CAN data into a 'Result' structure

struct EmptyMode {
  struct Command {};
  struct Format {};
  static void Make(WriteCanData*, const Command&, const Format&) {}
};

struct Query {
  struct ItemValue {
    int16_t register_number = detail::numeric_limits<int16_t>::max();
    double value = 0.0;
  };

  static constexpr int16_t kMaxExtra = 16;

  struct Result {
    Mode mode = Mode::kStopped;
    double position = NaN;
    double velocity = NaN;
    double torque = NaN;
    double q_current = NaN;
    double d_current = NaN;
    double abs_position = NaN;
    double motor_temperature = NaN;
    bool trajectory_complete = false;
    HomeState home_state = HomeState::kRelative;
    double voltage = NaN;
    double temperature = NaN;
    int8_t fault = 0;

    int8_t aux1_gpio = 0;
    int8_t aux2_gpio = 0;

    // Before gcc-12, initializating non-POD array types can be
    // painful if done in the idiomatic way with ={} inline.  Instead
    // we do it in the constructor.
    //
    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=92385
    ItemValue extra[kMaxExtra];

    Result() : extra() {}
  };

  struct ItemFormat {
    int16_t register_number = detail::numeric_limits<int16_t>::max();
    Resolution resolution = moteus::kIgnore;
  };

  struct Format {
    Resolution mode = kInt8;
    Resolution position = kFloat;
    Resolution velocity = kFloat;
    Resolution torque = kFloat;
    Resolution q_current = kFloat;     // was kIgnore
    Resolution d_current = kFloat;     // was kIgnore
    Resolution abs_position = kFloat;     // was kIgnore
    Resolution motor_temperature = kFloat;     // was kIgnore
    Resolution trajectory_complete = kInt16;  // was kIgnore
    Resolution home_state = kIgnore;
    Resolution voltage = kFloat;
    Resolution temperature = kFloat;
    Resolution fault = kInt8;

    Resolution aux1_gpio = kIgnore;
    Resolution aux2_gpio = kIgnore;

    ItemFormat extra[kMaxExtra];

    // gcc bug 92385 again
    Format() : extra() {}
  };

  static int ItemFormatSort(const void* lhs_in, const void* rhs_in) {
    const auto* lhs = reinterpret_cast<const ItemFormat*>(lhs_in);
    const auto* rhs = reinterpret_cast<const ItemFormat*>(rhs_in);

    return lhs->register_number - rhs->register_number;
  }

  static void Make(WriteCanData* frame, const Format& format) {
    {
      const Resolution kResolutions[] = {
        format.mode,
        format.position,
        format.velocity,
        format.torque,
        format.q_current,
        format.d_current,
        format.abs_position,
      };
      const uint16_t kResolutionsSize = sizeof(kResolutions) / sizeof(*kResolutions);
      WriteCombiner combiner(
          frame, 0x10, Register::kMode,
          kResolutions, kResolutionsSize);
      for (uint16_t i = 0; i < kResolutionsSize; i++) {
        combiner.MaybeWrite();
      }
    }

    {
      const Resolution kResolutions[] = {
        format.motor_temperature,
        format.trajectory_complete,
        format.home_state,
        format.voltage,
        format.temperature,
        format.fault,
      };
      const uint16_t kResolutionsSize = sizeof(kResolutions) / sizeof(*kResolutions);
      WriteCombiner combiner(
          frame, 0x10, Register::kMotorTemperature,
          kResolutions, kResolutionsSize);
      for (uint16_t i = 0; i < kResolutionsSize; i++) {
        combiner.MaybeWrite();
      }
    }

    {
      const Resolution kResolutions[] = {
        format.aux1_gpio,
        format.aux2_gpio,
      };
      const uint16_t kResolutionsSize = sizeof(kResolutions) / sizeof(*kResolutions);
      WriteCombiner combiner(
          frame, 0x10, Register::kAux1GpioStatus,
          kResolutions, kResolutionsSize);
      for (uint16_t i = 0; i < kResolutionsSize; i++) {
        combiner.MaybeWrite();
      }
    }

    {
      ItemFormat sorted_values[kMaxExtra] = {};
      ::memcpy(&sorted_values[0], &(format.extra[0]), sizeof(format.extra));
      ::qsort(&sorted_values[0], kMaxExtra, sizeof(ItemFormat), ItemFormatSort);

      const int16_t size = [&]() {
        for (int16_t i = 0; i < kMaxExtra; i++) {
          if (sorted_values[i].register_number ==
              detail::numeric_limits<int16_t>::max()) {
            return i;
          }
        }
        return kMaxExtra;
      }();

      if (size == 0) { return; }
      const int16_t min_register_number = sorted_values[0].register_number;
      const int16_t max_register_number = sorted_values[size - 1].register_number;

      const uint16_t required_registers =
          max_register_number - min_register_number + 1;

      // An arbitrary limit to constrain the amount of stack we use
      // below.
      if (required_registers > 512) { ::abort(); }

      Resolution resolutions[required_registers];
      ::memset(&resolutions[0], 0, sizeof(Resolution) * required_registers);

      for (int16_t this_register = min_register_number, index = 0;
           this_register <= max_register_number;
           this_register++) {
        if (sorted_values[index].register_number == this_register) {
          resolutions[this_register - min_register_number] =
              sorted_values[index].resolution;
          index++;
        } else {
          resolutions[this_register - min_register_number] = kIgnore;
        }
      }
      WriteCombiner combiner(
          frame, 0x10, min_register_number,
          resolutions, required_registers);
      for (uint16_t i = 0; i < required_registers; i++) {
        combiner.MaybeWrite();
      }
    }
  }

  static Result Parse(const uint8_t* data, uint8_t size) {
    MultiplexParser parser(data, size);

    return Parse(&parser);
  }

  static Result Parse(const CanData* frame) {
    MultiplexParser parser(frame);

    return Parse(&parser);
  }

  static Result Parse(MultiplexParser* parser) {
    Result result;

    int16_t extra_index = 0;

    while (true) {
      const auto current = parser->next();
      if (current.done) { return result; }

      const auto res = current.resolution;

      switch (static_cast<Register>(current.value)) {
        case Register::kMode: {
          result.mode = static_cast<Mode>(parser->ReadInt(res));
          break;
        }
        case Register::kPosition: {
          result.position = parser->ReadPosition(res);
          break;
        }
        case Register::kVelocity: {
          result.velocity = parser->ReadVelocity(res);
          break;
        }
        case Register::kTorque: {
          result.torque = parser->ReadTorque(res);
          break;
        }
        case Register::kQCurrent: {
          result.q_current = parser->ReadCurrent(res);
          break;
        }
        case Register::kDCurrent: {
          result.d_current = parser->ReadCurrent(res);
          break;
        }
        case Register::kAbsPosition: {
          result.abs_position = parser->ReadPosition(res);
          break;
        }
        case Register::kMotorTemperature: {
          result.motor_temperature = parser->ReadTemperature(res);
          break;
        }
        case Register::kTrajectoryComplete: {
          result.trajectory_complete = parser->ReadInt(res) != 0;
          break;
        }
        case Register::kHomeState: {
          result.home_state = static_cast<HomeState>(parser->ReadInt(res));
          break;
        }
        case Register::kVoltage: {
          result.voltage = parser->ReadVoltage(res);
          break;
        }
        case Register::kTemperature: {
          result.temperature = parser->ReadTemperature(res);
          break;
        }
        case Register::kFault: {
          result.fault = parser->ReadInt(res);
          break;
        }
        case Register::kAux1GpioStatus: {
          result.aux1_gpio = parser->ReadInt(res);
          break;
        }
        case Register::kAux2GpioStatus: {
          result.aux2_gpio = parser->ReadInt(res);
          break;
        }
        default: {
          if (extra_index < kMaxExtra) {
            result.extra[extra_index].register_number = current.value;
            result.extra[extra_index].value =
                ParseGeneric(parser, current.value, res);
            extra_index++;
          } else {
            // Ignore this value.
            parser->ReadInt(res);
          }
          break;
        }
      }
    }
  }

  static double ParseGeneric(MultiplexParser* parser,
                             int16_t register_number,
                             Resolution resolution) {
    const auto res = resolution;
    using R = Register;
    switch (static_cast<Register>(register_number)) {
      case R::kMode:      { return parser->ReadInt(res); }
      case R::kPosition:  { return parser->ReadPosition(res); }
      case R::kVelocity:  { return parser->ReadVelocity(res); }
      case R::kTorque:    { return parser->ReadTorque(res); }
      case R::kQCurrent:  { return parser->ReadCurrent(res); }
      case R::kDCurrent:  { return parser->ReadCurrent(res); }
      case R::kAbsPosition: { return parser->ReadPosition(res); }

      case R::kMotorTemperature: { return parser->ReadTemperature(res); }
      case R::kTrajectoryComplete: { return parser->ReadInt(res); }
      case R::kHomeState: { return parser->ReadInt(res); }
      case R::kVoltage:   { return parser->ReadVoltage(res); }
      case R::kTemperature: { return parser->ReadTemperature(res); }
      case R::kFault:     { return parser->ReadInt(res); }

      case R::kPwmPhaseA: { return parser->ReadPwm(res); }
      case R::kPwmPhaseB: { return parser->ReadPwm(res); }
      case R::kPwmPhaseC: { return parser->ReadPwm(res); }

      case R::kVoltagePhaseA: { return parser->ReadVoltage(res); }
      case R::kVoltagePhaseB: { return parser->ReadVoltage(res); }
      case R::kVoltagePhaseC: { return parser->ReadVoltage(res); }

      case R::kVFocTheta: { return parser->ReadPwm(res) * M_PI; }
      case R::kVFocVoltage: { return parser->ReadVoltage(res); }
      case R::kVoltageDqD: { return parser->ReadVoltage(res); }
      case R::kVoltageDqQ: { return parser->ReadVoltage(res); }

      case R::kCommandQCurrent: { return parser->ReadCurrent(res); }
      case R::kCommandDCurrent: { return parser->ReadCurrent(res); }

      case R::kCommandPosition: { return parser->ReadPosition(res); }
      case R::kCommandVelocity: { return parser->ReadVelocity(res); }
      case R::kCommandFeedforwardTorque: { return parser->ReadTorque(res); }
      case R::kCommandKpScale: { return parser->ReadPwm(res); }
      case R::kCommandKdScale: { return parser->ReadPwm(res); }
      case R::kCommandPositionMaxTorque: { return parser->ReadTorque(res); }
      case R::kCommandStopPosition: { return parser->ReadPosition(res); }
      case R::kCommandTimeout: { return parser->ReadTime(res); }

      case R::kPositionKp: { return parser->ReadTorque(res); }
      case R::kPositionKi: { return parser->ReadTorque(res); }
      case R::kPositionKd: { return parser->ReadTorque(res); }
      case R::kPositionFeedforward: { return parser->ReadTorque(res); }
      case R::kPositionCommand: { return parser->ReadTorque(res); }

      case R::kControlPosition: { return parser->ReadPosition(res); }
      case R::kControlVelocity: { return parser->ReadVelocity(res); }
      case R::kControlTorque: { return parser->ReadTorque(res); }
      case R::kControlPositionError: { return parser->ReadPosition(res); }
      case R::kControlVelocityError: { return parser->ReadVelocity(res); }
      case R::kControlTorqueError: { return parser->ReadTorque(res); }

      case R::kCommandStayWithinLowerBound: { return parser->ReadPosition(res); }
      case R::kCommandStayWithinUpperBound: { return parser->ReadPosition(res); }
      case R::kCommandStayWithinFeedforwardTorque: { return parser->ReadTorque(res); }
      case R::kCommandStayWithinKpScale: { return parser->ReadPwm(res); }
      case R::kCommandStayWithinKdScale: { return parser->ReadPwm(res); }
      case R::kCommandStayWithinPositionMaxTorque: { return parser->ReadTorque(res); }
      case R::kCommandStayWithinTimeout: { return parser->ReadTime(res); }

      case R::kEncoder0Position: { return parser->ReadPosition(res); }
      case R::kEncoder0Velocity: { return parser->ReadVelocity(res); }
      case R::kEncoder1Position: { return parser->ReadPosition(res); }
      case R::kEncoder1Velocity: { return parser->ReadVelocity(res); }
      case R::kEncoder2Position: { return parser->ReadPosition(res); }
      case R::kEncoder2Velocity: { return parser->ReadVelocity(res); }

      case R::kEncoderValidity: { return parser->ReadInt(res); }

      case R::kAux1GpioCommand: { return parser->ReadInt(res); }
      case R::kAux2GpioCommand: { return parser->ReadInt(res); }
      case R::kAux1GpioStatus: { return parser->ReadInt(res); }
      case R::kAux2GpioStatus: { return parser->ReadInt(res); }

      case R::kAux1AnalogIn1: { return parser->ReadPwm(res); }
      case R::kAux1AnalogIn2: { return parser->ReadPwm(res); }
      case R::kAux1AnalogIn3: { return parser->ReadPwm(res); }
      case R::kAux1AnalogIn4: { return parser->ReadPwm(res); }
      case R::kAux1AnalogIn5: { return parser->ReadPwm(res); }

      case R::kAux2AnalogIn1: { return parser->ReadPwm(res); }
      case R::kAux2AnalogIn2: { return parser->ReadPwm(res); }
      case R::kAux2AnalogIn3: { return parser->ReadPwm(res); }
      case R::kAux2AnalogIn4: { return parser->ReadPwm(res); }
      case R::kAux2AnalogIn5: { return parser->ReadPwm(res); }

      case R::kMillisecondCounter: { return parser->ReadInt(res); }
      case R::kClockTrim: { return parser->ReadInt(res); }

      case R::kRegisterMapVersion: { return parser->ReadInt(res); }
      case R::kSerialNumber1: { return parser->ReadInt(res); }
      case R::kSerialNumber2: { return parser->ReadInt(res); }
      case R::kSerialNumber3: { return parser->ReadInt(res); }

      case R::kSetOutputNearest: { return parser->ReadInt(res); }
      case R::kSetOutputExact: { return parser->ReadInt(res); }
      case R::kRequireReindex: { return parser->ReadInt(res); }

      case R::kDriverFault1: { return parser->ReadInt(res); }
      case R::kDriverFault2: { return parser->ReadInt(res); }
    }

    return parser->ReadInt(res);
  }
};

struct GenericQuery {
  struct Command {};

  struct ItemValue {
    int16_t register_number = -1;
    double value = 0.0;
  };

  // A CAN-FD frame can only have 64 bytes, so we definitely can't
  // have more than 64 items in a single one ever.
  static constexpr int16_t kMaxItems = 64;

  struct Result {
    ItemValue values[kMaxItems] = {};
  };

  struct ItemFormat {
    int16_t register_number = detail::numeric_limits<int16_t>::max();
    Resolution resolution = kIgnore;
  };

  struct Format {
    ItemFormat values[kMaxItems] = {};
  };

  static int ItemFormatSort(const void* lhs_in, const void* rhs_in) {
    const auto* lhs = reinterpret_cast<const ItemFormat*>(lhs_in);
    const auto* rhs = reinterpret_cast<const ItemFormat*>(rhs_in);

    return lhs->register_number - rhs->register_number;
  }

  static void Make(WriteCanData* frame, const Command&, const Format& format) {
    ItemFormat sorted_values[64] = {};
    ::memcpy(&sorted_values[0], &(format.values[0]), sizeof(format.values));
    ::qsort(&sorted_values[0], kMaxItems, sizeof(ItemFormat), ItemFormatSort);

    const int16_t size = [&]() {
      for (int16_t i = 0; i < kMaxItems; i++) {
        if (sorted_values[i].register_number ==
            detail::numeric_limits<int16_t>::max()) {
          return i;
        }
      }
      return kMaxItems;
    }();

    if (size == 0) { return; }
    const int16_t min_register_number = sorted_values[0].register_number;
    const int16_t max_register_number = sorted_values[size - 1].register_number;

    const uint16_t required_registers = max_register_number - min_register_number + 1;

    // An arbitrary limit to constrain the amount of stack we use
    // below.
    if (required_registers > 512) { ::abort(); }

    Resolution resolutions[required_registers];
    ::memset(&resolutions[0], 0, sizeof(Resolution) * required_registers);

    for (int16_t this_register = min_register_number, index = 0;
         this_register <= max_register_number;
         this_register++) {
      if (sorted_values[index].register_number == this_register) {
        resolutions[this_register - min_register_number] =
            sorted_values[index].resolution;
        index++;
      } else {
        resolutions[this_register - min_register_number] = kIgnore;
      }
    }
    WriteCombiner combiner(
        frame, 0x10, min_register_number,
          resolutions, required_registers);
    for (uint16_t i = 0; i < required_registers; i++) {
      combiner.MaybeWrite();
    }
  }

  static Result Parse(const uint8_t* data, uint8_t size) {
    MultiplexParser parser(data, size);

    return Parse(&parser);
  }

  static Result Parse(const CanData* frame) {
    MultiplexParser parser(frame);

    return Parse(&parser);
  }

  static Result Parse(MultiplexParser* parser) {
    Result result;

    int16_t index = 0;

    while (true) {
      if (index >= kMaxItems) { return result; }

      const auto current = parser->next();
      if (current.done) { return result; }

      result.values[index].register_number = current.value;
      result.values[index].value =
          Query::ParseGeneric(parser, current.value, current.resolution);

      index++;
    }
  }
};

struct PositionMode {
  struct Command {
    double position = 0.0;
    double velocity = 0.0;
    double feedforward_torque = 0.0;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
    double maximum_torque = NaN;
    double stop_position = NaN;
    double watchdog_timeout = NaN;
    double velocity_limit = NaN;
    double accel_limit = NaN;
    double fixed_voltage_override = NaN;
  };

  struct Format {
    Resolution position = kFloat;
    Resolution velocity = kFloat;
    Resolution feedforward_torque = kIgnore;
    Resolution kp_scale = kIgnore;
    Resolution kd_scale = kIgnore;
    Resolution maximum_torque = kIgnore;
    Resolution stop_position = kIgnore;
    Resolution watchdog_timeout = kIgnore;
    Resolution velocity_limit = kIgnore;
    Resolution accel_limit = kIgnore;
    Resolution fixed_voltage_override = kIgnore;
  };

  static void Make(WriteCanData* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kPosition);

    // Now we use some heuristics to try and group consecutive registers
    // of the same resolution together into larger writes.
    const Resolution kResolutions[] = {
      format.position,
      format.velocity,
      format.feedforward_torque,
      format.kp_scale,
      format.kd_scale,
      format.maximum_torque,
      format.stop_position,
      format.watchdog_timeout,
      format.velocity_limit,
      format.accel_limit,
      format.fixed_voltage_override,
    };
    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandPosition,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.position, format.position);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVelocity(command.velocity, format.velocity);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.feedforward_torque, format.feedforward_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kp_scale, format.kp_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kd_scale, format.kd_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.maximum_torque, format.maximum_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.stop_position, format.stop_position);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTime(command.watchdog_timeout, format.watchdog_timeout);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVelocity(command.velocity_limit, format.velocity_limit);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteAccel(command.accel_limit, format.accel_limit);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVoltage(command.fixed_voltage_override,
                          format.fixed_voltage_override);
    }
  }
};

/// The voltage-mode FOC command.
struct VFOCMode {
  struct Command {
    double theta_rad = 0.0;
    double voltage = 0.0;
    double theta_rad_rate = 0.0;
  };

  struct Format {
    Resolution theta_rad = kFloat;
    Resolution voltage = kFloat;
    Resolution theta_rad_rate = kFloat;
  };

  static void Make(WriteCanData* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kVoltageFoc);

    const Resolution kResolutions[] = {
      format.theta_rad,
      format.voltage,
      kIgnore,
      kIgnore,
      kIgnore,
      kIgnore,
      format.theta_rad_rate,
    };
    WriteCombiner combiner(
        frame, 0x00,
        Register::kVFocTheta,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.theta_rad / M_PI, format.theta_rad);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVoltage(command.voltage, format.voltage);
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVelocity(command.theta_rad_rate / M_PI, format.theta_rad_rate);
    }
  }
};

/// DQ phase current command.
struct CurrentMode {
  struct Command {
    double d_A = 0.0;
    double q_A = 0.0;
  };

  struct Format {
    Resolution d_A = kFloat;
    Resolution q_A = kFloat;
  };

  static void Make(WriteCanData* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kCurrent);

    const Resolution kResolutions[] = {
      format.d_A,
      format.q_A,
    };

    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandQCurrent,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WriteCurrent(command.q_A, format.q_A);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteCurrent(command.d_A, format.d_A);
    }
  }
};

struct StayWithinMode {
  struct Command {
    double lower_bound = NaN;
    double upper_bound = NaN;
    double feedforward_torque = 0.0;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
    double maximum_torque = 0.0;
    double watchdog_timeout = NaN;
  };

  struct Format {
    Resolution lower_bound = kFloat;
    Resolution upper_bound = kFloat;
    Resolution feedforward_torque = kIgnore;
    Resolution kp_scale = kIgnore;
    Resolution kd_scale = kIgnore;
    Resolution maximum_torque = kIgnore;
    Resolution watchdog_timeout = kIgnore;
  };

  static void Make(WriteCanData* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kStayWithin);

    const Resolution kResolutions[] = {
      format.lower_bound,
      format.upper_bound,
      format.feedforward_torque,
      format.kp_scale,
      format.kd_scale,
      format.maximum_torque,
      format.watchdog_timeout,
    };

    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandStayWithinLowerBound,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.lower_bound, format.lower_bound);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.upper_bound, format.upper_bound);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.feedforward_torque,
                         format.feedforward_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kp_scale, format.kp_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kd_scale, format.kd_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.maximum_torque, format.maximum_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTime(command.watchdog_timeout, format.watchdog_timeout);
    }
  }
};

struct BrakeMode {
  struct Command {};
  struct Format {};

  static void Make(WriteCanData* frame,
                   const Command&,
                   const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kBrake);
  }
};

struct StopMode {
  struct Command {};
  struct Format {};

  static void Make(WriteCanData* frame,
                   const Command&,
                   const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kStopped);
  }
};

struct GpioWrite {
  struct Command {
    int8_t aux1 = 0;
    int8_t aux2 = 0;
  };

  struct Format {
    Resolution aux1 = kInt8;
    Resolution aux2 = kInt8;
  };

  static void Make(WriteCanData* frame,
                   const Command& command,
                   const Format& format) {
    const Resolution kResolutions[] = {
      format.aux1,
      format.aux2,
    };

    WriteCombiner combiner(
        frame, 0x00,
        Register::kAux1GpioCommand,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WriteInt(command.aux1, format.aux1);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteInt(command.aux2, format.aux2);
    }
  }
};

struct OutputNearest {
  struct Command {
    double position = 0.0;
  };

  struct Format {};

  static void Make(WriteCanData* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteFloat | 0x01);
    frame->WriteVaruint(Register::kSetOutputNearest);
    frame->Write<float>(command.position);
  }
};

struct OutputExact {
  struct Command {
    double position = 0.0;
  };

  struct Format {};

  static void Make(WriteCanData* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteFloat | 0x01);
    frame->WriteVaruint(Register::kSetOutputExact);
    frame->Write<float>(command.position);
  }
};

struct RequireReindex {
  struct Command {};
  struct Format {};

  static void Make(WriteCanData* frame, const Command&, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->WriteVaruint(Register::kRequireReindex);
    frame->Write<int8_t>(1);
  }
};

struct DiagnosticWrite {
  struct Command {
    int8_t channel = 1;
    const char* data = nullptr;
    int8_t size = 0;
  };

  struct Format {};

  static void Make(WriteCanData* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kClientToServer);
    frame->Write<int8_t>(command.channel);
    frame->Write<int8_t>(command.size);
    frame->Write(command.data, command.size);
  }
};

struct DiagnosticRead {
  struct Command {
    int8_t channel = 1;
    int8_t max_length = 48;
  };

  struct Format {};

  static void Make(WriteCanData* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kClientPollServer);
    frame->Write<int8_t>(command.channel);
    frame->Write<int8_t>(command.max_length);
  }
};

struct DiagnosticResponse {
  struct Result {
    int8_t channel = 1;
    uint8_t data[64] = {};
    int8_t size = 0;
  };

  static Result Parse(const uint8_t* data, uint8_t size) {
    MultiplexParser parser(data, size);
    return Parse(&parser);
  }

  static Result Parse(MultiplexParser* parser) {
    Result result;
    result.channel = -1;

    if (parser->remaining() < 3) { return result; }

    const auto action = parser->Read<int8_t>();
    if (action != Multiplex::kServerToClient) { return result; }
    const auto channel = parser->Read<int8_t>();

    const uint16_t size = parser->ReadVaruint();
    if (parser->remaining() < size) { return result; }

    result.channel = channel;
    result.size = size;
    parser->ReadRaw(result.data, size);

    return result;
  }
};

struct ClockTrim {
  struct Command {
    int32_t trim = 0;
  };

  struct Format {};

  static void Make(WriteCanData* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt32 | 0x01);
    frame->WriteVaruint(Register::kClockTrim);
    frame->Write<int32_t>(command.trim);
  }
};

}
}
