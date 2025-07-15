// Copyright (C) Microsoft Corporation. 
// Copyright (C) IAMAI Consulting Corporation.  

// MIT License. All rights reserved.

#ifndef CORE_SIM_INCLUDE_CORE_SIM_MESSAGE_POSE_STAMPED_MESSAGE_HPP_
#define CORE_SIM_INCLUDE_CORE_SIM_MESSAGE_POSE_STAMPED_MESSAGE_HPP_

#include <memory>
#include <string>

#include "core_sim/clock.hpp"
#include "core_sim/math_utils.hpp"
#include "core_sim/message/message.hpp"

namespace microsoft {
namespace projectairsim {

class PoseStampedMessage : public Message {
 public:
  PoseStampedMessage(const TimeNano time_stamp, const Vector3 position,
              const Quaternion orientation);

  PoseStampedMessage();

  ~PoseStampedMessage() override;

  TimeNano GetTimeStamp() const;
  Vector3 GetPosition() const;
  Quaternion GetOrientation() const;

  std::string Serialize() const override;

  void Deserialize(const std::string& buffer) override;

 private:
  class Impl;
};

}  // namespace projectairsim
}  // namespace microsoft

#endif  // CORE_SIM_INCLUDE_CORE_SIM_MESSAGE_POSE_STAMPED_MESSAGE_HPP_
