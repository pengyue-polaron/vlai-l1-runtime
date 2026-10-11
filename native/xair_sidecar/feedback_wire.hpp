#pragma once

#include "motor_feedback.hpp"

namespace vlai_l1 {

// Independent raw Follower telemetry protocol VL1T/1. Not VL1S position state.
// No transport is opened here. Source/session provenance is supplied by caller.
constexpr std::size_t kFeedbackPacketSize = 328;

struct FeedbackIdentity {
    std::uint8_t side;  // 0 = left, 1 = right; role is always Follower.
    std::array<std::uint8_t, 16> session_id;
    std::array<std::uint8_t, 32> config_sha256;
};

std::array<std::uint8_t, kFeedbackPacketSize> encode_feedback_packet(
    const FeedbackIdentity& identity, const FeedbackBatch& batch,
    const std::array<FeedbackSpec, kFeedbackMotorCount>& specs);

}  // namespace vlai_l1
