#pragma once

// Pure candidate telemetry: no sockets, SDK handles, threads or clock reads.
// This module is not connected to the live sidecar or a force-control path.
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace vlai_l1 {

constexpr std::size_t kFeedbackMotorCount = 8;

struct FeedbackSpec {
    std::string name;
    std::uint32_t response_id;
    std::uint8_t slave_id;
    double position_max_rad;
    double velocity_max_rad_s;
    double torque_max_nm;
};

struct FeedbackFrame {
    std::uint32_t can_id;  // Full SocketCAN ID including flags; never pre-mask it.
    std::array<std::uint8_t, 8> payload;
    std::size_t payload_size;
    std::uint64_t host_receive_ns;  // Dequeue time, NOT motor acquisition time.
    bool can_fd;
};

struct DecodedFeedback {
    FeedbackFrame raw;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    std::uint8_t status_nibble;  // Raw, deliberately no enabled/healthy inference.
    std::uint8_t mos_temperature_raw;
    std::uint8_t rotor_temperature_raw;
    bool at_protocol_endpoint;  // Any integer field is at min/max; not proof of clipping.
};

DecodedFeedback decode_feedback(const FeedbackSpec& spec, const FeedbackFrame& frame);

struct FeedbackBatch {
    std::uint64_t sequence;
    std::uint64_t assembled_ns;
    std::array<std::string, kFeedbackMotorCount> names;
    std::array<DecodedFeedback, kFeedbackMotorCount> motors;
};

// A single-thread-owned collection of *raw* candidates. Every motor must update
// since the preceding emitted batch. Missing, old or skewed data emits nothing.
// Invalid input latches a fault permanently; recovery requires a new session.
// The caller must separately validate status, firmware ranges and queue age
// before any online use. A batch is not a calibrated or safety-valid FT input.
class FeedbackAccumulator {
public:
    FeedbackAccumulator(std::array<FeedbackSpec, kFeedbackMotorCount> specs,
                        std::uint64_t max_age_ns, std::uint64_t max_skew_ns);
    bool ingest(const FeedbackFrame& frame);  // false = unrelated standard ID
    std::optional<FeedbackBatch> take(std::uint64_t now_ns);
    void invalidate(const std::string& reason);
    bool faulted() const noexcept { return !fault_.empty(); }

private:
    void require_healthy() const;
    std::array<FeedbackSpec, kFeedbackMotorCount> specs_;
    std::array<std::optional<DecodedFeedback>, kFeedbackMotorCount> latest_{};
    std::array<std::uint64_t, kFeedbackMotorCount> emitted_ns_{};
    std::uint64_t max_age_ns_;
    std::uint64_t max_skew_ns_;
    std::uint64_t last_now_ns_ = 0;
    std::uint64_t sequence_ = 0;
    std::string fault_;
};

}  // namespace vlai_l1
