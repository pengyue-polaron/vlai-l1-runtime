#include "motor_feedback.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vlai_l1 {
namespace {

constexpr std::array<const char*, kFeedbackMotorCount> kNames{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "gripper"};

void validate_spec(const FeedbackSpec& spec) {
    if (std::find(kNames.begin(), kNames.end(), spec.name) == kNames.end() ||
        spec.response_id > 0x7ff || spec.slave_id == 0 || spec.slave_id > 15) {
        throw std::invalid_argument("invalid feedback name/response ID/slave nibble");
    }
    for (double maximum : {spec.position_max_rad, spec.velocity_max_rad_s, spec.torque_max_nm}) {
        if (!std::isfinite(maximum) || maximum <= 0 ||
            maximum > static_cast<double>(std::numeric_limits<float>::max()) ||
            static_cast<float>(maximum) == 0.0f) {
            throw std::invalid_argument("feedback ranges must be positive finite float32 values");
        }
    }
}

float decode_integer(unsigned value, double maximum, unsigned integer_maximum) {
    // Evaluate in double, then round once to match the audited C SDK getters.
    return static_cast<float>(value * (2.0 * maximum) / integer_maximum - maximum);
}

}  // namespace

DecodedFeedback decode_feedback(const FeedbackSpec& spec, const FeedbackFrame& frame) {
    validate_spec(spec);
    if (frame.can_id != spec.response_id || frame.payload_size != 8 ||
        frame.host_receive_ns == 0 || (frame.payload[0] & 0x0f) != spec.slave_id) {
        throw std::invalid_argument("invalid feedback ID/flags/length/time/slave nibble for " + spec.name);
    }
    const auto& b = frame.payload;
    const unsigned position = (static_cast<unsigned>(b[1]) << 8) | b[2];
    const unsigned velocity = (static_cast<unsigned>(b[3]) << 4) | (b[4] >> 4);
    const unsigned torque = (static_cast<unsigned>(b[4] & 0x0f) << 8) | b[5];
    return {frame,
            decode_integer(position, spec.position_max_rad, 65535),
            decode_integer(velocity, spec.velocity_max_rad_s, 4095),
            decode_integer(torque, spec.torque_max_nm, 4095),
            static_cast<std::uint8_t>(b[0] >> 4), b[6], b[7],
            position == 0 || position == 65535 || velocity == 0 || velocity == 4095 ||
                torque == 0 || torque == 4095};
}

FeedbackAccumulator::FeedbackAccumulator(
    std::array<FeedbackSpec, kFeedbackMotorCount> specs,
    std::uint64_t max_age_ns, std::uint64_t max_skew_ns)
    : specs_(std::move(specs)), max_age_ns_(max_age_ns), max_skew_ns_(max_skew_ns) {
    if (max_age_ns == 0 || max_skew_ns == 0 || max_skew_ns > max_age_ns) {
        throw std::invalid_argument("feedback age/skew limits must be positive and skew <= age");
    }
    for (std::size_t i = 0; i < specs_.size(); ++i) {
        validate_spec(specs_[i]);
        if (specs_[i].name != kNames[i]) {
            throw std::invalid_argument("feedback requires exact ordered motor names");
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (specs_[i].response_id == specs_[j].response_id ||
                specs_[i].slave_id == specs_[j].slave_id) {
                throw std::invalid_argument("feedback requires distinct response and slave IDs");
            }
        }
    }
}

void FeedbackAccumulator::require_healthy() const {
    if (faulted()) {
        throw std::runtime_error("feedback session fault: " + fault_);
    }
}

void FeedbackAccumulator::invalidate(const std::string& reason) {
    if (!faulted()) {
        fault_ = reason.empty() ? "unspecified input fault" : reason;
    }
}

bool FeedbackAccumulator::ingest(const FeedbackFrame& frame) {
    require_healthy();
    // This core accepts standard data frames only, including 8-byte CAN-FD.
    // Reject rather than mask flags (especially SocketCAN error/RTR frames).
    if (frame.can_id > 0x7ff) {
        invalidate("non-standard data frame or CAN error flag");
        require_healthy();
    }
    for (std::size_t i = 0; i < specs_.size(); ++i) {
        if (frame.can_id != specs_[i].response_id) {
            continue;
        }
        try {
            const auto decoded = decode_feedback(specs_[i], frame);
            if (latest_[i] && frame.host_receive_ns <= latest_[i]->raw.host_receive_ns) {
                throw std::invalid_argument("non-increasing receive timestamp for " + specs_[i].name);
            }
            latest_[i] = decoded;
        } catch (const std::exception& error) {
            invalidate(error.what());
            throw;
        }
        return true;
    }
    return false;
}

std::optional<FeedbackBatch> FeedbackAccumulator::take(std::uint64_t now_ns) {
    require_healthy();
    if (now_ns == 0 || now_ns <= last_now_ns_) {
        invalidate("non-increasing assembly timestamp");
        require_healthy();
    }
    last_now_ns_ = now_ns;
    bool complete = true;
    std::uint64_t oldest = now_ns;
    std::uint64_t newest = 0;
    for (std::size_t i = 0; i < latest_.size(); ++i) {
        if (!latest_[i]) {
            complete = false;
            continue;
        }
        const auto timestamp = latest_[i]->raw.host_receive_ns;
        if (timestamp > now_ns) {
            invalidate("receive timestamp is in the future for " + specs_[i].name);
            require_healthy();
        }
        if (timestamp <= emitted_ns_[i] || now_ns - timestamp > max_age_ns_) {
            complete = false;
        }
        oldest = std::min(oldest, timestamp);
        newest = std::max(newest, timestamp);
    }
    if (!complete || newest - oldest > max_skew_ns_) {
        return std::nullopt;
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        invalidate("feedback sequence exhausted");
        require_healthy();
    }
    FeedbackBatch batch;
    batch.sequence = ++sequence_;
    batch.assembled_ns = now_ns;
    for (std::size_t i = 0; i < latest_.size(); ++i) {
        batch.names[i] = specs_[i].name;
        batch.motors[i] = *latest_[i];
        emitted_ns_[i] = latest_[i]->raw.host_receive_ns;
    }
    return batch;
}

}  // namespace vlai_l1
