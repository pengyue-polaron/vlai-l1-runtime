#include "feedback_wire.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace vlai_l1 {
namespace {

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "feedback protocol requires IEEE754 binary32");

class Writer {
public:
    void integer(std::uint64_t value, std::size_t count) {
        for (std::size_t i = 0; i < count; ++i) bytes.at(offset++) = (value >> (8 * i)) & 0xff;
    }
    template<std::size_t N> void append(const std::array<std::uint8_t, N>& values) {
        for (auto value : values) integer(value, 1);
    }
    void real(float value) {
        std::uint32_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        integer(bits, 4);
    }
    std::array<std::uint8_t, kFeedbackPacketSize> bytes{};
    std::size_t offset = 0;
};

template<std::size_t N> bool nonzero(const std::array<std::uint8_t, N>& bytes) {
    return std::any_of(bytes.begin(), bytes.end(), [](auto byte) { return byte != 0; });
}

}  // namespace

std::array<std::uint8_t, kFeedbackPacketSize> encode_feedback_packet(
    const FeedbackIdentity& identity, const FeedbackBatch& batch,
    const std::array<FeedbackSpec, kFeedbackMotorCount>& specs) {
    if (identity.side > 1 || !nonzero(identity.session_id) || !nonzero(identity.config_sha256) ||
        batch.sequence == 0 || batch.assembled_ns == 0) {
        throw std::invalid_argument("invalid feedback packet identity/sequence/time");
    }
    // Reuse full named-spec validation without any hardware or physical defaults.
    // Here the assembly time bounds all positive source times; actual age/skew
    // acceptance is the caller/consumer policy, not a serializer decision.
    FeedbackAccumulator validator(specs, batch.assembled_ns, batch.assembled_ns);
    Writer writer;
    writer.append(std::array<std::uint8_t, 4>{'V', 'L', '1', 'T'});
    writer.integer(1, 2);
    writer.integer(identity.side, 1);
    writer.integer(0, 1);
    writer.append(identity.session_id);
    writer.append(identity.config_sha256);
    writer.integer(batch.sequence, 8);
    writer.integer(batch.assembled_ns, 8);
    for (std::size_t i = 0; i < batch.motors.size(); ++i) {
        const auto& motor = batch.motors[i];
        const auto& raw = motor.raw;
        const auto decoded = decode_feedback(specs[i], raw);
        if (batch.names[i] != specs[i].name || raw.host_receive_ns > batch.assembled_ns ||
            motor.position_rad != decoded.position_rad ||
            motor.velocity_rad_s != decoded.velocity_rad_s || motor.torque_nm != decoded.torque_nm ||
            motor.status_nibble != decoded.status_nibble ||
            motor.mos_temperature_raw != decoded.mos_temperature_raw ||
            motor.rotor_temperature_raw != decoded.rotor_temperature_raw ||
            motor.at_protocol_endpoint != decoded.at_protocol_endpoint) {
            throw std::invalid_argument("inconsistent raw/decoded feedback for " + specs[i].name);
        }
        writer.integer(raw.can_id, 2);
        writer.integer((raw.can_fd ? 1 : 0) | (motor.at_protocol_endpoint ? 2 : 0), 1);
        writer.integer(0, 1);
        writer.integer(raw.host_receive_ns, 8);
        writer.append(raw.payload);
        writer.real(motor.position_rad);
        writer.real(motor.velocity_rad_s);
        writer.real(motor.torque_nm);
    }
    if (writer.offset != writer.bytes.size()) throw std::logic_error("feedback packet size mismatch");
    return writer.bytes;
}

}  // namespace vlai_l1
