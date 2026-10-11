#include "motor_feedback.hpp"
#include "feedback_wire.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void rejects(const std::function<void()>& action) {
    try {
        action();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error("invalid input was accepted");
}

std::array<vlai_l1::FeedbackSpec, 8> specs() {
    std::array<vlai_l1::FeedbackSpec, 8> result;
    for (std::size_t i = 0; i < result.size(); ++i) {
        // Synthetic test settings, deliberately not motor-model defaults.
        result[i] = {i == 7 ? "gripper" : "joint_" + std::to_string(i + 1),
                     static_cast<std::uint32_t>(17 + i), static_cast<std::uint8_t>(1 + i),
                     2.0, 4.0, 8.0};
    }
    return result;
}

vlai_l1::FeedbackFrame frame(std::size_t index, std::uint64_t time) {
    return {static_cast<std::uint32_t>(17 + index),
            {static_cast<std::uint8_t>(1 + index), 0x80, 0x00, 0x80, 0x08, 0x00, 30, 31},
            8, time, true};
}

void fill(vlai_l1::FeedbackAccumulator& accumulator, std::uint64_t start) {
    for (std::size_t i = 0; i < 8; ++i) require(accumulator.ingest(frame(i, start + i)), "frame missing");
}

void decoder_tests() {
    const auto spec = specs()[0];
    auto input = frame(0, 10);
    auto decoded = vlai_l1::decode_feedback(spec, input);
    require(decoded.position_rad == static_cast<float>(2.0 / 65535) &&
            decoded.velocity_rad_s == static_cast<float>(4.0 / 4095) &&
            decoded.torque_nm == static_cast<float>(8.0 / 4095), "midpoint quantization");
    require(!decoded.at_protocol_endpoint, "false clipping flag");
    for (unsigned status = 0; status < 16; ++status) {
        input.payload[0] = static_cast<std::uint8_t>((status << 4) | 1);
        decoded = vlai_l1::decode_feedback(spec, input);
        require(decoded.status_nibble == status, "raw status must survive, even unknown/fault");
        require(decoded.mos_temperature_raw == 30 && decoded.rotor_temperature_raw == 31,
                "raw temperature lost");
    }
    for (bool maximum : {false, true}) {
        input.payload.fill(maximum ? 255 : 0);
        input.payload[0] = 1;
        decoded = vlai_l1::decode_feedback(spec, input);
        require(decoded.position_rad == (maximum ? 2.0f : -2.0f) &&
                decoded.velocity_rad_s == (maximum ? 4.0f : -4.0f) &&
                decoded.torque_nm == (maximum ? 8.0f : -8.0f) && decoded.at_protocol_endpoint,
                "integer endpoints or clipping flag incorrect");
    }
    for (auto mutate : std::array<std::function<void(vlai_l1::FeedbackFrame&)>, 6>{
             [](auto& f) { f.can_id |= 0x80000000; },
             [](auto& f) { f.can_id |= 0x40000000; },
             [](auto& f) { f.can_id |= 0x20000000; },
             [](auto& f) { f.payload_size = 7; },
             [](auto& f) { f.payload[0] = 2; },
             [](auto& f) { f.host_receive_ns = 0; }}) {
        input = frame(0, 10);
        mutate(input);
        rejects([&] { vlai_l1::decode_feedback(spec, input); });
    }
    // Recorded right J1, 2026-10-09 05:20:59 UTC. This is a regression fixture,
    // not a model lookup table. Source hash/provenance is in FORCE_ESTIMATION.md.
    const vlai_l1::FeedbackSpec recorded{"joint_1", 17, 1, 12.5, 45, 54};
    const vlai_l1::FeedbackFrame raw{17, {1, 0x80, 0x55, 0x7f, 0xf7, 0xff, 0x1a, 0x17}, 8, 10, true};
    decoded = vlai_l1::decode_feedback(recorded, raw);
    require(decoded.position_rad == 0.032616160809993744f &&
            decoded.velocity_rad_s == -0.010989011265337467f &&
            decoded.torque_nm == -0.013186813332140446f, "recorded C SDK values differ");
    input = raw;
    input.can_fd = false;
    require(vlai_l1::decode_feedback(recorded, input).torque_nm == decoded.torque_nm,
            "classic and FD payload decode must agree");
}

void accumulator_tests() {
    vlai_l1::FeedbackAccumulator acc(specs(), 100, 10);
    require(!acc.take(1), "empty cache must not become zero feedback");
    auto unrelated = frame(0, 2);
    unrelated.can_id = 100;
    require(!acc.ingest(unrelated), "unrelated standard ID accepted");
    for (std::size_t i = 0; i < 7; ++i) acc.ingest(frame(i, 10 + i));
    require(!acc.take(18), "missing motor accepted");
    acc.ingest(frame(7, 19));
    auto batch = acc.take(20);
    require(batch && batch->sequence == 1 && batch->names[7] == "gripper", "complete batch rejected");
    require(!acc.take(21), "cached batch republished");
    acc.ingest(frame(0, 22));
    require(!acc.take(23), "one fresh joint hides stale other joints");
    fill(acc, 30);
    batch = acc.take(38);
    require(batch && batch->sequence == 2, "second complete batch missing");
    fill(acc, 40);
    require(!acc.take(148), "old feedback accepted");
    fill(acc, 160);
    acc.ingest(frame(7, 180));
    require(!acc.take(181), "skewed feedback accepted");
    fill(acc, 190);
    require(acc.take(200).has_value(), "fresh data must replace old/skewed candidates");
    rejects([&] { acc.ingest(frame(0, 190)); });
    require(acc.faulted(), "timestamp fault not latched");
    rejects([&] { fill(acc, 300); });
    rejects([&] { acc.take(400); });

    for (auto mutate : std::array<std::function<void(vlai_l1::FeedbackFrame&)>, 3>{
             [](auto& f) { f.payload_size = 9; },
             [](auto& f) { f.payload[0] = 8; },
             [](auto& f) { f.can_id |= 0x20000000; }}) {
        vlai_l1::FeedbackAccumulator broken(specs(), 100, 10);
        auto input = frame(0, 10);
        mutate(input);
        rejects([&] { broken.ingest(input); });
        require(broken.faulted(), "malformed input fault not latched");
        rejects([&] { broken.take(20); });
    }
    vlai_l1::FeedbackAccumulator future(specs(), 100, 10);
    future.ingest(frame(0, 50));
    rejects([&] { future.take(49); });  // Even if other motors are missing.
    require(future.faulted(), "future timestamp fault not latched");
    vlai_l1::FeedbackAccumulator clock(specs(), 100, 10);
    clock.take(10);
    rejects([&] { clock.take(10); });
    vlai_l1::FeedbackAccumulator overflow(specs(), 100, 10);
    overflow.invalidate("receive queue overflow");
    rejects([&] { overflow.take(10); });

    vlai_l1::FeedbackAccumulator boundary(specs(), 100, 7);
    fill(boundary, 10);
    require(boundary.take(110).has_value(), "inclusive age/skew boundary rejected");
    rejects([&] { boundary.ingest(frame(0, 9)); });
    require(boundary.faulted(), "decreasing per-motor timestamp accepted");
}

void config_tests() {
    for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN(), 1e100, 1e-300}) {
        auto bad = specs();
        bad[0].torque_max_nm = value;
        rejects([&] { vlai_l1::FeedbackAccumulator unused(bad, 100, 10); });
    }
    for (auto mutate : std::array<std::function<void(std::array<vlai_l1::FeedbackSpec, 8>&)>, 5>{
             [](auto& s) { s[0].name = "joint_2"; },
             [](auto& s) { s[0].response_id = s[1].response_id; },
             [](auto& s) { s[0].slave_id = s[1].slave_id; },
             [](auto& s) { s[0].slave_id = 16; },
             [](auto& s) { s[0].response_id = 0x800; }}) {
        auto bad = specs();
        mutate(bad);
        rejects([&] { vlai_l1::FeedbackAccumulator unused(bad, 100, 10); });
    }
    rejects([] { vlai_l1::FeedbackAccumulator unused(specs(), 0, 10); });
    rejects([] { vlai_l1::FeedbackAccumulator unused(specs(), 10, 0); });
    rejects([] { vlai_l1::FeedbackAccumulator unused(specs(), 10, 11); });
}

std::array<std::uint8_t, vlai_l1::kFeedbackPacketSize> packet_fixture() {
    vlai_l1::FeedbackAccumulator accumulator(specs(), 100, 10);
    fill(accumulator, 1000);
    const auto batch = accumulator.take(1010);
    require(batch.has_value(), "fixture batch missing");
    vlai_l1::FeedbackIdentity identity{};
    identity.side = 1;
    identity.session_id.fill(0x11);
    identity.config_sha256.fill(0x22);
    return vlai_l1::encode_feedback_packet(identity, *batch, specs());
}

void wire_tests() {
    const auto fixture = packet_fixture();
    require(fixture.size() == 328 && fixture[0] == 'V' && fixture[3] == 'T' &&
            fixture[4] == 1 && fixture[5] == 0 && fixture[6] == 1 && fixture[7] == 0,
            "wire header layout mismatch");
    require(fixture[56] == 1 && fixture[64] == 0xf2 && fixture[65] == 3 &&
            fixture[72] == 17 && fixture[73] == 0 && fixture[74] == 1 &&
            fixture[76] == 0xe8 && fixture[77] == 3 && fixture[84] == 1,
            "wire little-endian fields mismatch");
    vlai_l1::FeedbackAccumulator accumulator(specs(), 100, 10);
    fill(accumulator, 1000);
    const auto batch = *accumulator.take(1010);
    vlai_l1::FeedbackIdentity identity{};
    identity.side = 1;
    identity.session_id.fill(0x11);
    identity.config_sha256.fill(0x22);
    for (auto mutate : std::array<std::function<void(vlai_l1::FeedbackBatch&)>, 6>{
             [](auto& b) { b.names[0] = "joint_2"; },
             [](auto& b) { b.motors[0].torque_nm = 99; },
             [](auto& b) { b.motors[0].status_nibble = 8; },
             [](auto& b) { b.motors[0].at_protocol_endpoint = true; },
             [](auto& b) { b.sequence = 0; },
             [](auto& b) { b.assembled_ns = 999; }}) {
        auto bad = batch;
        mutate(bad);
        rejects([&] { vlai_l1::encode_feedback_packet(identity, bad, specs()); });
    }
    for (auto mutate : std::array<std::function<void(vlai_l1::FeedbackIdentity&)>, 3>{
             [](auto& id) { id.side = 2; },
             [](auto& id) { id.session_id.fill(0); },
             [](auto& id) { id.config_sha256.fill(0); }}) {
        auto bad = identity;
        mutate(bad);
        rejects([&] { vlai_l1::encode_feedback_packet(bad, batch, specs()); });
    }
}

// Optional offline replay: each line supplies explicit spec, raw payload, and
// recorded SDK floats. No library, device, configuration default or CAN access.
void replay(const std::string& path, std::ostream* wire_output) {
    std::ifstream input(path);
    require(input.is_open(), "cannot open replay file");
    std::string line;
    std::size_t count = 0;
    std::array<vlai_l1::FeedbackSpec, 8> group_specs;
    vlai_l1::FeedbackBatch batch{};
    vlai_l1::FeedbackIdentity identity{};
    // Synthetic identity for this offline test tool, never a live session.
    identity.side = 1;
    identity.session_id.fill(0x11);
    identity.config_sha256.fill(0x22);
    while (std::getline(input, line)) {
        std::istringstream row(line);
        vlai_l1::FeedbackSpec spec;
        vlai_l1::FeedbackFrame raw{};
        unsigned slave;
        std::string hex;
        float position, velocity, torque;
        require(static_cast<bool>(row >> spec.name >> spec.response_id >> slave >>
                spec.position_max_rad >> spec.velocity_max_rad_s >> spec.torque_max_nm >>
                raw.host_receive_ns >> hex >> position >> velocity >> torque), "bad replay row");
        require(slave > 0 && slave <= 15 && hex.size() == 16, "bad replay identity/payload");
        std::string extra;
        require(!(row >> extra), "extra replay fields");
        spec.slave_id = static_cast<std::uint8_t>(slave);
        raw.can_id = spec.response_id;
        raw.payload_size = 8;
        raw.can_fd = true;
        for (std::size_t i = 0; i < 8; ++i) {
            const auto pair = hex.substr(2 * i, 2);
            require(pair.find_first_not_of("0123456789abcdef") == std::string::npos, "invalid hex");
            raw.payload[i] = static_cast<std::uint8_t>(std::stoul(pair, nullptr, 16));
        }
        const auto decoded = vlai_l1::decode_feedback(spec, raw);
        require(decoded.position_rad == position && decoded.velocity_rad_s == velocity &&
                decoded.torque_nm == torque, "native replay differs from recorded SDK floats");
        const auto index = count % 8;
        group_specs[index] = spec;
        batch.names[index] = spec.name;
        batch.motors[index] = decoded;
        batch.assembled_ns = std::max(batch.assembled_ns, raw.host_receive_ns);
        ++count;
        if (count % 8 == 0) {
            batch.sequence = count / 8;
            if (wire_output != nullptr) {
                const auto encoded = vlai_l1::encode_feedback_packet(identity, batch, group_specs);
                wire_output->write(reinterpret_cast<const char*>(encoded.data()), encoded.size());
                require(static_cast<bool>(*wire_output), "cannot write replay wire packet");
            }
            batch.assembled_ns = 0;
        }
    }
    require(count > 0 && count % 8 == 0, "replay must contain complete eight-motor groups");
    std::cout << "Exact SDK float32 agreement: " << count << " frames\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--replay") {
            replay(argv[2], nullptr);
        } else if (argc == 4 && std::string(argv[1]) == "--replay-wire") {
            std::ofstream output(argv[3], std::ios::binary);
            require(output.is_open(), "cannot open replay wire output");
            replay(argv[2], &output);
            output.close();
            require(static_cast<bool>(output), "cannot finish replay wire output");
        } else if (argc == 3 && std::string(argv[1]) == "--emit-packet") {
            const auto packet = packet_fixture();
            std::ofstream output(argv[2], std::ios::binary);
            require(output.is_open(), "cannot open fixture output");
            output.write(reinterpret_cast<const char*>(packet.data()), packet.size());
            output.close();
            require(static_cast<bool>(output), "cannot write fixture packet");
        } else if (argc == 1) {
            decoder_tests();
            accumulator_tests();
            config_tests();
            wire_tests();
            std::cout << "Pure motor feedback tests passed\n";
        } else {
            throw std::invalid_argument(
                "usage: motor_feedback_test [--replay FILE | --emit-packet FILE | --replay-wire IN OUT]");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
