#pragma once
// Pure bounded diagnostic trajectory and safety rules, no SDK or devices.
#include "motor_feedback.hpp"
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <optional>
#include <vector>

namespace vlai_l1 {
constexpr double kDegrees = 180.0 / 3.14159265358979323846;
struct AutoMotor {
    FeedbackSpec feedback;
    int motor_type = 0;
    unsigned send_id = 0;
    double kp = 0, kd = 0, min_deg = 0, max_deg = 0;
    double tracking_deg = 0, speed_deg_s = 0, torque_nm = 0, excursion_deg = 0;
};
struct ManualHandling {
    std::array<double,8> hard_speed_deg_s{};
    std::uint64_t speed_persistence_ns=0, settle_ns=0, settle_timeout_ns=0;
    double settle_speed_deg_s=0, settle_span_deg=0;
};
struct AutoSpec {
    std::vector<std::array<double,8>> waypoints; // V4: named-order offsets, closed path, fixed gripper.
    std::optional<ManualHandling> handling;
    std::array<AutoMotor, 8> motors{};
    double amplitude_deg = 0, transition_s = 0, hold_s = 0, hold_test_s = 0;
    double maximum_amplitude_deg = 0, maximum_reference_speed = 0, maximum_reference_acceleration = 0;
    unsigned cycles = 0, rate_hz = 0, capacity = 0;
    unsigned enabled_status = 0, disabled_status = 0, temperature = 0;
    std::uint64_t state_timeout_ns = 0, tracking_timeout_ns = 0, shutdown_timeout_ns = 0;
    unsigned health_poll_ms = 0;
    unsigned active_motor = 8; // Required in V3 input; invalid until explicitly supplied.
    int priority = 0;
    void validate() const;
    double duration(bool hold_only) const {
        return hold_only ? hold_test_s : hold_s + cycles * legs() * (transition_s + hold_s);
    }
    std::size_t legs() const { return waypoints.empty() ? 4 : waypoints.size()-1; }
    double offset(double seconds, bool hold_only) const;
    std::array<double,8> target(const std::array<double,8>& reference, double seconds, bool hold_only) const {
        auto result=reference;
        if(waypoints.empty()) result.at(active_motor)+=offset(seconds,hold_only);
        else {
            if(!std::isfinite(seconds) || seconds<0) throw std::invalid_argument("invalid trajectory time");
            if(hold_only || seconds<=hold_s || seconds>=duration(false)) return result;
            const double t=seconds-hold_s;
            const auto segment=static_cast<std::size_t>(t/(transition_s+hold_s));
            const auto leg=segment%legs();
            const double u=std::min(1.0,(t-segment*(transition_s+hold_s))/transition_s);
            const double blend=u*u*u*(10-15*u+6*u*u);
            for(std::size_t i=0;i<8;++i)
                result[i]+=waypoints[leg][i]+(waypoints[leg+1][i]-waypoints[leg][i])*blend;
        }
        return result;
    }
};
inline void AutoSpec::validate() const {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    for (double v : {amplitude_deg, transition_s, hold_s, hold_test_s, maximum_amplitude_deg,
                     maximum_reference_speed, maximum_reference_acceleration}) {
        if (!positive(v)) throw std::invalid_argument("invalid automatic scalar");
    }
    if ((waypoints.empty() ? (active_motor!=3 && active_motor!=5 && active_motor!=6) :
        (active_motor!=8 || waypoints.size()<3 || waypoints.size()>64 || handling.has_value())) ||
        !cycles || !rate_hz || capacity < 8 || capacity > 2'000'000 || !state_timeout_ns || !tracking_timeout_ns ||
        !shutdown_timeout_ns || !health_poll_ms || priority < 1 || priority > 99 ||
        enabled_status > 15 || disabled_status > 15 || enabled_status == disabled_status ||
        !temperature || temperature > 255 || amplitude_deg > maximum_amplitude_deg ||
        1.875 * amplitude_deg / transition_s > maximum_reference_speed ||
        (10 * std::sqrt(3.0) / 3) * amplitude_deg / (transition_s * transition_s) > maximum_reference_acceleration ||
        1e9 / rate_hz >= state_timeout_ns || health_poll_ms * 1'000'000ULL > state_timeout_ns ||
        !std::isfinite(duration(false)) || std::max(duration(false),duration(true)) * rate_hz * 8 + rate_hz * 8 * 10 > capacity) {
        throw std::invalid_argument("invalid automatic timing, capacity or trajectory budget");
    }
    const std::array<std::string,8> names{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7","gripper"};
    for (std::size_t i=0;i<8;++i) {
        const auto& m=motors[i];
        if (m.feedback.name != names[i] || m.send_id < 1 || m.send_id > 15 ||
            m.feedback.slave_id != m.send_id || m.feedback.response_id > 0x7ff ||
            m.motor_type < 0 || m.motor_type > 9 || !positive(m.feedback.position_max_rad) ||
            !positive(m.feedback.velocity_max_rad_s) || !positive(m.feedback.torque_max_nm) ||
            !positive(m.kp) || m.kp > 500 || !positive(m.kd) || m.kd > 5 ||
            !std::isfinite(m.min_deg) || !std::isfinite(m.max_deg) || m.min_deg >= m.max_deg ||
            !positive(m.tracking_deg) || !positive(m.speed_deg_s) || !positive(m.torque_nm) ||
            !positive(m.excursion_deg) || m.torque_nm > m.feedback.torque_max_nm ||
            std::max(std::abs(m.min_deg),std::abs(m.max_deg)) > m.feedback.position_max_rad*kDegrees) {
            throw std::invalid_argument("invalid automatic motor " + names[i]);
        }
        for (std::size_t j=0;j<i;++j) if (m.send_id==motors[j].send_id || m.feedback.response_id==motors[j].feedback.response_id)
            throw std::invalid_argument("duplicate motor ID");
    }
    if(waypoints.empty()) {
        if (amplitude_deg > motors[active_motor].excursion_deg) throw std::invalid_argument("trajectory exceeds excursion");
    } else {
        std::array<bool,8> moves{};
        for(std::size_t p=0;p<waypoints.size();++p) for(std::size_t i=0;i<8;++i) {
            const double v=waypoints[p][i];
            if(!std::isfinite(v) || std::abs(v)>amplitude_deg || std::abs(v)>motors[i].excursion_deg ||
               ((p==0 || p+1==waypoints.size() || i==7) && v!=0))
                throw std::invalid_argument("invalid waypoint envelope/closure/gripper");
            moves[i]=moves[i] || v!=0;
            if(p) {
                const double distance=std::abs(v-waypoints[p-1][i]);
                if(1.875*distance/transition_s>maximum_reference_speed ||
                   10*std::sqrt(3.0)/3*distance/(transition_s*transition_s)>maximum_reference_acceleration)
                    throw std::invalid_argument("waypoint segment speed/acceleration budget");
            }
        }
        unsigned moving=0; for(bool m:moves) moving+=m;
        if(moving<2) throw std::invalid_argument("waypoint path requires multiple joints");
    }
    if(handling) {
        const auto& h=*handling;
        if(!h.speed_persistence_ns || h.speed_persistence_ns>tracking_timeout_ns ||
           h.settle_ns<=state_timeout_ns || h.settle_timeout_ns<=h.settle_ns ||
           !positive(h.settle_speed_deg_s) || !positive(h.settle_span_deg))
            throw std::invalid_argument("invalid manual handling timing/settling");
        for(std::size_t i=0;i<8;++i) {
            const auto& m=motors[i];
            if(!positive(h.hard_speed_deg_s[i]) || h.hard_speed_deg_s[i]<m.speed_deg_s ||
               h.hard_speed_deg_s[i]>m.feedback.velocity_max_rad_s*kDegrees ||
               h.settle_speed_deg_s>=m.speed_deg_s || h.settle_span_deg>=m.tracking_deg ||
               h.settle_span_deg<=4*m.feedback.position_max_rad*kDegrees/65535)
                throw std::invalid_argument("invalid manual handling envelope");
        }
    }
}
inline double AutoSpec::offset(double seconds, bool hold_only) const {
    if (!std::isfinite(seconds) || seconds < 0) throw std::invalid_argument("invalid trajectory time");
    if (hold_only || seconds <= hold_s || seconds >= duration(false)) return 0;
    const double t = seconds - hold_s;
    const unsigned segment = static_cast<unsigned>(t / (transition_s + hold_s));
    const double local = t - segment * (transition_s + hold_s);
    const std::array<double,4> starts{0,amplitude_deg,0,-amplitude_deg};
    const std::array<double,4> ends{amplitude_deg,0,-amplitude_deg,0};
    const unsigned leg=segment%4;
    const double u=std::min(1.0,local/transition_s);
    return starts[leg]+(ends[leg]-starts[leg])*u*u*u*(10-15*u+6*u*u);
}
// One reference and one enabled owner across unloaded/loaded/unloaded sweeps.
// Explicit operator acknowledgements advance only stationary waiting phases.
class PairedSweep {
public:
    explicit PairedSweep(const AutoSpec& s):duration_(s.duration(false)),cap_(s.hold_test_s),
        settle_timeout_(s.handling ? s.handling->settle_timeout_ns/1e9 : 0),
        settle_minimum_(s.handling ? s.handling->settle_ns/1e9 : 0) {
        if(!s.waypoints.empty()) throw std::invalid_argument("waypoint path excludes paired handling");
        if(cap_<=3*duration_+2*settle_timeout_) throw std::invalid_argument("paired duration budget too short");
    }
    void step(double elapsed, bool advance, bool settled=false) {
        if(!std::isfinite(elapsed) || elapsed<last_ || phase_==5) throw std::runtime_error("invalid paired state/time");
        last_=elapsed;
        if(elapsed>=cap_) throw std::runtime_error("paired session time cap");
        if(advance) {
            if(!waiting() || settling_) throw std::runtime_error("unexpected paired advance");
            if(elapsed+(phase_==1 ? 2 : 1)*(duration_+settle_timeout_)>=cap_) throw std::runtime_error("insufficient paired remaining time");
            if(settle_timeout_>0) settling_=true;
            else ++phase_;
            start_=elapsed;
        } else if(settling_) {
            if(elapsed-start_>=settle_timeout_) throw std::runtime_error("manual settling timeout");
            if(settled) {
                if(elapsed-start_<settle_minimum_) throw std::runtime_error("premature settling completion");
                settling_=false; ++phase_; start_=elapsed;
            }
        } else if(!waiting() && elapsed-start_>=duration_) {
            ++phase_; start_=elapsed;
        }
    }
    unsigned phase() const { return phase_; }
    bool waiting() const { return phase_==1 || phase_==3; }
    bool settling() const { return settling_; }
    bool complete() const { return phase_==5; }
    double local(double elapsed) const { return elapsed-start_; }
    const char* name() const {
        if(settling_) return phase_==1 ? "settling_loaded" : "settling_empty_after";
        const std::array<const char*,6> names{"baseline","awaiting_attach","loaded","awaiting_remove","empty_after","complete"};
        return names.at(phase_);
    }
private:
    double duration_,cap_,settle_timeout_,settle_minimum_,start_=0,last_=0;
    bool settling_=false;
    unsigned phase_=0;
};
// Starts AFTER hands-clear acknowledgement. Any renewed motion resets the dwell.
class SettlingGate {
public:
    explicit SettlingGate(ManualHandling h):h_(h) {}
    void reset() { since_=0; last_=0; }
    bool update(std::uint64_t now, const std::array<DecodedFeedback,8>& states) {
        if(!now || now<=last_) throw std::runtime_error("nonmonotonic settling time");
        last_=now;
        std::array<double,8> q{};
        for(std::size_t i=0;i<8;++i) {
            q[i]=states[i].position_rad*kDegrees;
            const auto v=states[i].velocity_rad_s*kDegrees;
            if(!std::isfinite(q[i]) || !std::isfinite(v)) throw std::runtime_error("nonfinite settling state");
            if(std::abs(v)>h_.settle_speed_deg_s) { since_=0; return false; }
        }
        if(!since_) { since_=now; lo_=hi_=q; return false; }
        bool moved=false;
        for(std::size_t i=0;i<8;++i) {
            lo_[i]=std::min(lo_[i],q[i]); hi_[i]=std::max(hi_[i],q[i]);
            moved=moved || hi_[i]-lo_[i]>h_.settle_span_deg;
        }
        if(moved) { since_=now; lo_=hi_=q; return false; }
        return now-since_>=h_.settle_ns;
    }
private:
    ManualHandling h_;
    std::uint64_t since_=0,last_=0;
    std::array<double,8> lo_{},hi_{};
};
class AutoGuard {
public:
    explicit AutoGuard(AutoSpec spec):spec_(std::move(spec)) { spec_.validate(); }
    void set_reference(const std::array<double,8>& q) {
        if (ready_ || failed_) throw std::runtime_error("reference already set/faulted");
        reference_=q;
        for (std::size_t i=0;i<8;++i) {
            const auto& m=spec_.motors[i];
            if (!std::isfinite(q[i]) || q[i] < m.min_deg || q[i] > m.max_deg) fail("initial bounds:"+m.feedback.name);
        }
        if(spec_.waypoints.empty()) {
            const auto active=spec_.active_motor;
            if (q[active]-spec_.amplitude_deg < spec_.motors[active].min_deg || q[active]+spec_.amplitude_deg > spec_.motors[active].max_deg)
                fail("trajectory bounds:"+spec_.motors[active].feedback.name);
        } else {
            // Quintic blend is monotonic in [0,1]: endpoint bounds cover every segment.
            for(const auto& point:spec_.waypoints) for(std::size_t i=0;i<8;++i)
                if(q[i]+point[i]<spec_.motors[i].min_deg || q[i]+point[i]>spec_.motors[i].max_deg)
                    fail("trajectory bounds:"+spec_.motors[i].feedback.name);
        }
        ready_=true;
    }
    void check(std::uint64_t now, const std::array<DecodedFeedback,8>& states,
               const std::array<double,8>& target, bool enabled, bool handling=false) {
        if (failed_ || !ready_ || now<=last_now_) fail("invalid/non-monotonic guard state");
        last_now_=now;
        if(handling && (!enabled || !spec_.handling || target!=reference_)) fail("invalid manual handling context");
        for (std::size_t i=0;i<8;++i) {
            const auto& s=states[i]; const auto& m=spec_.motors[i]; const auto stamp=s.raw.host_receive_ns;
            if (!stamp || stamp>now || now-stamp>spec_.state_timeout_ns || stamp<last_source_[i]) fail("stale/regressed motor:"+m.feedback.name);
            last_source_[i]=stamp;
            const auto q=s.position_rad*kDegrees;
            if (!std::isfinite(q) || !std::isfinite(target[i]) || !std::isfinite(s.velocity_rad_s) || !std::isfinite(s.torque_nm) ||
                q<m.min_deg || q>m.max_deg || target[i]<m.min_deg || target[i]>m.max_deg ||
                std::abs(q-reference_[i])>m.excursion_deg || std::abs(target[i]-reference_[i])>m.excursion_deg ||
                std::abs(s.torque_nm)>m.torque_nm ||
                s.mos_temperature_raw>spec_.temperature || s.rotor_temperature_raw>spec_.temperature || s.at_protocol_endpoint)
                fail("position/effort/temperature guard:"+m.feedback.name);
            const auto speed=std::abs(s.velocity_rad_s*kDegrees);
            if(speed>(handling ? spec_.handling->hard_speed_deg_s[i] : m.speed_deg_s))
                fail("speed hard guard:"+m.feedback.name);
            if(handling && speed>m.speed_deg_s) {
                if(!speed_since_[i]) speed_since_[i]=now;
                if(now-speed_since_[i]>=spec_.handling->speed_persistence_ns)
                    fail("manual sustained speed guard:"+m.feedback.name);
            } else speed_since_[i]=0;
            if (enabled ? s.status_nibble!=spec_.enabled_status : (s.status_nibble!=spec_.disabled_status && s.status_nibble!=spec_.enabled_status))
                fail("motor status:"+m.feedback.name);
            if (std::abs(q-target[i])>m.tracking_deg) {
                if (!error_since_[i]) error_since_[i]=now;
                if (now-error_since_[i]>=spec_.tracking_timeout_ns) fail("tracking error:"+m.feedback.name);
            } else error_since_[i]=0;
        }
    }
private:
    [[noreturn]] void fail(const std::string& message) { failed_=true; throw std::runtime_error(message); }
    AutoSpec spec_; bool ready_=false,failed_=false;
    std::array<double,8> reference_{};
    std::array<std::uint64_t,8> last_source_{},error_since_{},speed_since_{};
    std::uint64_t last_now_=0;
};
}
