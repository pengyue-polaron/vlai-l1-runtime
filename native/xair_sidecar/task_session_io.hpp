#pragma once
// Bounded pipe protocol shared by physical owner and hardware-free simulator.
#include "task_session_core.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <ctime>
#include <iomanip>
namespace vlai_l1 {
inline std::uint64_t task_now_ns() {
    timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t)) throw std::runtime_error("task clock failed");
    return static_cast<std::uint64_t>(t.tv_sec)*1000000000ULL+t.tv_nsec;
}
inline void nonblocking(int fd) {
    const auto flags=fcntl(fd,F_GETFL,0);
    if(flags<0 || fcntl(fd,F_SETFL,flags|O_NONBLOCK)<0) throw std::runtime_error("task nonblocking pipe setup");
}
inline void task_input(vlai_l1::TaskSession& task,std::string& pending) {
    // At most 4096 input bytes and eight complete commands per control tick.
    char bytes[4096];const auto count=::read(STDIN_FILENO,bytes,sizeof(bytes));
    if(count==0) task.close_input();
    if(count<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)
        throw std::runtime_error("task input read failure");
    if(count>0) pending.append(bytes,static_cast<std::size_t>(count));
    if(pending.size()>4096) throw std::runtime_error("task input backlog");
    unsigned commands=0;
    for(auto end=pending.find('\n');end!=std::string::npos;end=pending.find('\n')) {
        if(++commands>8) throw std::runtime_error("task command flood");
        const auto command=vlai_l1::parse_task_command(pending.substr(0,end));
        pending.erase(0,end+1);task.accept(command,task_now_ns());
    }
    if(pending.size()>1024) throw std::runtime_error("task unterminated input size");
}
inline void task_output(const vlai_l1::TaskSession& task,const std::array<vlai_l1::DecodedFeedback,8>& feedback,
                 const std::array<double,8>& target) {
    std::uint64_t earliest=feedback[0].raw.host_receive_ns;
    for(const auto& m:feedback) earliest=std::min(earliest,m.raw.host_receive_ns);
    std::ostringstream out;out<<std::setprecision(17)
        <<"{\"state\":\"task_state\",\"owner_pid\":"<<getpid()<<",\"monotonic_ns\":"<<task_now_ns()
        <<",\"earliest_motor_response_ns\":"<<earliest<<",\"accepted_sequence\":"<<task.accepted()
        <<",\"applied_sequence\":"<<task.applied()<<",\"completed_sequence\":"<<task.completed()
        <<",\"observation_ns\":"<<task.observation_ns()
        <<",\"policy_gap_ns\":"<<task.policy_gap_ns()<<",\"policy_gap_count\":"<<task.policy_gap_count()
        <<",\"pending_sequence\":"<<task.pending_sequence()<<",\"phase\":\""<<task.phase()<<"\",\"q_deg\":[";
    for(std::size_t i=0;i<8;++i) out<<(i?",":"")<<feedback[i].position_rad*vlai_l1::kDegrees;
    out<<"],\"reference_deg\":[";
    for(std::size_t i=0;i<8;++i) out<<(i?",":"")<<target[i];
    out<<"]}\n";const auto line=out.str();
    if(line.size()>4096 || ::write(STDOUT_FILENO,line.data(),line.size())!=static_cast<ssize_t>(line.size()))
        throw std::runtime_error("task output blocked/failed");
}
}
