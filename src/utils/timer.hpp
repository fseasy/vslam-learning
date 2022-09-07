#pragma once 

#include <chrono>
#include <iostream>
#include <string>

class AutoTimer {
public:
    AutoTimer(const std::string& name): 
        name_(name),
        start_(std::chrono::steady_clock::now()),
        duration_start_(start_) {}
    ~AutoTimer();
    double duration_ms(const std::string& info);
private:
    const std::string name_;
    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point duration_start_;
};

inline 
AutoTimer::~AutoTimer() {
    auto etp = std::chrono::steady_clock::now() - start_;
    auto ems = std::chrono::duration_cast<std::chrono::milliseconds>(etp).count();
    std::clog << "[" << name_ << "] elapsed " << ems << "ms\n";
}

inline 
double AutoTimer::duration_ms(const std::string& info) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_tp = now - duration_start_;
    auto elapsed_ms = std::chrono::duration_cast<
        std::chrono::milliseconds>(elapsed_tp).count();
    std::clog << "[" << name_ << "] [" << info 
        << "] duration elapsed " << elapsed_ms << "ms\n";
    duration_start_ = std::chrono::steady_clock::now();
    return elapsed_ms;
}