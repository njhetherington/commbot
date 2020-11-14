#ifndef _PULSER_HPP_
#define _PULSER_HPP_

#include <thread>
#include <mutex>
#include <chrono>
#include <math.h>
#include <iostream>

const int GIGA = 1000000000; // 10^9

namespace commbot{

class Pulser{

    public:

        Pulser(const float_t &freq=0) noexcept(false){
            set_freq(freq);
            thread_timer_ = std::thread{&Pulser::timer, this}; // start timer thread
        }
        
        ~Pulser() noexcept(false){
            is_on_ = false; // stops thread
            if(thread_timer_.joinable()) thread_timer_.join();
            else throw std::runtime_error("~Pulser: thread not joinable.");
        }

        bool get_state(){
            std::lock_guard<std::mutex> guard_state(mutex_);
            return state_;
        }

        void set_freq(const float_t &freq) noexcept(false){
            std::lock_guard<std::mutex> guard_period(mutex_);
            if(freq < 0) throw std::invalid_argument("Pulser: tried to set illegal frequency of " + std::to_string(freq) + " Hz");
            else if(freq == 0){
                period_ = 9999; // unused
                state_ = false;
            } 
            else period_ = 1/freq;
        }

        void activate(const float_t &freq) {
            set_is_active(true);
            set_freq(freq);
        }

        void deactivate() {
            set_is_active(false);
            set_freq(0);
        }

    private:
        
        bool is_on_ = true; // controls timer thread
        bool is_active_ = false;
        bool state_ = false; // pulse state
        std::mutex mutex_; // protect fields used in timer thread
        float_t period_; // period in seconds
        std::thread thread_timer_;

        // Sets is_active_ using lock_guard
        void set_is_active(const bool &is_active){
            std::lock_guard<std::mutex> guard_is_active(mutex_);
            is_active_ = is_active;
        }

        // Changes state_ at freq_
        void timer() noexcept{
            std::chrono::steady_clock::time_point t_prev = std::chrono::steady_clock::now();
            while(is_on_){
                if(!is_active_) state_ = false;
                else{
                    if(std::chrono::steady_clock::now().time_since_epoch().count()
                        - t_prev.time_since_epoch().count() > period_*GIGA){
                        state_ = !state_; // change state
                        t_prev = std::chrono::steady_clock::now(); // track time of state change
                    }
                }
            }
        }



}; // end Pulser class

} // end commbot namespace

#endif
