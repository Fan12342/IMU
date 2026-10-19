//
// Created by Administrator on 2026/10/11.
//

#include "pid.h"
#include <complex>

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k):
    kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), out_max_(out_max), d_filter_k_(d_filter_k) {}

void PID::reset()
{
    err_sum_ = 0;
    last_err_ = 0;
}

float PID::calc(float ref, float fdb)
{
    float error = ref - fdb;

    float pout = kp_ * error;
    pout_ = pout;

    float iout = 0;
    if (std::abs(error) < i_max_)
    {
        err_sum_ += error;
        iout = ki_ * err_sum_;
    }
    else
    {
        err_sum_ = 0;
    }

    dout_ = (1 - d_filter_k_) * dout_ + d_filter_k_ * kd_ * (error - last_err_);

    last_err_ = error;

    float out = pout + iout + dout_;
    out = (out < out_max_) ? out : out_max_;
    out = (out > -out_max_) ? out : -out_max_;

    return out;
}