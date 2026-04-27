#include "pid_controller.h"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd,
                             double outMin, double outMax)
    : kp_(kp), ki_(ki), kd_(kd),
      outMin_(outMin), outMax_(outMax),
      integralSum_(0.0), prevError_(0.0),
      firstTick_(true),
      lastP_(0.0), lastI_(0.0), lastD_(0.0), lastError_(0.0)
{
}

double PIDController::compute(double setpoint, double measured, double dt)
{
    lastError_ = setpoint - measured;
    lastP_ = kp_ * lastError_;
    integralSum_ += lastError_ * dt;

    double integralLimit = (outMax_ - outMin_) / (ki_ > 0.0 ? ki_ : 1.0);
    integralSum_ = std::min(std::max(integralSum_, -integralLimit), integralLimit);

    lastI_ = ki_ * integralSum_;
    lastD_ = 0.0;
    if (!firstTick_ && dt > 0.0)
    {
        double errorRate = (lastError_ - prevError_) / dt;
        lastD_ = kd_ * errorRate;
    }
    firstTick_ = false;
    prevError_ = lastError_;

    double output = lastP_ + lastI_ + lastD_;
    return std::min(std::max(output, outMin_), outMax_);
}

void PIDController::reset()
{
    integralSum_ = 0.0;
    prevError_ = 0.0;
    firstTick_ = true;
    lastP_ = lastI_ = lastD_ = lastError_ = 0.0;
}

double PIDController::getLastError() const { return lastError_; }
double PIDController::getLastP() const { return lastP_; }
double PIDController::getLastI() const { return lastI_; }
double PIDController::getLastD() const { return lastD_; }