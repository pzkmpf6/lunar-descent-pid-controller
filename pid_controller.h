#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:
    PIDController(double kp, double ki, double kd,
                  double outMin, double outMax);

    double compute(double setpoint, double measured, double dt);

    void reset();

    double getLastError() const;
    double getLastP() const;
    double getLastI() const;
    double getLastD() const;

private:
    double kp_;
    double ki_;
    double kd_;
    double outMin_;
    double outMax_;
    double integralSum_;
    double prevError_;
    bool firstTick_;
    double lastP_, lastI_, lastD_, lastError_;
};

#endif