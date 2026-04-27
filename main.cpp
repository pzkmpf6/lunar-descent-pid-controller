#include <iostream>
#include <iomanip>
#include <algorithm>
#include "lander.h"
#include "pid_controller.h"

const double TIME_STEP = 0.1;        // seconds per tick
const double TARGET_VELOCITY = -2.0; // m/s  (negative = downward)

const double LANDER_MASS = 500.0;
const double LUNAR_GRAVITY = 1.62;
const double HOVER_THRUST = LANDER_MASS * LUNAR_GRAVITY; // 810 N

int main()
{
    Lander lander(100.0, 0.0, 500.0);
    PIDController pid(300.0, 10.0, 80.0, -1000.0, 1000.0);

    double time = 0.0;

    std::cout << "=== Lunar Lander Simulation - Phase 3 ===" << "\n";
    std::cout << "PID target velocity: " << TARGET_VELOCITY << " m/s"
              << "  |  Feedforward (hover): " << HOVER_THRUST << " N\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left
              << std::setw(9) << "Time(s)"
              << std::setw(14) << "Alt(m)"
              << std::setw(15) << "Vel(m/s)"
              << std::setw(14) << "Thrust(N)"
              << std::setw(13) << "Error(m/s)"
              << std::setw(10) << "Fuel(kg)"
              << "\n";
    std::cout << std::string(75, '-') << "\n";

    while (!lander.hasLanded() && !lander.hasCrashed())
    {
        double vel = lander.getVelocity();
        double alt = lander.getAltitude();
        double correction = pid.compute(TARGET_VELOCITY, vel, TIME_STEP);
        double thrustCmd = std::min(std::max(HOVER_THRUST + correction, 0.0), 2000.0);

        std::cout << std::setw(9) << time
                  << std::setw(14) << alt
                  << std::setw(15) << vel
                  << std::setw(14) << thrustCmd
                  << std::setw(13) << pid.getLastError()
                  << std::setw(10) << lander.getFuel()
                  << "\n";

        lander.applyThrust(thrustCmd);
        lander.update(TIME_STEP);
        time += TIME_STEP;
    }

    std::cout << std::string(75, '-') << "\n";

    if (lander.hasLanded())
    {
        std::cout << ">> SAFE LANDING  T=" << time
                  << "s  |  Fuel left: " << lander.getFuel() << " kg\n";
    }
    else
    {
        std::cout << ">> CRASH  T=" << time
                  << "s  |  Impact: " << lander.getVelocity() << " m/s\n";
    }

    return 0;
}