#include <iostream>
#include <iomanip>
#include "lander.h"

const double TIME_STEP = 0.1; // seconds

int main()
{
    // Create a lander: 100m altitude, 0 m/s starting velocity, 500kg fuel
    Lander lander(100.0, 0.0, 500.0);

    std::cout << "=== Lunar Lander Simulation - Phase 2 ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left
              << std::setw(10) << "Time(s)"
              << std::setw(16) << "Altitude(m)"
              << std::setw(16) << "Velocity(m/s)"
              << std::setw(14) << "Thrust(N)"
              << std::setw(12) << "Fuel(kg)"
              << std::endl;
    std::cout << std::string(68, '-') << std::endl;

    double time = 0.0;

    while (!lander.hasLanded() && !lander.hasCrashed())
    {
        std::cout << std::setw(10) << time
                  << std::setw(16) << lander.getAltitude()
                  << std::setw(16) << lander.getVelocity()
                  << std::setw(14) << lander.getThrust()
                  << std::setw(12) << lander.getFuel()
                  << std::endl;

        lander.applyThrust(900.0);
        lander.update(TIME_STEP);

        time += TIME_STEP;
    }

    std::cout << std::string(68, '-') << std::endl;

    if (lander.hasLanded())
    {
        std::cout << ">> SAFE LANDING at T=" << time << "s  |  "
                  << "Fuel remaining: " << lander.getFuel() << " kg" << std::endl;
    }
    else
    {
        std::cout << ">> CRASH at T=" << time << "s  |  "
                  << "Impact velocity: " << lander.getVelocity() << " m/s" << std::endl;
    }

    return 0;
}