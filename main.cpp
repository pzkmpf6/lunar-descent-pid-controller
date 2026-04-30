#include <iostream>
#include <iomanip>
#include <random>
#include <algorithm>
#include <chrono>
#include <thread>
#include <ctime>
#include "lander.h"
#include "pid_controller.h"
#include "telemetry.h"

// --- Simulation config ---
const double TIME_STEP = 0.1;
const double TARGET_VELOCITY = -2.0;                     // m/s
const double LANDER_MASS = 500.0;                        // kg
const double LUNAR_GRAVITY = 1.62;                       // m/s^2
const double HOVER_THRUST = LANDER_MASS * LUNAR_GRAVITY; // 810 N
const double MAX_THRUST = 2000.0;                        // N

// --- Wind model parameters ---
const double WIND_DECAY = 0.97;
const double WIND_SIGMA = 18.0; // N
const double WIND_MAX = 130.0;  // N

// --- Sensor noise parameters ---
const double NOISE_SIGMA = 0.08; // m/s

int main()
{
    Lander lander(100.0, 0.0, 500.0);
    PIDController pid(300.0, 10.0, 80.0, -1000.0, 1000.0);
    Telemetry telemetry(100.0, 500.0, MAX_THRUST);

    unsigned int seed = static_cast<unsigned int>(std::time(nullptr));
    std::mt19937 rng(seed);

    std::cout << "Simulation seed: " << seed
              << "  (use this number to replay this exact run)\n\n";

    std::normal_distribution<double> windStepDist(0.0, WIND_SIGMA);
    std::normal_distribution<double> noiseDist(0.0, NOISE_SIGMA);

    double time = 0.0;
    double windForce = 0.0;

    while (!lander.hasLanded() && !lander.hasCrashed())
    {

        windForce = windForce * WIND_DECAY + windStepDist(rng) * TIME_STEP;
        windForce = std::max(-WIND_MAX, std::min(WIND_MAX, windForce));

        double sensorNoise = noiseDist(rng);
        double noisyVelocity = lander.getVelocity() + sensorNoise;

        double correction = pid.compute(TARGET_VELOCITY, noisyVelocity, TIME_STEP);
        double thrustCmd = std::max(0.0, std::min(MAX_THRUST, HOVER_THRUST + correction));

        telemetry.render(lander, pid, time, thrustCmd, windForce, sensorNoise);

        lander.applyDisturbance(windForce);
        lander.applyThrust(thrustCmd);
        lander.update(TIME_STEP);

        time += TIME_STEP;

        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    std::cout << "\n";
    if (lander.hasLanded())
    {
        std::cout << "  >> MISSION SUCCESS  |  T=" << std::fixed << std::setprecision(2)
                  << time << " s  |  Fuel remaining: "
                  << lander.getFuel() << " kg\n\n";
    }
    else
    {
        std::cout << "  >> MISSION FAILURE  |  T=" << std::fixed << std::setprecision(2)
                  << time << " s  |  Impact velocity: "
                  << lander.getVelocity() << " m/s\n\n";
    }

    return 0;
}