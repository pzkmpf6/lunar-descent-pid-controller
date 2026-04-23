#include <iostream>
#include <iomanip>

// --- Simulation Constants ---
// Lunar gravity is about 1/6th of Earth's (9.81 / 6 ≈ 1.62 m/s²)
const double LUNAR_GRAVITY = 1.62; // m/s²
const double TIME_STEP = 0.1;      // seconds per simulation tick

int main()
{
    // --- Initial State  ---
    double altitude = 100.0; // meters above the surface
    double velocity = 0.0;   // m/s (positive = moving upward)
    double time = 0.0;       // elapsed simulation time in seconds

    std::cout << "=== Lunar Lander Simulation ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::left
              << std::setw(10) << "Time(s)"
              << std::setw(18) << "Altitude(m)"
              << std::setw(18) << "Velocity(m/s)"
              << std::endl;
    std::cout << std::string(46, '-') << std::endl;

    // --- Main simulation loop ---
    while (altitude > 0.0)
    {
        // Print current state
        std::cout << std::setw(10) << time
                  << std::setw(18) << altitude
                  << std::setw(18) << velocity
                  << std::endl;

        // --- Physics Update (Euler integration) ---
        // Gravity pulls the lander DOWN, so velocity decreases (goes negative)
        velocity -= LUNAR_GRAVITY * TIME_STEP;

        // Altitude changes based on current velocity
        altitude += velocity * TIME_STEP;

        time += TIME_STEP;
    }

    std::cout << std::string(46, '-') << std::endl;
    std::cout << ">> IMPACT at T=" << time << "s  |  "
              << "Final velocity: " << velocity << " m/s" << std::endl;

    return 0;
}