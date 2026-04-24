#include "lander.h"
#include <cmath>

// --- Static constant definitions ---
const double Lander::MASS = 500.0;                // kg
const double Lander::LUNAR_GRAVITY = 1.62;        // m/s²
const double Lander::FUEL_BURN_RATE = 0.05;       // kg/(N·s)  - rough estimate
const double Lander::SAFE_LANDING_VELOCITY = 2.0; // m/s max safe impact speed

Lander::Lander(double startAltitude, double startVelocity, double startFuel)
    : altitude_(startAltitude),
      velocity_(startVelocity),
      fuel_(startFuel),
      thrust_(0.0),
      landed_(false),
      crashed_(false)
{
}

void Lander::applyThrust(double thrustForce)
{
    if (fuel_ <= 0.0)
    {
        thrust_ = 0.0;
        return;
    }

    thrust_ = (thrustForce > 0.0) ? thrustForce : 0.0;
}

void Lander::update(double dt)
{
    if (landed_ || crashed_)
        return;

    double fuelUsed = thrust_ * FUEL_BURN_RATE * dt;
    fuel_ -= fuelUsed;
    if (fuel_ < 0.0)
        fuel_ = 0.0;

    // --- 2. Calculate net acceleration (Newton's 2nd Law: F = ma → a = F/m) ---
    // Gravity pulls down (-), thrust pushes up (+)
    double netForce = thrust_ - (MASS * LUNAR_GRAVITY);
    double acceleration = netForce / MASS;

    // --- 3. Euler integration: update velocity then position ---
    velocity_ += acceleration * dt;
    altitude_ += velocity_ * dt;

    // --- 4. Check landing/crash conditions ---
    if (altitude_ <= 0.0)
    {
        altitude_ = 0.0;

        if (std::abs(velocity_) <= SAFE_LANDING_VELOCITY)
        {
            landed_ = true;
        }
        else
        {
            crashed_ = true;
        }
    }
}

double Lander::getAltitude() const { return altitude_; }
double Lander::getVelocity() const { return velocity_; }
double Lander::getFuel() const { return fuel_; }
double Lander::getThrust() const { return thrust_; }
bool Lander::hasLanded() const { return landed_; }
bool Lander::hasCrashed() const { return crashed_; }