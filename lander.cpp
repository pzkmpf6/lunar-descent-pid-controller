#include "lander.h"
#include <cmath>

const double Lander::MASS = 500.0;
const double Lander::LUNAR_GRAVITY = 1.62;
const double Lander::FUEL_BURN_RATE = 0.05;
const double Lander::SAFE_LANDING_VELOCITY = 2.0;

Lander::Lander(double startAltitude, double startVelocity, double startFuel)
    : altitude_(startAltitude),
      velocity_(startVelocity),
      fuel_(startFuel),
      thrust_(0.0),
      disturbanceForce_(0.0),
      lastAcceleration_(0.0),
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

void Lander::applyDisturbance(double forceNewtons)
{
    disturbanceForce_ = forceNewtons;
}

void Lander::update(double dt)
{
    if (landed_ || crashed_)
        return;

    double fuelUsed = thrust_ * FUEL_BURN_RATE * dt;
    fuel_ -= fuelUsed;
    if (fuel_ < 0.0)
        fuel_ = 0.0;

    double netForce = thrust_ + disturbanceForce_ - (MASS * LUNAR_GRAVITY);
    double acceleration = netForce / MASS;
    lastAcceleration_ = acceleration;

    velocity_ += acceleration * dt;
    altitude_ += velocity_ * dt;

    thrust_ = 0.0;
    disturbanceForce_ = 0.0;

    if (altitude_ <= 0.0)
    {
        altitude_ = 0.0;
        if (std::abs(velocity_) <= SAFE_LANDING_VELOCITY)
            landed_ = true;
        else
            crashed_ = true;
    }
}

double Lander::getAltitude() const { return altitude_; }
double Lander::getVelocity() const { return velocity_; }
double Lander::getFuel() const { return fuel_; }
double Lander::getThrust() const { return thrust_; }
double Lander::getAcceleration() const { return lastAcceleration_; }
bool Lander::hasLanded() const { return landed_; }
bool Lander::hasCrashed() const { return crashed_; }