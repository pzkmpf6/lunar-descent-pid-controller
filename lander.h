#ifndef LANDER_H
#define LANDER_H

class Lander
{
public:
    Lander(double startAltitude, double startVelocity, double startFuel);

    // --- Core simulation methods ---
    void applyThrust(double thrustForce);
    void update(double dt);

    double getAltitude() const;
    double getVelocity() const;
    double getFuel() const;
    double getThrust() const;
    bool hasLanded() const;
    bool hasCrashed() const;

private:
    // --- Physical state ---
    double altitude_; // meters above surface
    double velocity_; // m/s  (positive = up, negative = falling)
    double thrust_;   // Newtons of upward force applied this tick
    double fuel_;     // kg of fuel remaining

    // --- Lander physical properties ---
    static const double MASS;           // kg (dry mass, no fuel)
    static const double LUNAR_GRAVITY;  // m/s²
    static const double FUEL_BURN_RATE; // kg per Newton per second

    bool landed_;
    bool crashed_;

    static const double SAFE_LANDING_VELOCITY;
};

#endif