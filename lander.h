#ifndef LANDER_H
#define LANDER_H

class Lander
{
public:
    Lander(double startAltitude, double startVelocity, double startFuel);

    void applyThrust(double thrustForce);
    void applyDisturbance(double forceNewtons); // NEW: wind / external forces
    void update(double dt);

    double getAltitude() const;
    double getVelocity() const;
    double getFuel() const;
    double getThrust() const;
    double getAcceleration() const;
    bool hasLanded() const;
    bool hasCrashed() const;

private:
    double altitude_;
    double velocity_;
    double thrust_;
    double fuel_;
    double disturbanceForce_;
    double lastAcceleration_;

    static const double MASS;
    static const double LUNAR_GRAVITY;
    static const double FUEL_BURN_RATE;
    static const double SAFE_LANDING_VELOCITY;

    bool landed_;
    bool crashed_;
};

#endif