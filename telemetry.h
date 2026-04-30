#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <string>
#include "lander.h"
#include "pid_controller.h"

class Telemetry
{
public:
    Telemetry(double maxAltitude, double maxFuel, double maxThrust);

    void render(const Lander &lander,
                const PIDController &pid,
                double time,
                double thrustCmd,
                double windForce,
                double sensorNoise);

private:
    bool firstRender_;
    double maxAltitude_;
    double maxFuel_;
    double maxThrust_;

    static const int LINE_COUNT = 22;

    std::string makeBar(double ratio, int width = 16) const;
    void printLine(const std::string &content) const;
    void printSep() const;
    void printTop() const;
};

#endif