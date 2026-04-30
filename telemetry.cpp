#include "telemetry.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <string>

static const int INNER = 54;

Telemetry::Telemetry(double maxAltitude, double maxFuel, double maxThrust)
    : firstRender_(true),
      maxAltitude_(maxAltitude),
      maxFuel_(maxFuel),
      maxThrust_(maxThrust)
{
}

std::string Telemetry::makeBar(double ratio, int width) const
{
    ratio = std::max(0.0, std::min(1.0, ratio));
    int filled = static_cast<int>(std::round(ratio * width));
    return "[" + std::string(filled, '#') + std::string(width - filled, '-') + "]";
}

void Telemetry::printLine(const std::string &content) const
{
    std::string s = content;

    if ((int)s.size() < INNER)
        s += std::string(INNER - (int)s.size(), ' ');
    else
        s = s.substr(0, INNER);
    std::cout << "| " << s << " |\n";
}

void Telemetry::printSep() const
{
    std::cout << "+" << std::string(INNER + 2, '-') << "+\n";
}

void Telemetry::printTop() const
{
    std::cout << "+" << std::string(INNER + 2, '=') << "+\n";
}

void Telemetry::render(const Lander &lander, const PIDController &pid,
                       double time, double thrustCmd,
                       double windForce, double sensorNoise)
{

    if (!firstRender_)
    {
        std::cout << "\033[" << LINE_COUNT << "A";
    }
    firstRender_ = false;

    std::ostringstream s;
    s << std::fixed << std::setprecision(2);

    // ---- LINE 1 ----
    printTop();

    // ---- LINE 2 ---- title
    printLine("     LUNAR LANDER TELEMETRY  //  AUTONOMOUS DESCENT");

    // ---- LINE 3 ----
    printTop();

    // ---- LINE 4 ---- mission time
    s.str("");
    s << "  MISSION TIME  :  " << std::setw(7) << time << " s";
    printLine(s.str());

    // ---- LINE 5 ----
    printSep();

    // ---- LINE 6 ---- altitude with bar
    double altRatio = lander.getAltitude() / maxAltitude_;
    s.str("");
    s << "  ALTITUDE  :  " << std::setw(7) << lander.getAltitude()
      << " m    " << makeBar(altRatio)
      << "  " << std::setw(3) << (int)(altRatio * 100) << "%";
    printLine(s.str());

    // ---- LINE 7 ---- velocity
    s.str("");
    s << "  VELOCITY  :  " << std::setw(8) << lander.getVelocity() << " m/s";
    printLine(s.str());

    // ---- LINE 8 ---- acceleration (was a TODO since phase 2!)
    s.str("");
    s << "  ACCEL     :  " << std::setw(8) << lander.getAcceleration() << " m/s^2";
    printLine(s.str());

    // ---- LINE 9 ----
    printSep();

    // ---- LINE 10 ---- thrust with bar
    double thrustRatio = thrustCmd / maxThrust_;
    s.str("");
    s << "  THRUST    :  " << std::setw(7) << thrustCmd
      << " N    " << makeBar(thrustRatio)
      << "  " << std::setw(3) << (int)(thrustRatio * 100) << "%";
    printLine(s.str());

    // ---- LINE 11 ---- fuel with bar
    double fuelRatio = lander.getFuel() / maxFuel_;
    s.str("");
    s << "  FUEL      :  " << std::setw(7) << lander.getFuel()
      << " kg   " << makeBar(fuelRatio)
      << "  " << std::setw(3) << (int)(fuelRatio * 100) << "%";
    printLine(s.str());

    // ---- LINE 12 ----
    printSep();

    // ---- LINE 13 ---- PID header
    printLine("  --- PID CONTROLLER ---");

    // ---- LINE 14 ---- target velocity & error
    s.str("");
    s << "  Target : " << std::setw(7) << -2.00 << " m/s"
      << "      Error  : " << std::setw(7) << pid.getLastError() << " m/s";
    printLine(s.str());

    // ---- LINE 15 ---- P & I terms
    s.str("");
    s << "  P term : " << std::setw(8) << pid.getLastP() << " N"
      << "      I term : " << std::setw(8) << pid.getLastI() << " N";
    printLine(s.str());

    // ---- LINE 16 ---- D term
    s.str("");
    s << "  D term : " << std::setw(8) << pid.getLastD() << " N";
    printLine(s.str());

    // ---- LINE 17 ----
    printSep();

    // ---- LINE 18 ---- environment header
    printLine("  --- ENVIRONMENT ---");

    // ---- LINE 19 ---- wind and sensor noise
    s.str("");
    s << "  Wind gust  : " << std::setw(7) << windForce << " N"
      << "     Sensor noise : " << std::setw(5) << sensorNoise << " m/s";
    printLine(s.str());

    // ---- LINE 20 ----
    printSep();

    // ---- LINE 21 ---- plain-English status
    std::string status;
    double alt = lander.getAltitude();
    double err = std::abs(pid.getLastError());

    if (alt > 60.0)
        status = "INITIALIZING DESCENT";
    else if (err > 2.0)
        status = "WARNING: VELOCITY DEVIATION  [ PID CORRECTING ]";
    else if (alt < 15.0)
        status = ">>> FINAL APPROACH <<<";
    else
        status = "NOMINAL DESCENT";

    s.str("");
    s << "  STATUS  :  " << status;
    printLine(s.str());

    // ---- LINE 22 ----
    printTop();

    std::cout.flush();
}