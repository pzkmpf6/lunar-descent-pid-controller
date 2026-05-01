# Lunar Descent PID Controller

A terminal simulation of an autonomous lunar lander written in C++.
The lander uses a **PID controller** to regulate its descent velocity against
lunar gravity, random wind gusts, and IMU sensor noise — and lands safely
without any manual input.

I built this as a portfolio project during my 2nd semester of Automation Systems
to get a hands-on feel for how closed-loop control actually works beyond the theory.

---

## What it looks like

```
+========================================================+
|      LUNAR LANDER TELEMETRY  //  AUTONOMOUS DESCENT    |
+========================================================+
|  MISSION TIME  :    18.70 s                            |
+--------------------------------------------------------+
|  ALTITUDE  :    41.23 m    [########--------]   51%   |
|  VELOCITY  :    -2.04 m/s                              |
|  ACCEL     :     0.11 m/s^2                            |
+--------------------------------------------------------+
|  THRUST    :   821.30 N    [######----------]   41%   |
|  FUEL      :   412.87 kg   [#############---]   83%   |
+--------------------------------------------------------+
|  --- PID CONTROLLER ---                                |
|  Target :   -2.00 m/s      Error  :   -0.04 m/s       |
|  P term :    -12.00 N      I term :     8.61 N         |
|  D term :      3.20 N                                  |
+--------------------------------------------------------+
|  --- ENVIRONMENT ---                                   |
|  Wind gust  :   -14.22 N     Sensor noise :  0.03 m/s |
+--------------------------------------------------------+
|  STATUS  :  NOMINAL DESCENT                            |
+========================================================+
```

The dashboard updates **in-place** every tick using ANSI escape codes —
no scrolling, just a live readout like a real embedded display.

---

## Build and run

**Requirements:** any C++11-compliant compiler (`g++`, `clang++`)

```bash
git clone https://github.com/pzkmpf6/lunar-descent-pid-controller.git
cd lunar-descent-pid-controller

g++ -std=c++11 -O2 -o lander \
    main.cpp lander.cpp pid_controller.cpp telemetry.cpp

./lander
```

> **Windows:** use WSL or Windows Terminal for ANSI escape code support.

Each run prints a seed at startup:
```
Simulation seed: 1718023456  (use this number to replay this exact run)
```
To replay a specific run, replace `std::time(nullptr)` with that number in `main.cpp`.

---

## Project structure

```
lunar-descent-pid-controller/
├── main.cpp               # simulation driver: wind, PID loop, render calls
├── lander.h / lander.cpp  # Lander class — physics state and Euler integration
├── pid_controller.h / .cpp  # generic reusable PID with anti-windup
├── telemetry.h / .cpp     # ANSI terminal dashboard
└── README.md
```

The `PIDController` class has zero dependency on `Lander` —
it just takes a setpoint, a measurement, and a dt. You could
drop it into a different project (temperature control, drone altitude, etc.)
without changing a line.

---

## Physics model

Each tick (`dt = 0.1 s`) the simulation runs Euler integration:

```
a(t)   = ( F_thrust + F_wind - m * g ) / m
v(t+1) = v(t) + a(t) * dt
h(t+1) = h(t) + v(t) * dt
```

| Parameter | Value |
|:---|:---|
| Lander mass | 500 kg |
| Lunar gravity | 1.62 m/s² |
| Fuel burn rate | 0.05 kg / (N·s) |
| Safe landing speed | ≤ 2.0 m/s |
| Max engine thrust | 2000 N |
| Starting altitude | 100 m |
| Starting fuel | 500 kg |

Euler integration isn't the most accurate method (it drifts slightly each step),
but for a slow lunar descent with `dt = 0.1 s` the error is negligible.
RK4 would be the natural upgrade if higher fidelity was needed.

---

## PID controller

The controller tries to hold a constant descent velocity of **−2.0 m/s**
all the way to the ground.

```
u(t) = Kp * e(t)  +  Ki * integral(e)  +  Kd * de/dt

e(t) = setpoint − measured = −2.0 − v_noisy(t)
```

The output `u(t)` is a **correction in Newtons**. It gets added on top of
a pre-computed hover thrust:

```
F_hover = m * g = 500 * 1.62 = 810 N
F_total = F_hover + u(t)          (clamped to 0 – 2000 N)
```

Separating the known physics (feedforward) from the error correction (PID)
means the integrator doesn't have to slowly "discover" that 810 N is needed
to hover — it starts from equilibrium and only handles deviations.

### Why these gains

| Gain | Value | Reasoning |
|:---|---:|:---|
| Kp | 300 | 1 m/s error → 300 N correction. Responsive without oscillating a 500 kg mass |
| Ki | 10 | Slow integrator. With feedforward doing the heavy lifting, Ki just trims small residuals |
| Kd | 80 | Enough damping to suppress overshoot; low enough not to amplify the 0.08 m/s sensor noise |

### Anti-windup

When the engine is physically saturated (clamped at 0 or 2000 N),
the integral would keep accumulating uselessly and then cause a big
overshoot when saturation lifts. So the integral sum is clamped:

```cpp
double integralLimit = (outMax_ - outMin_) / (ki_ > 0.0 ? ki_ : 1.0);
integralSum_ = std::min(std::max(integralSum_, -integralLimit), integralLimit);
```

Note: I used `std::min/std::max` instead of `std::clamp` to stay C++11 compatible.

---

## Environmental disturbances

### Wind — decaying random walk

```cpp
wind(t+1) = wind(t) * 0.97 + N(0, 18 N) * dt
```

The decay factor (0.97) makes the wind naturally drift back toward zero
rather than accumulating forever. Max magnitude is clamped to ±130 N
(about 16% of hover thrust).

### Sensor noise — Gaussian IMU model

```cpp
v_measured = v_true + N(0, 0.08 m/s)
```

The PID **never sees the true velocity** — only the noisy measurement,
exactly like a real flight computer reading from an IMU.
The fact that it still lands safely despite this is the whole point.

---

## How it was built

The project went through 5 phases on separate Git branches,
merged into `main` with `--no-ff` to keep the history readable:

| Phase | Branch | What was added |
|:---|:---|:---|
| 1 | `phase-1/basic-physics` | Gravity loop, Euler integration, console output |
| 2 | `phase-2/lander-class` | `Lander` class, encapsulation, fuel model |
| 3 | `phase-3/pid-controller` | `PIDController` class, feedforward, closed-loop control |
| 4 | `phase-4/telemetry-and-noise` | Live ANSI dashboard, wind model, sensor noise |
| 5 | `phase-5/final-polish` | Cleanup, reproducible seeding, README |

---

## What I learned

Going into this I understood PID control on a whiteboard level.
Actually implementing it showed me a few things that don't come through in lectures:

- **Feedforward matters a lot.** Without it the integrator spends the first
  20+ seconds just "learning" how much gravity pulls. With it, the controller
  is already near-stable on tick one.
- **Derivative noise sensitivity is real.** Kd=80 works fine with
  σ=0.08 m/s noise. Bump it to 200 and the thrust readout starts twitching
  visibly even though the lander still lands — you can see why real systems
  filter the derivative term.
- **Anti-windup is easy to forget and immediately obvious when missing.**
  Remove the integral clamp and the lander overshoots badly on the way down.

