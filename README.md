# Dronesim 3D

A 3D quadcopter physics simulator built with Python and pygame, viewed top-down (looking straight down the world z-axis, in the xy-plane). It simulates a full quadrotor in a "+" configuration with four rotors, rigid-body roll/pitch/yaw/altitude dynamics (Euler angles, no quaternions), and a cascaded PID autopilot for 3D position control. Altitude is always held automatically; you fly x/y manually (self-leveling angle mode) or click anywhere in the world to fly there autonomously.

This is the 3D sibling of [dronesim](https://github.com/Simennha/dronesim) (a 2D, side-view planar quadcopter). The cascaded PID design (altitude -> thrust, position -> tilt angle, tilt angle -> torque -> rotor mixing) is carried over and extended with a fourth rotor, a yaw axis, tilt-compensated thrust, and a proper 3D thrust-vector rotation.

## Features

- Rigid-body physics: 4-rotor thrust/torque mixing, gravity, roll/pitch/yaw dynamics, ground collision + friction (`quad_sim/drone.py`)
- Cascaded PID position controller: altitude -> thrust (tilt-compensated), xy position -> desired tilt angle, tilt angle -> torque, yaw hold (`quad_sim/controller.py`)
- Altitude and heading are always auto-held; manual flight is self-leveling angle-mode (release a key and it levels out instead of spinning up)
- Manual keyboard control and full xy autopilot, switchable at runtime
- A pre-game settings menu to toggle optional gameplay features (`quad_sim/menu.py`)
- Optional obstacles, wind, waypoint queueing, and battery drain (`quad_sim/environment.py`)
- Top-down renderer (xy-plane) with a world grid, target/waypoint markers, obstacles, battery HUD, and live telemetry (`quad_sim/renderer.py`)

## Requirements

- Python 3.12+
- pygame >= 2.5.0

## Setup

```bash
pip install -r quad_sim/requirements.txt
```

## Running

```bash
python quad_sim/main.py
```

A settings menu appears first. Use Up/Down to select an option, Space to toggle it on/off, and Enter to start flying:

| Option | Effect |
| --- | --- |
| Hindringer (obstacles) | Fixed circular buildings on the map; flying into one resets the drone to the origin after a short "crashed" message |
| Vind (wind) | A slowly-varying random gust pushes the drone in the xy-plane |
| Rutepunkter (waypoints) | Left-clicks queue up multiple targets instead of replacing the current one; the drone visits them in order |
| Batteri (battery) | A charge meter drains with motor power draw; below 15% the drone ignores manual/autopilot input and forces an auto-land in place |

## Controls

| Input | Action |
| --- | --- |
| Up / Down | Raise / lower the altitude-hold target (climb / descend) |
| A / D (or Left/Right) | Commanded pitch -> move along world x (self-levels on release) |
| W / S | Commanded roll -> move along world y (self-levels on release) |
| Q / E | Turn the heading target (yaw) |
| Left click | Set target (x, y) and enable autopilot (queues as a waypoint if that mode is on) |
| Right click | Clear the current target/waypoint queue and stop the autopilot |
| P | Toggle full xy autopilot on/off |
| Esc | Quit |

## Project layout

- `quad_sim/main.py` - game loop, input handling, ties simulation and rendering together
- `quad_sim/drone.py` - drone state and 3D rigid-body physics integration (Euler angles)
- `quad_sim/controller.py` - PID class and the cascaded 3D position controller
- `quad_sim/environment.py` - obstacles, wind, and battery models
- `quad_sim/menu.py` - pre-game settings screen
- `quad_sim/renderer.py` - pygame top-down rendering and world/screen coordinate conversion
- `quad_sim/parameters.py` - physical constants and tuning parameters
