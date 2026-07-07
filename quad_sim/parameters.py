import math

MASS = 3.0
IXX = 0.0075
IYY = 0.0075
IZZ = 0.015

K = 1.0e-2
B = 1.0e-4
L = 0.2

GRAVITY = 9.81

DT = 1.0 / 60.0

W_MIN = 0.0
W_MAX = 90.0
W_HOVER = math.sqrt(MASS * GRAVITY / (4 * K))

W_STEP = 0.5
W_DIFF_STEP = 0.25
W_YAW_STEP = 0.25

Z_TARGET_DEFAULT = 1.0
Z_TARGET_MIN = 0.0
Z_TARGET_MAX = 20.0
Z_TARGET_RATE = 1.5  # m/s, how fast Up/Down change the altitude hold target

MANUAL_TILT_ANGLE = math.radians(20.0)  # commanded tilt while a manual key is held
YAW_RATE = math.radians(60.0)  # rad/s, how fast Q/E turn the heading target

MAX_TILT_ANGLE = math.radians(60.0)

TORQUE_ROLL_MAX = 0.4
TORQUE_PITCH_MAX = 0.4
TORQUE_YAW_MAX = 0.2

DRONE_COLLISION_RADIUS_M = 0.25
GROUND_FRICTION = 3.0  # 1/s, decelerates horizontal sliding once landed

# --- Obstacles ---
OBSTACLES = [
    # (x, y, radius) in meters
    (4.0, 2.0, 0.8),
    (-3.0, 4.0, 1.0),
    (-5.0, -3.0, 0.7),
    (3.0, -5.0, 0.9),
    (0.0, 6.0, 0.6),
    (7.0, -1.0, 1.1),
]

# --- Wind ---
WIND_ACCEL_STD = 0.6       # m/s^2, std dev of the random-walk wind acceleration
WIND_MAX_ACCEL = 2.0       # m/s^2, clamp so gusts can't get absurd
WIND_DAMPING = 0.5         # 1/s, pulls wind acceleration back toward zero over time

# --- Waypoints ---
WAYPOINT_REACH_RADIUS_M = 0.3

# --- Battery ---
BATTERY_FULL_SECONDS = 120.0   # seconds of hover-power flight on a full charge
BATTERY_LOW_FRACTION = 0.15    # below this, force an auto-land
