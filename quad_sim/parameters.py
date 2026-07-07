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
