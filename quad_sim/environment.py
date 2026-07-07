import random

import parameters as p


def default_obstacles():
    return [tuple(o) for o in p.OBSTACLES]


def find_collision(x, y, obstacles, drone_radius=p.DRONE_COLLISION_RADIUS_M):
    """Return the obstacle (x, y, radius) the drone is touching, or None."""
    for obstacle in obstacles:
        ox, oy, radius = obstacle
        dx = x - ox
        dy = y - oy
        if dx * dx + dy * dy < (radius + drone_radius) ** 2:
            return obstacle
    return None


class Wind:
    """Slowly-varying random-walk wind acceleration, applied in the world xy-plane."""

    def __init__(self, enabled, rng=None):
        self.enabled = enabled
        self.ax = 0.0
        self.ay = 0.0
        self._rng = rng or random.Random()

    def step(self, dt):
        if not self.enabled:
            return 0.0, 0.0

        self.ax += self._rng.gauss(0.0, p.WIND_ACCEL_STD) * dt - p.WIND_DAMPING * self.ax * dt
        self.ay += self._rng.gauss(0.0, p.WIND_ACCEL_STD) * dt - p.WIND_DAMPING * self.ay * dt

        self.ax = max(-p.WIND_MAX_ACCEL, min(p.WIND_MAX_ACCEL, self.ax))
        self.ay = max(-p.WIND_MAX_ACCEL, min(p.WIND_MAX_ACCEL, self.ay))

        return self.ax, self.ay


class Battery:
    """Drains proportional to motor power draw; reports when it's time to force a landing."""

    def __init__(self, enabled):
        self.enabled = enabled
        self.fraction = 1.0

    def step(self, w_front, w_right, w_back, w_left, dt):
        if not self.enabled:
            return
        power_ratio = (w_front ** 2 + w_right ** 2 + w_back ** 2 + w_left ** 2) / (4 * p.W_HOVER ** 2)
        self.fraction -= power_ratio * dt / p.BATTERY_FULL_SECONDS
        self.fraction = max(0.0, self.fraction)

    @property
    def is_low(self):
        return self.enabled and self.fraction <= p.BATTERY_LOW_FRACTION

    @property
    def is_empty(self):
        return self.enabled and self.fraction <= 0.0
