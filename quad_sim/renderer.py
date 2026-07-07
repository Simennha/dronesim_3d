import math
import pygame

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800

PIXELS_PER_METER = 60
GRID_SPACING_M = 1.0
GRID_LINES = 20

BG_COLOR = (16, 16, 22)
GRID_COLOR = (36, 36, 46)
AXIS_COLOR = (60, 60, 75)
ARM_COLOR = (180, 180, 190)
ROTOR_COLOR = (250, 180, 60)
FRONT_MARK_COLOR = (255, 90, 90)
BODY_COLOR = (230, 230, 235)
TARGET_COLOR = (80, 220, 120)
WAYPOINT_COLOR = (80, 140, 220)
TEXT_COLOR = (200, 220, 255)
OBSTACLE_COLOR = (120, 60, 60)
OBSTACLE_OUTLINE_COLOR = (200, 90, 90)
CRASH_COLOR = (255, 80, 80)

BATTERY_OK_COLOR = (110, 220, 130)
BATTERY_LOW_COLOR = (240, 190, 80)
BATTERY_EMPTY_COLOR = (230, 70, 70)

BODY_ARM_M = 0.2
ROTOR_RADIUS_PX = 8

ORIGIN = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)


def world_to_screen(x, y, origin=ORIGIN):
    screen_x = origin[0] + x * PIXELS_PER_METER
    screen_y = origin[1] - y * PIXELS_PER_METER
    return int(screen_x), int(screen_y)


def screen_to_world(screen_x, screen_y, origin=ORIGIN):
    x = (screen_x - origin[0]) / PIXELS_PER_METER
    y = (origin[1] - screen_y) / PIXELS_PER_METER
    return x, y


def _rotate(local_x, local_y, psi):
    cos_p, sin_p = math.cos(psi), math.sin(psi)
    wx = local_x * cos_p - local_y * sin_p
    wy = local_x * sin_p + local_y * cos_p
    return wx, wy


class Renderer:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Quad Sim 3D - top-down (xy)")
        self.font = pygame.font.SysFont("consolas", 18)

    def draw_background(self):
        self.screen.fill(BG_COLOR)
        origin = world_to_screen(0, 0)

        for i in range(-GRID_LINES, GRID_LINES + 1):
            x = origin[0] + i * GRID_SPACING_M * PIXELS_PER_METER
            pygame.draw.line(self.screen, GRID_COLOR, (x, 0), (x, SCREEN_HEIGHT), 1)
        for i in range(-GRID_LINES, GRID_LINES + 1):
            y = origin[1] + i * GRID_SPACING_M * PIXELS_PER_METER
            pygame.draw.line(self.screen, GRID_COLOR, (0, y), (SCREEN_WIDTH, y), 1)

        pygame.draw.line(self.screen, AXIS_COLOR, (0, origin[1]), (SCREEN_WIDTH, origin[1]), 2)
        pygame.draw.line(self.screen, AXIS_COLOR, (origin[0], 0), (origin[0], SCREEN_HEIGHT), 2)

    def draw_drone(self, state):
        x, y, z, psi = state["x"], state["y"], state["z"], state["psi"]
        center = world_to_screen(x, y)

        arms_local = {
            "front": (BODY_ARM_M, 0.0),
            "back": (-BODY_ARM_M, 0.0),
            "left": (0.0, BODY_ARM_M),
            "right": (0.0, -BODY_ARM_M),
        }

        rotor_screen = {}
        for name, (lx, ly) in arms_local.items():
            wx, wy = _rotate(lx, ly, psi)
            rotor_screen[name] = world_to_screen(x + wx, y + wy)

        pygame.draw.line(self.screen, ARM_COLOR, rotor_screen["front"], rotor_screen["back"], 3)
        pygame.draw.line(self.screen, ARM_COLOR, rotor_screen["left"], rotor_screen["right"], 3)

        # higher altitude -> slightly larger rotor markers (rough depth cue for a top-down view)
        altitude_scale = max(0.6, min(1.6, 1.0 + z / 10.0))
        rotor_radius = int(ROTOR_RADIUS_PX * altitude_scale)

        for name, pos in rotor_screen.items():
            color = FRONT_MARK_COLOR if name == "front" else ROTOR_COLOR
            pygame.draw.circle(self.screen, color, pos, rotor_radius)

        pygame.draw.circle(self.screen, BODY_COLOR, center, 4)

    def draw_target(self, target):
        if target is None:
            return
        tx, ty = target[0], target[1]
        pos = world_to_screen(tx, ty)
        radius = 10
        pygame.draw.circle(self.screen, TARGET_COLOR, pos, radius, 2)
        pygame.draw.line(self.screen, TARGET_COLOR, (pos[0] - radius, pos[1]), (pos[0] + radius, pos[1]), 1)
        pygame.draw.line(self.screen, TARGET_COLOR, (pos[0], pos[1] - radius), (pos[0], pos[1] + radius), 1)

    def draw_waypoints(self, waypoints):
        for wx, wy, _ in waypoints:
            pos = world_to_screen(wx, wy)
            pygame.draw.circle(self.screen, WAYPOINT_COLOR, pos, 6, 2)

    def draw_obstacles(self, obstacles):
        for ox, oy, radius in obstacles:
            pos = world_to_screen(ox, oy)
            px_radius = int(radius * PIXELS_PER_METER)
            pygame.draw.circle(self.screen, OBSTACLE_COLOR, pos, px_radius)
            pygame.draw.circle(self.screen, OBSTACLE_OUTLINE_COLOR, pos, px_radius, 2)

    def draw_hud(self, state, fps, target=None, autopilot=False, z_target=None, battery=None, crashed=False):
        lines = [
            f"x = {state['x']:.2f} m   y = {state['y']:.2f} m   z = {state['z']:.2f} m"
            + (f"   (hoyde-mal: {z_target:.2f} m)" if z_target is not None else ""),
            f"roll = {math.degrees(state['phi']):.1f} deg  pitch = {math.degrees(state['theta']):.1f} deg  yaw = {math.degrees(state['psi']):.1f} deg",
            f"vx = {state['vx']:.2f}  vy = {state['vy']:.2f}  vz = {state['vz']:.2f} m/s",
            f"w: front={state['w_front']:.1f} right={state['w_right']:.1f} back={state['w_back']:.1f} left={state['w_left']:.1f}",
            f"FPS: {fps:.0f}",
            f"Autopilot: {'PA' if autopilot else 'AV'}"
            + (f"   Mal: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})" if target else ""),
            "A/D = flytt x, W/S = flytt y, Q/E = gir (yaw). Hoyden holdes automatisk.",
            "Opp/Ned = juster hoyde-mal. Venstreklikk = sett mal. Hoyreklikk = avbryt.",
            "P = av/pa autopilot. ESC = avslutt.",
        ]
        y = 10
        for line in lines:
            surf = self.font.render(line, True, TEXT_COLOR)
            self.screen.blit(surf, (10, y))
            y += surf.get_height() + 4

        if battery is not None and battery.enabled:
            self._draw_battery(battery, y)
            y += 30

        if crashed:
            surf = self.font.render("KRASJET! Nullstiller drone...", True, CRASH_COLOR)
            self.screen.blit(surf, (10, y))

    def _draw_battery(self, battery, y):
        pct = battery.fraction
        if battery.is_empty:
            color = BATTERY_EMPTY_COLOR
        elif battery.is_low:
            color = BATTERY_LOW_COLOR
        else:
            color = BATTERY_OK_COLOR

        bar_width, bar_height = 160, 16
        pygame.draw.rect(self.screen, (60, 60, 70), (10, y, bar_width, bar_height), 1)
        pygame.draw.rect(self.screen, color, (12, y + 2, int((bar_width - 4) * pct), bar_height - 4))

        label = "TOM" if battery.is_empty else ("LAV BATTERI - LANDER" if battery.is_low else "Batteri")
        surf = self.font.render(f"{label} {pct * 100:.0f}%", True, color)
        self.screen.blit(surf, (bar_width + 20, y - 2))

    def render(self, state, fps, target=None, autopilot=False, z_target=None,
               obstacles=None, waypoints=None, battery=None, crashed=False):
        self.draw_background()
        if obstacles:
            self.draw_obstacles(obstacles)
        if waypoints:
            self.draw_waypoints(waypoints)
        self.draw_target(target)
        self.draw_drone(state)
        self.draw_hud(state, fps, target=target, autopilot=autopilot, z_target=z_target,
                      battery=battery, crashed=crashed)
        pygame.display.flip()

    def quit(self):
        pygame.quit()
