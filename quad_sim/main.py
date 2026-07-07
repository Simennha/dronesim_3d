import sys
import pygame

import parameters as p
from drone import Drone
from renderer import Renderer, screen_to_world
from controller import PositionController
from environment import default_obstacles, find_collision, Wind, Battery
from menu import run_menu


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def fresh_drone_and_controller(z_target):
    drone = Drone(x=0.0, y=0.0, z=0.0)
    controller = PositionController(z_target=z_target)
    return drone, controller


def main():
    renderer = Renderer()
    clock = pygame.time.Clock()

    settings = run_menu(renderer, clock)
    if settings is None:
        renderer.quit()
        sys.exit()

    obstacles_enabled = settings["obstacles"]
    waypoints_enabled = settings["waypoints"]
    obstacles = default_obstacles() if obstacles_enabled else []
    wind = Wind(enabled=settings["wind"])
    battery = Battery(enabled=settings["battery"])

    z_target = p.Z_TARGET_DEFAULT
    drone, controller = fresh_drone_and_controller(z_target)

    autopilot = False
    target = None
    waypoints = []
    was_manual = False
    was_forced_land = False
    crash_timer = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_p:
                    autopilot = not autopilot
                    if autopilot and target is None:
                        target = (drone.x, drone.y, z_target)
                        controller.set_target(*target, psi_target=drone.psi, state=drone.get_state())
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    wx, wy = screen_to_world(*event.pos)
                    z_target = max(z_target, p.Z_TARGET_DEFAULT)
                    if waypoints_enabled:
                        waypoints.append((wx, wy, z_target))
                        if target is None:
                            target = waypoints.pop(0)
                            controller.set_target(*target, psi_target=drone.psi, state=drone.get_state())
                    else:
                        target = (wx, wy, z_target)
                        controller.set_target(*target, psi_target=drone.psi, state=drone.get_state())
                    autopilot = True
                elif event.button == 3:
                    waypoints.clear()
                    target = None
                    autopilot = False

        keys = pygame.key.get_pressed()

        forced_land = battery.is_low
        if forced_land and not was_forced_land:
            waypoints.clear()
            target = (drone.x, drone.y, 0.0)
            controller.set_target(*target, psi_target=drone.psi, state=drone.get_state())
            autopilot = True
        was_forced_land = forced_land

        if forced_land:
            z_target = 0.0
            controller.z_target = 0.0
        else:
            # Up/Down always adjust the altitude-hold target, independent of xy mode
            if keys[pygame.K_UP]:
                z_target += p.Z_TARGET_RATE * p.DT
            if keys[pygame.K_DOWN]:
                z_target -= p.Z_TARGET_RATE * p.DT
            z_target = clamp(z_target, p.Z_TARGET_MIN, p.Z_TARGET_MAX)
            controller.z_target = z_target

            # Q/E always adjust the heading target, independent of xy mode
            if keys[pygame.K_q]:
                controller.psi_target += p.YAW_RATE * p.DT
            if keys[pygame.K_e]:
                controller.psi_target -= p.YAW_RATE * p.DT

        xy_manual_pressed = (not forced_land) and any(keys[k] for k in (
            pygame.K_LEFT, pygame.K_RIGHT, pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d,
        ))

        if autopilot and target is not None and not xy_manual_pressed:
            if was_manual:
                controller.reset_pids(state=drone.get_state())
            w_front, w_right, w_back, w_left = controller.compute_motor_speeds(drone.get_state(), p.DT)
        else:
            # A/D (or Left/Right) -> commanded pitch -> move along world x
            theta_desired = 0.0
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                theta_desired -= p.MANUAL_TILT_ANGLE
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                theta_desired += p.MANUAL_TILT_ANGLE

            # W/S -> commanded roll -> move along world y
            phi_desired = 0.0
            if keys[pygame.K_w]:
                phi_desired -= p.MANUAL_TILT_ANGLE
            if keys[pygame.K_s]:
                phi_desired += p.MANUAL_TILT_ANGLE

            w_front, w_right, w_back, w_left = controller.compute_motor_speeds_manual(
                drone.get_state(), theta_desired, phi_desired, p.DT)

        was_manual = xy_manual_pressed

        w_front = clamp(w_front, p.W_MIN, p.W_MAX)
        w_right = clamp(w_right, p.W_MIN, p.W_MAX)
        w_back = clamp(w_back, p.W_MIN, p.W_MAX)
        w_left = clamp(w_left, p.W_MIN, p.W_MAX)

        if forced_land and drone.z <= 0.05:
            w_front = w_right = w_back = w_left = 0.0

        battery.step(w_front, w_right, w_back, w_left, p.DT)
        wind_ax, wind_ay = wind.step(p.DT)

        drone.step(w_front, w_right, w_back, w_left, dt=p.DT, wind_ax=wind_ax, wind_ay=wind_ay)

        if waypoints_enabled and autopilot and target is not None and waypoints:
            dx = target[0] - drone.x
            dy = target[1] - drone.y
            dz = target[2] - drone.z
            close = (dx * dx + dy * dy + dz * dz) ** 0.5 < p.WAYPOINT_REACH_RADIUS_M
            slow = abs(drone.vx) < 0.15 and abs(drone.vy) < 0.15 and abs(drone.vz) < 0.15
            if close and slow:
                target = waypoints.pop(0)
                controller.set_target(*target, psi_target=drone.psi, state=drone.get_state())

        if obstacles_enabled and crash_timer <= 0.0:
            hit = find_collision(drone.x, drone.y, obstacles)
            if hit is not None:
                drone, controller = fresh_drone_and_controller(z_target)
                autopilot = False
                target = None
                waypoints.clear()
                crash_timer = 2.0

        if crash_timer > 0.0:
            crash_timer -= p.DT

        renderer.render(drone.get_state(), clock.get_fps(), target=target, autopilot=autopilot,
                         z_target=z_target, obstacles=obstacles, waypoints=waypoints,
                         battery=battery, crashed=crash_timer > 0.0)

        clock.tick(60)

    renderer.quit()
    sys.exit()


if __name__ == "__main__":
    main()
