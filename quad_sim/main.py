import sys
import pygame

import parameters as p
from drone import Drone
from renderer import Renderer, screen_to_world
from controller import PositionController


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def main():
    renderer = Renderer()
    clock = pygame.time.Clock()

    drone = Drone(x=0.0, y=0.0, z=0.0)

    z_target = p.Z_TARGET_DEFAULT

    controller = PositionController(z_target=z_target)
    autopilot = False
    target = None
    was_manual = False

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
                wx, wy = screen_to_world(*event.pos)
                z_target = max(z_target, p.Z_TARGET_DEFAULT)
                target = (wx, wy, z_target)
                controller.set_target(*target, psi_target=drone.psi, state=drone.get_state())
                autopilot = True

        keys = pygame.key.get_pressed()

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

        xy_manual_pressed = any(keys[k] for k in (
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

        drone.step(w_front, w_right, w_back, w_left, dt=p.DT)

        renderer.render(drone.get_state(), clock.get_fps(), target=target, autopilot=autopilot, z_target=z_target)

        clock.tick(60)

    renderer.quit()
    sys.exit()


if __name__ == "__main__":
    main()
