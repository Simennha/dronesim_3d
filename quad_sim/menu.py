import pygame

MENU_ITEMS = [
    ("obstacles", "Hindringer (bygninger) - krasj ved kontakt"),
    ("wind", "Vind - tilfeldige vindkast"),
    ("waypoints", "Rutepunkter - koe opp flere mal med museklikk"),
    ("battery", "Batteri - lades ut, tvinger landing ved lavt niva"),
]

BG_COLOR = (16, 16, 22)
TITLE_COLOR = (230, 230, 235)
ITEM_COLOR = (200, 220, 255)
SELECTED_COLOR = (255, 210, 90)
HINT_COLOR = (150, 160, 180)


def run_menu(renderer, clock):
    """Pre-game settings screen. Returns a dict of {key: bool}, or None if the user quit."""
    settings = {key: False for key, _ in MENU_ITEMS}
    selected = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return None
                elif event.key in (pygame.K_UP, pygame.K_w):
                    selected = (selected - 1) % len(MENU_ITEMS)
                elif event.key in (pygame.K_DOWN, pygame.K_s):
                    selected = (selected + 1) % len(MENU_ITEMS)
                elif event.key == pygame.K_SPACE:
                    key = MENU_ITEMS[selected][0]
                    settings[key] = not settings[key]
                elif event.key == pygame.K_RETURN:
                    running = False

        renderer.screen.fill(BG_COLOR)
        title_surf = renderer.font.render("DRONESIM 3D - innstillinger", True, TITLE_COLOR)
        renderer.screen.blit(title_surf, (60, 60))

        y = 130
        for i, (key, label) in enumerate(MENU_ITEMS):
            marker = ">" if i == selected else " "
            state = "PA" if settings[key] else "AV"
            color = SELECTED_COLOR if i == selected else ITEM_COLOR
            line = f"{marker} [{state}]  {label}"
            surf = renderer.font.render(line, True, color)
            renderer.screen.blit(surf, (60, y))
            y += 36

        y += 20
        for line in ("Opp/Ned = velg, MELLOMROM = av/pa, ENTER = start, ESC = avslutt",):
            surf = renderer.font.render(line, True, HINT_COLOR)
            renderer.screen.blit(surf, (60, y))
            y += 28

        pygame.display.flip()
        clock.tick(60)

    return settings
