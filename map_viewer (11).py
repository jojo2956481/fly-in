"""
Pygame map viewer avec camera : zoom a la molette (centre sur le curseur)
et deplacement (pan) en glissant avec le clic gauche.
"""

import math
import pygame
from src.parsing_map import parser_file
from cooperative_dijkstra import schedule_drones

# --- Config fenêtre ---
WINDOW_W, WINDOW_H = 1000, 700
BG_COLOR = (250, 240, 217)
LINE_COLOR = (150, 140, 120)
TEXT_COLOR = (60, 55, 45)
HUB_OUTLINE_COLOR = (60, 55, 45)
HUB_RADIUS = 14
LINE_WIDTH = 1
FONT_NAME = "consolas"
FONT_SIZE = 12          # taille de base à zoom = 1.0
FONT_SIZE_MIN = 8       # en dessous, le label est masqué (illisible)
FONT_SIZE_MAX = 22      # taille plafond, même très zoomé

# --- Config caméra ---
UNIT_SCALE = 60          # 1 unité de map = 60px à zoom 1.0
ZOOM_MIN, ZOOM_MAX = 0.15, 6.0
ZOOM_STEP = 1.15          # facteur multiplicatif par cran de molette

_font_cache = {}


def get_font(size):
    """Renvoie une police Pygame de la taille demandée, en la mettant en cache
    pour éviter de recréer un objet Font à chaque frame/hub."""
    size = int(size)
    if size not in _font_cache:
        _font_cache[size] = pygame.font.SysFont(FONT_NAME, size, bold=True)
    return _font_cache[size]

COLOR_MAP = {
    "green": (0, 200, 0), "red": (220, 30, 30), "purple": (150, 60, 200),
    "black": (10, 10, 10), "brown": (110, 70, 40), "orange": (255, 140, 0),
    "maroon": (128, 0, 0), "gold": (212, 175, 55), "darkred": (139, 0, 0),
    "violet": (150, 80, 220), "crimson": (200, 20, 60), "rainbow": (255, 255, 255),
    None: (180, 180, 180),
}


def compute_world_layout(hubs, unit_scale=UNIT_SCALE):
    """Positions fixes en 'coordonnées monde' (pixels à zoom=1.0),
    indépendantes de la taille de fenêtre et de la caméra.
    Calculées une seule fois, jamais recalculées au resize/zoom/pan.
    """
    return {
        h.name: (h.x * unit_scale, -h.y * unit_scale)  # -y : origine visuelle vers le haut
        for h in hubs
    }


class Camera:
    """Gère le décalage (pan) et le zoom, et convertit monde -> écran."""

    def __init__(self, width, height):
        self.x = 0.0       # position du centre de vue, en coordonnées monde
        self.y = 0.0
        self.zoom = 1.0
        self.width = width
        self.height = height

    def resize(self, width, height):
        self.width = width
        self.height = height

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.zoom = 1.0

    def world_to_screen(self, wx, wy):
        sx = (wx - self.x) * self.zoom + self.width / 2
        sy = (wy - self.y) * self.zoom + self.height / 2
        return int(sx), int(sy)

    def screen_to_world(self, sx, sy):
        wx = (sx - self.width / 2) / self.zoom + self.x
        wy = (sy - self.height / 2) / self.zoom + self.y
        return wx, wy

    def zoom_at(self, screen_pos, factor):
        """Zoom en gardant fixe le point du monde sous le curseur."""
        world_before = self.screen_to_world(*screen_pos)
        self.zoom = max(ZOOM_MIN, min(ZOOM_MAX, self.zoom * factor))
        world_after = self.screen_to_world(*screen_pos)
        self.x += world_before[0] - world_after[0]
        self.y += world_before[1] - world_after[1]

    def pan(self, dx, dy):
        self.x -= dx / self.zoom
        self.y -= dy / self.zoom


def invert_color(color):
    r, g, b = color[:3]
    return (255 - r, 255 - g, 255 - b)


DRONE_COLOR = (0, 140, 200)
DRONE_SIZE = 14  # longueur du curseur (pointe -> arrière), à zoom = 1.0


def _segment_direction(path, index, world_positions):
    """Direction (dx, dy) du tronçon path[index]->path[index+1]. Si c'est
    un segment 'rester sur place' (même hub), cherche la direction du
    prochain vrai déplacement, sinon celle du précédent, sinon (1, 0)."""
    hub_a, _ = path[index]
    hub_b, _ = path[index + 1]
    if hub_a != hub_b:
        xa, ya = world_positions[hub_a]
        xb, yb = world_positions[hub_b]
        return (xb - xa, yb - ya)

    # segment "rester" : cherche en avant un vrai mouvement
    for i in range(index + 1, len(path) - 1):
        a, _ = path[i]
        b, _ = path[i + 1]
        if a != b:
            xa, ya = world_positions[a]
            xb, yb = world_positions[b]
            return (xb - xa, yb - ya)

    # sinon cherche en arrière
    for i in range(index - 1, -1, -1):
        a, _ = path[i]
        b, _ = path[i + 1]
        if a != b:
            xa, ya = world_positions[a]
            xb, yb = world_positions[b]
            return (xb - xa, yb - ya)

    return (1.0, 0.0)  # aucun mouvement dans tout le chemin : direction par défaut


def interpolate_drone_state(path, sim_turn, world_positions):
    """Calcule la position (monde) ET la direction (angle en radians) d'un
    drone à un instant donné (sim_turn, valeur flottante).
    Retourne None si le drone n'est pas encore parti ou déjà arrivé."""
    if sim_turn < path[0][1] or sim_turn > path[-1][1]:
        return None

    for i, ((hub_a, t_a), (hub_b, t_b)) in enumerate(zip(path, path[1:])):
        if t_a <= sim_turn <= t_b:
            progress = 0.0 if t_b == t_a else (sim_turn - t_a) / (t_b - t_a)
            xa, ya = world_positions[hub_a]
            xb, yb = world_positions[hub_b]
            pos = (xa + (xb - xa) * progress, ya + (yb - ya) * progress)
            dx, dy = _segment_direction(path, i, world_positions)
            angle = math.atan2(dy, dx)
            return pos, angle

    xb, yb = world_positions[path[-1][0]]
    dx, dy = _segment_direction(path, len(path) - 2, world_positions)
    return (xb, yb), math.atan2(dy, dx)


def draw_drone_arrow(screen, screen_pos, angle, size, color, outline_color=(20, 20, 20)):
    """Dessine un drone en forme de curseur de souris (pointe fine, dos en
    biseau avec petite encoche), orienté selon angle (radians), centré sur
    screen_pos. La pointe indique le sens du déplacement."""
    # points en coordonnées locales, pointe a l'origine, orientee vers +x
    local_points = [
        (0.00, 0.00),
        (-0.75, 0.25),
        (-0.50, 0.30),
        (-0.65, 0.75),
        (-0.45, 0.65),
        (-0.30, 0.28),
        (-0.15, 0.30),
    ]

    cos_a, sin_a = math.cos(angle), math.sin(angle)
    x, y = screen_pos
    points = []
    for lx, ly in local_points:
        rx = lx * cos_a - ly * sin_a
        ry = lx * sin_a + ly * cos_a
        points.append((x + rx * size, y + ry * size))

    pygame.draw.polygon(screen, color, points)
    pygame.draw.polygon(screen, outline_color, points, 1)


def draw_zone_symbol(screen, pos, radius, zone, symbol_color):
    """Dessine un symbole au centre du hub selon sa zone.
    'normal' -> rien. 'blocked' -> croix. 'restricted' -> triangle. 'priority' -> carré.
    """
    x, y = pos
    thickness = max(1, int(radius / 6))

    if zone == "blocked":
        s = radius * 0.5
        pygame.draw.line(screen, symbol_color, (x - s, y - s), (x + s, y + s), thickness)
        pygame.draw.line(screen, symbol_color, (x - s, y + s), (x + s, y - s), thickness)

    elif zone == "restricted":
        s = radius * 0.6
        points = [(x, y - s), (x - s, y + s * 0.7), (x + s, y + s * 0.7)]
        pygame.draw.polygon(screen, symbol_color, points, thickness)

    elif zone == "priority":
        s = radius * 0.5
        rect = pygame.Rect(x - s, y - s, s * 2, s * 2)
        pygame.draw.rect(screen, symbol_color, rect, thickness)

    # "normal" -> pas de symbole


def draw_map(screen, drone_map, world_positions, camera):
    screen.fill(BG_COLOR)

    for conn in drone_map.connections:
        p1 = camera.world_to_screen(*world_positions[conn.src])
        p2 = camera.world_to_screen(*world_positions[conn.dst])
        pygame.draw.line(screen, LINE_COLOR, p1, p2, LINE_WIDTH)

    label_font_size = max(FONT_SIZE_MIN, min(FONT_SIZE_MAX, FONT_SIZE * camera.zoom))
    show_labels = (FONT_SIZE * camera.zoom) >= FONT_SIZE_MIN
    label_font = get_font(label_font_size) if show_labels else None

    for hub in drone_map.hubs:
        pos = camera.world_to_screen(*world_positions[hub.name])
        color = COLOR_MAP.get(hub.color, COLOR_MAP[None])
        radius = max(2, int((HUB_RADIUS + (4 if hub.kind in ("start", "end") else 0)) * camera.zoom))
        pygame.draw.circle(screen, color, pos, radius)
        pygame.draw.circle(screen, HUB_OUTLINE_COLOR, pos, radius, 1)
        draw_zone_symbol(screen, pos, radius, hub.zone, invert_color(color))

        if show_labels:
            label = label_font.render(hub.name, True, TEXT_COLOR)
            label_x = pos[0] - label.get_width() // 2
            label_y = pos[1] - radius - label.get_height() - 2
            screen.blit(label, (label_x, label_y))


def main():
    drone_map = parser_file("maps/challenger/01_the_impossible_dream.txt")
    world_positions = compute_world_layout(drone_map.hubs)

    # --- calcul de la simulation, une seule fois avant la boucle graphique ---
    drone_paths, total_turns = schedule_drones(drone_map)
    SECONDS_PER_TURN = 0.6  # vitesse de l'animation : ajuster selon le rendu voulu

    pygame.init()
    width, height = WINDOW_W, WINDOW_H
    fullscreen = False
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
    pygame.display.set_caption("Fly-in — Map Viewer")
    hud_font = pygame.font.SysFont("consolas", 16, bold=True)
    clock = pygame.time.Clock()

    camera = Camera(width, height)
    sim_start_ticks = pygame.time.get_ticks()

    pending_size = None
    last_resize_event_ms = 0
    DEBOUNCE_MS = 80

    panning = False
    last_mouse_pos = None

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.VIDEORESIZE and not fullscreen:
                if event.w > 0 and event.h > 0:
                    pending_size = (event.w, event.h)
                    last_resize_event_ms = pygame.time.get_ticks()

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_F11:
                fullscreen = not fullscreen
                if fullscreen:
                    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                    width, height = screen.get_size()
                else:
                    width, height = WINDOW_W, WINDOW_H
                    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
                camera.resize(width, height)

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                camera.reset()

            elif event.type == pygame.MOUSEWHEEL:
                factor = ZOOM_STEP if event.y > 0 else (1 / ZOOM_STEP)
                camera.zoom_at(pygame.mouse.get_pos(), factor)

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                panning = True
                last_mouse_pos = event.pos

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                panning = False
                last_mouse_pos = None

            elif event.type == pygame.MOUSEMOTION and panning:
                dx = event.pos[0] - last_mouse_pos[0]
                dy = event.pos[1] - last_mouse_pos[1]
                camera.pan(dx, dy)
                last_mouse_pos = event.pos

        if pending_size and pygame.time.get_ticks() - last_resize_event_ms > DEBOUNCE_MS:
            width, height = pending_size
            screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
            camera.resize(width, height)
            pending_size = None

        draw_map(screen, drone_map, world_positions, camera)

        # --- dessin des drones, interpolés selon le temps de simulation écoulé ---
        elapsed_seconds = (pygame.time.get_ticks() - sim_start_ticks) / 1000
        sim_turn = elapsed_seconds / SECONDS_PER_TURN
        in_flight = 0
        for path in drone_paths:
            state = interpolate_drone_state(path, sim_turn, world_positions)
            if state is not None:
                world_pos, angle = state
                screen_pos = camera.world_to_screen(*world_pos)
                size = max(3, int(DRONE_SIZE * camera.zoom))
                draw_drone_arrow(screen, screen_pos, angle, size, DRONE_COLOR)
                in_flight += 1

        # --- HUD : compteurs drones et tours en fonction du temps ---
        current_turn = min(int(sim_turn), total_turns)
        delivered = sum(1 for p in drone_paths if sim_turn >= p[-1][1])
        hud_lines = [
            f"Drones: {len(drone_paths)}  en vol: {in_flight}  livres: {delivered}",
            f"Tour: {current_turn} / {total_turns}",
        ]
        for i, line in enumerate(hud_lines):
            surf = hud_font.render(line, True, TEXT_COLOR)
            screen.blit(surf, (12, 10 + i * 22))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
