import pygame


WINDOW_W, WINDOW_H = 2000, 1000
MARGIN = 60
BG_COLOR = (250, 240, 217)
LINE_COLOR = (150, 140, 120)
TEXT_COLOR = (60, 55, 45)
HUB_OUTLINE_COLOR = (60, 55, 45)
LINE_WIDTH = 3
HUB_RADIUS = 13
FONT_NAME = "consolas"
FONT_SIZE = 12
FONT_SIZE_MIN = 18
FONT_SIZE_MAX = 22

UNIT_SCALE = 80
ZOOM_MIN, ZOOM_MAX = 0.15, 6.0
ZOOM_STEP = 1.15


_font_cache = {}


def get_font(size):
    size = int(size)
    if size not in _font_cache:
        _font_cache[size] = pygame.font.SysFont(FONT_NAME, size, bold=True)
    return _font_cache[size]


COLOR_MAP = {
    "green": (0, 200, 0), "red": (220, 30, 30), "purple": (150, 60, 200),
    "black": (10, 10, 10), "brown": (110, 70, 40), "orange": (255, 140, 0),
    "maroon": (128, 0, 0), "gold": (212, 175, 55), "darkred": (139, 0, 0),
    "violet": (150, 80, 220), "crimson": (200, 20, 60),
    "rainbow": (255, 255, 255),
    None: (180, 180, 180),
}


def compute_world_layout(hubs, unit_scale=UNIT_SCALE):
    return {
        h.name: (h.x * unit_scale, -h.y * unit_scale)
        for h in hubs
    }


class Camera:

    def __init__(self, width, height, max_x, max_y, min_x, min_y):
        self.x = (((max_x - min_x) / 2) + min_x) * UNIT_SCALE
        self.y = (((max_y - min_y) / 2) + min_y) * UNIT_SCALE
        self.width = width
        self.height = height
        margin = 50
        map_width = (max_x - min_x) * UNIT_SCALE
        map_height = (max_y - min_y) * UNIT_SCALE
        zoom_x = width / (map_width + margin)
        zoom_y = height / (map_height + margin)
        self.dep_zoom = min(zoom_x, zoom_y)
        self.zoom = self.dep_zoom
        self.max_y = max_y
        self.max_x = max_x
        self.min_y = min_y
        self.min_x = min_x

    def resize(self, width, height):
        self.width = width
        self.height = height

    def reset(self):
        self.x = (((self.max_x - self.min_x) / 2) + self.min_x) * UNIT_SCALE
        self.y = (((self.max_y - self.min_y) / 2) + self.min_y) * UNIT_SCALE
        self.zoom = self.dep_zoom

    def world_to_screen(self, wx, wy):
        sx = (wx - self.x) * self.zoom + self.width / 2
        sy = (wy - self.y) * self.zoom + self.height / 2
        return int(sx), int(sy)

    def screen_to_world(self, sx, sy):
        wx = (sx - self.width / 2) / self.zoom + self.x
        wy = (sy - self.height / 2) / self.zoom + self.y
        return wx, wy

    def zoom_at(self, screen_pos, factor):
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


def draw_zone_symbol(screen, pos, radius, zone, symbol_color):
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


def draw_map(screen, drone_map, world_positions, camera):
    screen.fill(BG_COLOR)
    # hub_by_name = {h.name: h for h in drone_map.hubs}
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


def window_controle_info(screen):
    width, height = screen.get_size()
    overlay = pygame.Surface((width, height), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 0))
    screen.blit(overlay, (0, 0))
    window_width = 300
    window_height = 200
    margin = 20
    window = pygame.Rect(
        width - window_width - margin,
        margin,
        window_width,
        window_height
    )
    window_surface = pygame.Surface(
        (window_width, window_height),
        pygame.SRCALPHA
    )
    window_surface.fill((0, 0, 0, 0))
    pygame.draw.rect(
        window_surface,
        (255, 255, 255, 255),
        window, 2,
        border_radius=10
    )

    font = pygame.font.Font(None, 36)

    title = font.render("Informations", True, (0, 0, 0))
    screen.blit(title, (window.x + 20, window.y + 20))

    text = font.render("Press Esc to close", True, (0, 0, 0))
    screen.blit(text, (window.x + 20, window.y + 80))

    text = font.render("Press c to refocus", True, (0, 0, 0))
    screen.blit(text, (window.x + 20, window.y + 140))


def window_simu_info(screen, nb_drones, horizon):
    width, height = screen.get_size()
    overlay = pygame.Surface((width, height), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 0))
    screen.blit(overlay, (0, 0))
    window_width = 300
    window_height = 200
    margin = 20
    window = pygame.Rect(
        margin,
        margin,
        window_width,
        window_height
    )
    window_surface = pygame.Surface(
        (window_width, window_height),
        pygame.SRCALPHA
    )
    window_surface.fill((0, 0, 0, 0))
    pygame.draw.rect(
        window_surface,
        (255, 255, 255, 255),
        window, 2,
        border_radius=10
    )
    font = pygame.font.Font(None, 36)

    title = font.render("Simulation", True, (0, 0, 0))
    screen.blit(title, (window.x + 20, window.y + 20))

    text = font.render(f"drone : {nb_drones}", True, (0, 0, 0))
    screen.blit(text, (window.x + 20, window.y + 80))

    text = font.render(f"tours : {horizon}", True, (0, 0, 0))
    screen.blit(text, (window.x + 20, window.y + 140))


