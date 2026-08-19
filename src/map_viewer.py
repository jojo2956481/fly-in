import pygame


WINDOW_W, WINDOW_H = 1000, 700
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

UNIT_SCALE = 80   # 60 default
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

    def __init__(self, width, height):
        self.x = 0.0
        self.y = 0.0
        self.zoom = 1.0
        self.width = width
        self.height = height

    def resize(self, width, height):
        self.width = width
        self.height = height

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


# queue[(distance=0, tie_BREAKER=i++, hub=A)]

# distance, i, hub = queue.pop()

# hub.voisin

# (3, 1, voisin.b)
# (3, 2, voisin.c)

# queue[(3, B), (3, C)]

# distance, hub = queue.pop()

