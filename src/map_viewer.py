import pygame
from src.parsing_map import Hub, Map_format


class Window:
    WIDTH: int = 2000
    HEIGHT: int = 1000
    MARGIN: int = 60


class Colors:
    BG: tuple[int, int, int] = (250, 240, 217)
    LINE: tuple[int, int, int] = (150, 140, 120)
    TEXT: tuple[int, int, int] = (60, 55, 45)
    HUB_OUTLINE: tuple[int, int, int] = (60, 55, 45)


class Style:
    LINE_WIDTH: int = 3
    HUB_RADIUS: int = 13
    FONT_NAME: str = "consolas"
    FONT_SIZE: int = 12
    FONT_SIZE_MIN: int = 18
    FONT_SIZE_MAX: int = 22


class Zoom:
    UNIT_SCALE: int = 80
    MIN: float = 0.15
    MAX: float = 6.0
    STEP: float = 1.15


COLOR_MAP: dict[str | None, tuple[int, int, int]] = {
    "green": (0, 200, 0),
    "red": (220, 30, 30),
    "purple": (150, 60, 200),
    "black": (10, 10, 10),
    "brown": (110, 70, 40),
    "orange": (255, 140, 0),
    "maroon": (128, 0, 0),
    "gold": (212, 175, 55),
    "darkred": (139, 0, 0),
    "violet": (150, 80, 220),
    "crimson": (200, 20, 60),
    "rainbow": (255, 255, 255),
    None: (180, 180, 180),
}


_font_cache: dict[int, pygame.font.Font] = {}


def get_font(size: int) -> pygame.font.Font:
    size = int(size)
    if size not in _font_cache:
        _font_cache[size] = pygame.font.SysFont(
            Style.FONT_NAME, size, bold=True
        )
    return _font_cache[size]


def compute_world_layout(
    hubs: list[Hub],
    unit_scale: int = Zoom.UNIT_SCALE,
) -> dict[str, tuple[float, float]]:
    return {
        h.name: (h.x * unit_scale, -h.y * unit_scale)
        for h in hubs
    }


class Camera:

    def __init__(
        self,
        width: int,
        height: int,
        max_x: int,
        max_y: int,
        min_x: int,
        min_y: int,
    ) -> None:
        self.x: float = ((max_x + min_x) / 2) * Zoom.UNIT_SCALE
        self.y: float = -((max_y + min_y) / 2) * Zoom.UNIT_SCALE
        self.width: int = width
        self.height: int = height
        margin: int = 50
        map_width: int = (max_x - min_x) * Zoom.UNIT_SCALE
        map_height: int = (max_y - min_y) * Zoom.UNIT_SCALE
        zoom_x: float = width / (map_width + margin)
        zoom_y: float = height / (map_height + margin)
        self.dep_zoom: float = min(zoom_x, zoom_y)
        self.zoom: float = self.dep_zoom
        self.max_y: int = max_y
        self.max_x: int = max_x
        self.min_y: int = min_y
        self.min_x: int = min_x

    def resize(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

    def reset(self) -> None:
        self.x = ((self.max_x + self.min_x) / 2) * Zoom.UNIT_SCALE
        self.y = -((self.max_y + self.min_y) / 2) * Zoom.UNIT_SCALE
        self.zoom = self.dep_zoom

    def world_to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        sx: float = (wx - self.x) * self.zoom + self.width / 2
        sy: float = (wy - self.y) * self.zoom + self.height / 2
        return int(sx), int(sy)

    def screen_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        wx: float = (sx - self.width / 2) / self.zoom + self.x
        wy: float = (sy - self.height / 2) / self.zoom + self.y
        return wx, wy

    def zoom_at(
        self,
        screen_pos: tuple[int, int],
        factor: float,
    ) -> None:
        world_before = self.screen_to_world(*screen_pos)
        self.zoom = max(Zoom.MIN, min(Zoom.MAX, self.zoom * factor))
        world_after = self.screen_to_world(*screen_pos)
        self.x += world_before[0] - world_after[0]
        self.y += world_before[1] - world_after[1]

    def pan(self, dx: float, dy: float) -> None:
        self.x -= dx / self.zoom
        self.y -= dy / self.zoom


def invert_color(
    color: tuple[int, int, int],
) -> tuple[int, int, int]:
    r, g, b = color[:3]
    return (255 - r, 255 - g, 255 - b)


def draw_zone_symbol(
    screen: pygame.Surface,
    pos: tuple[int, int],
    radius: int,
    zone: str,
    symbol_color: tuple[int, int, int],
) -> None:
    x, y = pos
    thickness: int = max(1, int(radius / 6))

    if zone == "blocked":
        s = radius * 0.5
        pygame.draw.line(
            screen, symbol_color,
            (x - s, y - s), (x + s, y + s), thickness,
        )
        pygame.draw.line(
            screen, symbol_color,
            (x - s, y + s), (x + s, y - s), thickness,
        )

    elif zone == "restricted":
        s = radius * 0.6
        points = [
            (x, y - s),
            (x - s, y + s * 0.7),
            (x + s, y + s * 0.7),
        ]
        pygame.draw.polygon(screen, symbol_color, points, thickness)

    elif zone == "priority":
        s = radius * 0.5
        rect = pygame.Rect(x - s, y - s, s * 2, s * 2)
        pygame.draw.rect(screen, symbol_color, rect, thickness)


def draw_map(
    screen: pygame.Surface,
    drone_map: Map_format,
    world_positions: dict[str, tuple[float, float]],
    camera: Camera,
) -> None:
    screen.fill(Colors.BG)
    for conn in drone_map.connections:
        p1 = camera.world_to_screen(*world_positions[conn.src])
        p2 = camera.world_to_screen(*world_positions[conn.dst])
        pygame.draw.line(screen, Colors.LINE, p1, p2, Style.LINE_WIDTH)

    label_font_size: int = max(
        Style.FONT_SIZE_MIN,
        min(Style.FONT_SIZE_MAX, int(Style.FONT_SIZE * camera.zoom)),
    )
    show_labels: bool = (
        Style.FONT_SIZE * camera.zoom
    ) >= Style.FONT_SIZE_MIN
    label_font = get_font(label_font_size) if show_labels else None

    for hub in drone_map.hubs:
        pos = camera.world_to_screen(*world_positions[hub.name])
        color = COLOR_MAP.get(hub.color, COLOR_MAP[None])
        bonus: int = 4 if hub.kind in ("start", "end") else 0
        radius: int = max(
            2, int((Style.HUB_RADIUS + bonus) * camera.zoom)
        )
        pygame.draw.circle(screen, color, pos, radius)
        pygame.draw.circle(screen, Colors.HUB_OUTLINE, pos, radius, 1)
        draw_zone_symbol(
            screen, pos, radius, hub.zone, invert_color(color)
        )

        if show_labels and label_font is not None:
            label = label_font.render(hub.name, True, Colors.TEXT)
            label_x: int = pos[0] - label.get_width() // 2
            label_y: int = pos[1] - radius - label.get_height() - 2
            screen.blit(label, (label_x, label_y))


def window_controle_info(screen: pygame.Surface) -> None:
    width, height = screen.get_size()
    window_width: int = 300
    window_height: int = 200
    margin_y: int = 20
    margin_x: int = 150
    window = pygame.Rect(
        width - window_width - margin_x,
        margin_y,
        window_width,
        window_height,
    )

    font = pygame.font.Font(None, 36)
    lines: list[str] = [
        "---Informations---",
        "Press Esc to close",
        "Press c to refocus",
        "Press <- and -> go manual mode",
        "Press escape to quit manual mode",
        "Press r to restart simulation",
    ]
    for i, line in enumerate(lines):
        text = font.render(line, True, (0, 0, 0))
        screen.blit(text, (window.x + 20, window.y + 20 + i * 60))


def window_simu_info(
        screen: pygame.Surface, nb_drones: int, elapse: int,
        horizon: int) -> None:
    window_width: int = 300
    window_height: int = 200
    margin: int = 20
    window = pygame.Rect(
        margin, margin, window_width, window_height
    )

    if elapse >= horizon:
        elapse = horizon

    font = pygame.font.Font(None, 36)
    lines: list[str] = [
        "----Simulation----",
        f"drone : {nb_drones}",
        f"tours : {elapse}",
    ]
    for i, line in enumerate(lines):
        text = font.render(line, True, (0, 0, 0))
        screen.blit(text, (window.x + 20, window.y + 20 + i * 60))
