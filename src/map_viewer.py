from typing import Optional

import pygame

from src.parsing_map import Map_format
from src.dijkstra_reservation_tab import Scheduler, Step


Position = tuple[float, float]
Color = tuple[int, int, int]

class Config:
    WINDOW_W: int = 1400
    WINDOW_H: int = 800
    UNIT_SCALE: int = 80
    ZOOM_MIN: float = 0.15
    ZOOM_MAX: float = 6.0
    ZOOM_STEP: float = 1.15
    LINE_WIDTH: int = 3
    HUB_RADIUS: int = 13
    FONT_NAME: str = "consolas"
    FONT_SIZE_MIN: int = 18
    FONT_SIZE_MAX: int = 22
    DRONE_RADIUS: int = 5
    SECONDS_PER_TURN: float = 0.6
    DEBOUNCE_MS: int = 80


class Palette:
    BG: Color = (250, 240, 217)
    LINE: Color = (150, 140, 120)
    TEXT: Color = (60, 55, 45)
    HUB_OUTLINE: Color = (60, 55, 45)
    DRONE: Color = (0, 140, 200)
    ZONES: dict[Optional[str], Color] = {
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


class Camera:
    """Gère le zoom et le déplacement (pan), et convertit monde -> écran."""

    def __init__(
        self,
        width: int,
        height: int,
        max_x: int,
        max_y: int,
        min_x: int,
        min_y: int,
    ) -> None:
        self.width = width
        self.height = height
        self.max_x, self.max_y = max_x, max_y
        self.min_x, self.min_y = min_x, min_y
        self.x: float = ((max_x + min_x) / 2) * Config.UNIT_SCALE
        self.y: float = -((max_y + min_y) / 2) * Config.UNIT_SCALE
        margin = min(width, height) * 0.10
        map_w = (max_x - min_x) * Config.UNIT_SCALE
        map_h = (max_y - min_y) * Config.UNIT_SCALE
        self.zoom_x: float = width / (map_w + margin)
        self.zoom_y: float = height / (map_h + margin)
        self.dep_zoom: float = min(self.zoom_x, self.zoom_y)
        self.zoom: float = self.dep_zoom

    def resize(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

    def reset(self) -> None:
        self.x = ((self.max_x + self.min_x) / 2) * Config.UNIT_SCALE
        self.y = -((self.max_y + self.min_y) / 2) * Config.UNIT_SCALE
        self.zoom = self.dep_zoom

    def world_to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        sx = (wx - self.x) * self.zoom + self.width / 2
        sy = (wy - self.y) * self.zoom + self.height / 2
        return int(sx), int(sy)

    def screen_to_world(self, sx: float, sy: float) -> Position:
        wx = (sx - self.width / 2) / self.zoom + self.x
        wy = (sy - self.height / 2) / self.zoom + self.y
        return wx, wy

    def zoom_at(
        self, screen_pos: tuple[int, int], factor: float
    ) -> None:
        before = self.screen_to_world(*screen_pos)
        self.zoom = max(
            Config.ZOOM_MIN, min(Config.ZOOM_MAX, self.zoom * factor)
        )
        after = self.screen_to_world(*screen_pos)
        self.x += before[0] - after[0]
        self.y += before[1] - after[1]

    def pan(self, dx: float, dy: float) -> None:
        self.x -= dx / self.zoom
        self.y -= dy / self.zoom


class Viewer:
    """Fenêtre Pygame : affiche la map et anime les drones tour par tour,
    avec zoom, pan, plein écran et mode pas-à-pas."""

    def __init__(self, drone_map: Map_format) -> None:
        self.drone_map = drone_map
        self.world_positions: dict[str, Position] = {
            h.name: (h.x * Config.UNIT_SCALE, -h.y * Config.UNIT_SCALE)
            for h in drone_map.hubs
        }
        self.drone_paths, self.horizon = Scheduler(drone_map).run()
        if not self.drone_paths:
            raise ValueError("The map is impossible to solve")
        self._font_cache: dict[int, pygame.font.Font] = {}

        self.width = Config.WINDOW_W
        self.height = Config.WINDOW_H
        self.fullscreen = False
        self.manual_mode = False
        self.manual_turn = 0
        self.sim_turn: float = 0.0
        self.panning = False
        self.last_mouse_pos: Optional[tuple[int, int]] = None
        self.pending_size: Optional[tuple[int, int]] = None
        self.last_resize_ms = 0
        self.running = True

    # --- rendu ---

    def _get_font(self, size: int) -> pygame.font.Font:
        if size not in self._font_cache:
            self._font_cache[size] = pygame.font.SysFont(
                Config.FONT_NAME, size, bold=True
            )
        return self._font_cache[size]

    @staticmethod
    def _invert(color: Color) -> Color:
        r, g, b = color
        return (255 - r, 255 - g, 255 - b)

    def _resolve_position(self, name: str) -> Position:
        if name in self.world_positions:
            return self.world_positions[name]
        src, dst = name.split("-", 1)
        xa, ya = self.world_positions[src]
        xb, yb = self.world_positions[dst]
        return ((xa + xb) / 2, (ya + yb) / 2)

    def _interpolate(self, path: list[Step]) -> Optional[Position]:
        if (
            self.sim_turn < path[0][1]
            or self.sim_turn > path[-1][1]
        ):
            return None
        for (name_a, t_a, _), (name_b, t_b, _) in zip(
            path, path[1:]
        ):
            if t_a <= self.sim_turn <= t_b:
                if t_b == t_a:
                    prog = 0.0
                else:
                    prog = (self.sim_turn - t_a) / (t_b - t_a)
                xa, ya = self._resolve_position(name_a)
                xb, yb = self._resolve_position(name_b)
                return (
                    xa + (xb - xa) * prog,
                    ya + (yb - ya) * prog,
                )
        return self._resolve_position(path[-1][0])

    def _draw_zone_symbol(
        self,
        pos: tuple[int, int],
        radius: int,
        zone: str,
        color: Color,
    ) -> None:
        x, y = pos
        thick = max(1, int(radius / 6))
        if zone == "blocked":
            s = radius * 0.5
            pygame.draw.line(
                self.screen, color,
                (x - s, y - s), (x + s, y + s), thick,
            )
            pygame.draw.line(
                self.screen, color,
                (x - s, y + s), (x + s, y - s), thick,
            )
        elif zone == "restricted":
            s = radius * 0.6
            pts = [
                (x, y - s),
                (x - s, y + s * 0.7),
                (x + s, y + s * 0.7),
            ]
            pygame.draw.polygon(self.screen, color, pts, thick)
        elif zone == "priority":
            s = radius * 0.5
            rect = pygame.Rect(x - s, y - s, s * 2, s * 2)
            pygame.draw.rect(self.screen, color, rect, thick)

    def _draw_map(self) -> None:
        self.screen.fill(Palette.BG)
        for conn in self.drone_map.connections:
            p1 = self.camera.world_to_screen(
                *self.world_positions[conn.src]
            )
            p2 = self.camera.world_to_screen(
                *self.world_positions[conn.dst]
            )
            pygame.draw.line(
                self.screen, Palette.LINE, p1, p2, Config.LINE_WIDTH
            )

        base = 12
        size = max(
            Config.FONT_SIZE_MIN,
            min(Config.FONT_SIZE_MAX, int(base * self.camera.zoom)),
        )
        show = (base * self.camera.zoom) >= Config.FONT_SIZE_MIN
        font = self._get_font(size) if show else None

        for hub in self.drone_map.hubs:
            pos = self.camera.world_to_screen(
                *self.world_positions[hub.name]
            )
            color = Palette.ZONES.get(
                hub.color, Palette.ZONES[None]
            )
            bonus = 4 if hub.kind in ("start", "end") else 0
            radius = max(
                2, int((Config.HUB_RADIUS + bonus) * self.camera.zoom)
            )
            pygame.draw.circle(self.screen, color, pos, radius)
            pygame.draw.circle(
                self.screen, Palette.HUB_OUTLINE, pos, radius, 1
            )
            self._draw_zone_symbol(
                pos, radius, hub.zone, self._invert(color)
            )
            if show and font is not None:
                label = font.render(hub.name, True, Palette.TEXT)
                lx = pos[0] - label.get_width() // 2
                ly = pos[1] - radius - label.get_height() - 2
                self.screen.blit(label, (lx, ly))

    def _draw_drones(self) -> None:
        for path in self.drone_paths:
            world_pos = self._interpolate(path)
            if world_pos is not None:
                screen_pos = self.camera.world_to_screen(*world_pos)
                radius = max(
                    2, int(Config.DRONE_RADIUS * self.camera.zoom)
                )
                pygame.draw.circle(
                    self.screen, Palette.DRONE, screen_pos, radius
                )

    def _draw_hud(self) -> None:
        font = pygame.font.Font(None, 28)
        turn = min(int(self.sim_turn), self.horizon)
        lines = [
            "----Simulation----",
            f"drone : {self.drone_map.nb_drones}",
            f"tours : {turn} / {self.horizon}",
        ]
        for i, line in enumerate(lines):
            text = font.render(line, True, (0, 0, 0))
            self.screen.blit(text, (20, 20 + i * 40))

    def _draw_info(self) -> None:
        width, height = self.screen.get_size()
        font = pygame.font.Font(None, 28)
        lines = [
            "----Simulation----",
           "Press Esc to close",
            "Press c to refocus",
            "Press <- and -> go manual mode",
            "Press escape to quit manual mode",
            "Press r to restart simulation",
        ]
        window_width = 340
        window_height = 30 + len(lines) * 52
        window = pygame.Rect(
        width - window_width - 20, 20,
        window_width, window_height
        )
        for i, line in enumerate(lines):
            text = font.render(line, True, (0, 0, 0))
            self.screen.blit(text, (window.x + 5, window.y + 5 + i * 40))
      

    # --- événements ---

    def _handle_key(self, key: int) -> None:
        if key == pygame.K_F11:
            self._toggle_fullscreen()
        elif key == pygame.K_c:
            self.camera.reset()
        elif key == pygame.K_r:
            self.manual_mode = False
            self.manual_turn = 0
            self.sim_start_ticks = pygame.time.get_ticks()
        elif key == pygame.K_RIGHT:
            self._step(+1)
        elif key == pygame.K_LEFT:
            self._step(-1)
        elif key == pygame.K_SPACE:
            self.manual_mode = False
            offset = int(
                self.manual_turn * Config.SECONDS_PER_TURN * 1000
            )
            self.sim_start_ticks = pygame.time.get_ticks() - offset
        elif key == pygame.K_ESCAPE:
            self.running = False

    def _step(self, delta: int) -> None:
        if not self.manual_mode:
            self.manual_mode = True
            self.manual_turn = int(self.sim_turn)
        self.manual_turn = max(
            0, min(self.manual_turn + delta, self.horizon)
        )

    def _toggle_fullscreen(self) -> None:
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            self.screen = pygame.display.set_mode(
                (0, 0), pygame.FULLSCREEN
            )
            self.width, self.height = self.screen.get_size()
        else:
            self.width = Config.WINDOW_W
            self.height = Config.WINDOW_H
            self.screen = pygame.display.set_mode(
                (self.width, self.height), pygame.RESIZABLE
            )
        self.camera.resize(self.width, self.height)

    def _handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            self._handle_key(event.key)
        elif event.type == pygame.VIDEORESIZE and not self.fullscreen:
            if event.w > 0 and event.h > 0:
                self.pending_size = (event.w, event.h)
                self.last_resize_ms = pygame.time.get_ticks()
        elif event.type == pygame.MOUSEWHEEL:
            factor = (
                Config.ZOOM_STEP if event.y > 0
                else 1 / Config.ZOOM_STEP
            )
            self.camera.zoom_at(pygame.mouse.get_pos(), factor)
        elif (
            event.type == pygame.MOUSEBUTTONDOWN
            and event.button == 1
        ):
            self.panning = True
            self.last_mouse_pos = event.pos
        elif (
            event.type == pygame.MOUSEBUTTONUP and event.button == 1
        ):
            self.panning = False
            self.last_mouse_pos = None
        elif (
            event.type == pygame.MOUSEMOTION
            and self.panning
            and self.last_mouse_pos is not None
        ):
            dx = event.pos[0] - self.last_mouse_pos[0]
            dy = event.pos[1] - self.last_mouse_pos[1]
            self.camera.pan(dx, dy)
            self.last_mouse_pos = event.pos

    def _apply_pending_resize(self) -> None:
        if self.pending_size is None:
            return
        now = pygame.time.get_ticks()
        if now - self.last_resize_ms > Config.DEBOUNCE_MS:
            self.width, self.height = self.pending_size
            self.screen = pygame.display.set_mode(
                (self.width, self.height), pygame.RESIZABLE
            )
            self.camera.resize(self.width, self.height)
            self.pending_size = None

    def _update_sim_turn(self) -> None:
        if self.manual_mode:
            self.sim_turn = float(self.manual_turn)
        else:
            elapsed = (
                pygame.time.get_ticks() - self.sim_start_ticks
            ) / 1000
            self.sim_turn = elapsed / Config.SECONDS_PER_TURN

    # --- boucle principale ---

    def run(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.width, self.height), pygame.RESIZABLE
        )
        pygame.display.set_caption("Fly-in — Map Viewer")
        clock = pygame.time.Clock()

        xs = [h.x for h in self.drone_map.hubs]
        ys = [h.y for h in self.drone_map.hubs]
        self.camera = Camera(
            self.width, self.height,
            max(xs), max(ys), min(xs), min(ys),
        )
        self.sim_start_ticks = pygame.time.get_ticks()

        while self.running:
            for event in pygame.event.get():
                self._handle_event(event)
            self._apply_pending_resize()
            self._update_sim_turn()

            self._draw_map()
            self._draw_drones()
            self._draw_hud()
            self._draw_info()

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()
