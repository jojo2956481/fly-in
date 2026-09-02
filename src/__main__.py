from src.parsing_map import parser_file
# from src.algorithm import schedule_drones, decompose_drone_paths
from src.dijkstra_reservation_tab import schedule_drones
import argparse
import sys
import pygame
from src.map_viewer import (
    compute_world_layout, draw_map,
    WINDOW_W, WINDOW_H, FONT_SIZE, Camera, ZOOM_STEP
                )


def take_arg():
    parser = argparse.ArgumentParser()

    parser.add_argument("--map", default=None)
    parser.add_argument("--display")
    args = parser.parse_args()
    if args.map is None:
        print("Error no map selected")
        sys.exit(1)
    return args.map, args.display


DRONE_COLOR = (0, 140, 200)
DRONE_RADIUS = 5


def interpolate_drone_position(path, sim_turn, world_positions):
    if sim_turn < path[0][1] or sim_turn > path[-1][1]:
        return None

    for (hub_a, t_a), (hub_b, t_b) in zip(path, path[1:]):
        if t_a <= sim_turn <= t_b:
            if t_b == t_a:
                progress = 0.0
            else:
                progress = (sim_turn - t_a) / (t_b - t_a)
            xa, ya = world_positions[hub_a]
            xb, yb = world_positions[hub_b]
            return (xa + (xb - xa) * progress, ya + (yb - ya) * progress)

    return world_positions[path[-1][0]]


def display_interface(drone_map):
    world_positions = compute_world_layout(drone_map.hubs)
    drone_paths = []
    drone_paths, horizon = schedule_drones(drone_map)
    SECONDS_PER_TURN = 0.6
    pygame.init()
    width, height = WINDOW_W, WINDOW_H
    fullscreen = False
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
    pygame.display.set_caption("Fly-in — Map Viewer")
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
                print("here")
                if event.w > 0 and event.h > 0:
                    width, height = event.w, event.h
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
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                camera.reset()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
                camera.resize(width, height)
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
            print("resize applique ->", width, height)
        draw_map(screen, drone_map, world_positions, camera)
        elapsed_seconds = (pygame.time.get_ticks() - sim_start_ticks) / 1000
        sim_turn = elapsed_seconds / SECONDS_PER_TURN
        for path in drone_paths:
            world_pos = interpolate_drone_position(path, sim_turn, world_positions)
            if world_pos is not None:
                screen_pos = camera.world_to_screen(*world_pos)
                radius = max(2, int(DRONE_RADIUS * camera.zoom))
                pygame.draw.circle(screen, DRONE_COLOR, screen_pos, radius)

        pygame.display.flip()
        clock.tick(60)
    pygame.quit()


def display_data(drone_map):
    for hub in drone_map.hubs:
        print(
            f"{hub.name}, {hub.x}, {hub.y},"
            f"{hub.kind}, {hub.color}, {hub.zone},"
            f"{hub.max_drones}, {hub.neighbors}"
            )
    for conection in drone_map.connections:
        print(
            f"{conection.src}-{conection.dst}, {conection.max_link_capacity}"
        )


def main():
    path_map, display = take_arg()
    drone_map = parser_file(path_map)
    if display == "data":
        display_data(drone_map)
    else:
        display_interface(drone_map)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
