from src.parsing_map import parser_file
from src.dijkstra_reservation_tab import schedule_drones
import argparse
import sys
import pygame
import copy
from src.map_viewer import (
    compute_world_layout, draw_map, window_controle_info, window_simu_info,
    WINDOW_W, WINDOW_H, FONT_SIZE, Camera, ZOOM_STEP
                )


def take_arg():
    parser = argparse.ArgumentParser()

    parser.add_argument("--map", default=None)
    parser.add_argument("--capacity-info", action="store_true")
    args = parser.parse_args()
    if args.map is None:
        print("Error no map selected")
        sys.exit(1)
    return args.map, args.capacity_info


DRONE_COLOR = (0, 140, 200)
DRONE_RADIUS = 5


def resolve_position(name, world_positions):
    """Position monde d'une étape : hub direct, ou point milieu si c'est
    une connexion 'src-dst' (tour de transit)."""
    if name in world_positions:
        return world_positions[name]
    src, dst = name.split("-", 1)
    xa, ya = world_positions[src]
    xb, yb = world_positions[dst]
    return ((xa + xb) / 2, (ya + yb) / 2)


def interpolate_drone_position(path, sim_turn, world_positions):
    if sim_turn < path[0][1] or sim_turn > path[-1][1]:
        return None
    for (name_a, t_a, _), (name_b, t_b, _) in zip(path, path[1:]):
        if t_a <= sim_turn <= t_b:
            if t_b == t_a:
                progress = 0.0
            else:
                progress = (sim_turn - t_a) / (t_b - t_a)
            xa, ya = resolve_position(name_a, world_positions)
            xb, yb = resolve_position(name_b, world_positions)
            return (xa + (xb - xa) * progress, ya + (yb - ya) * progress)

    return resolve_position(path[-1][0], world_positions)


def display_interface(drone_map, capacity):
    world_positions = compute_world_layout(drone_map.hubs)
    drone_paths = []
    drone_paths, horizon = schedule_drones(drone_map)
    if not drone_paths:
        raise ValueError("The map is impossible to solve")
    SECONDS_PER_TURN = 0.6
    pygame.init()
    width, height = WINDOW_W, WINDOW_H
    fullscreen = False
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
    pygame.display.set_caption("Fly-in — Map Viewer")
    clock = pygame.time.Clock()
    max_x = max([hub.x for hub in drone_map.hubs])
    max_y = max([hub.y for hub in drone_map.hubs])
    min_x = min([hub.x for hub in drone_map.hubs])
    min_y = min([hub.y for hub in drone_map.hubs])
    camera = Camera(width, height, max_x, max_y, min_x, min_y)
    sim_start_ticks = pygame.time.get_ticks()
    manual_mode = False
    manual_turn = 0
    sim_turn = 0.0
    pending_size = None
    last_resize_event_ms = 0
    DEBOUNCE_MS = 80
    panning = False
    last_mouse_pos = None
    running = True
    temp = -1
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE and not fullscreen:
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
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                manual_mode = False
                manual_turn = 0
                sim_start_ticks = pygame.time.get_ticks()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                if not manual_mode:
                    manual_mode = True
                    manual_turn = int(sim_turn)
                manual_turn = min(manual_turn + 1, horizon)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                if not manual_mode:
                    manual_mode = True
                    manual_turn = int(sim_turn)
                manual_turn = max(manual_turn - 1, 0)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                manual_mode = False
                sim_start_ticks = pygame.time.get_ticks() - int(manual_turn * SECONDS_PER_TURN * 1000)
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
        draw_map(screen, drone_map, world_positions, camera)
        if manual_mode:
            sim_turn = float(manual_turn)
        else:
            elapsed_seconds = (pygame.time.get_ticks() - sim_start_ticks) / 1000
            sim_turn = elapsed_seconds / SECONDS_PER_TURN
        for path in drone_paths:
            world_pos = interpolate_drone_position(path, sim_turn, world_positions)
            if world_pos is not None:
                screen_pos = camera.world_to_screen(*world_pos)
                radius = max(2, int(DRONE_RADIUS * camera.zoom))
                pygame.draw.circle(screen, DRONE_COLOR, screen_pos, radius)
        window_controle_info(screen)
        if manual_mode:
            pass
        id = 1
        dis = ""
        for path in drone_paths:
            result = next((x for x in path if x[1] == int(sim_turn + 1)), None)
            if result:
                dis += f"D{id}-{result[0]} "
            id = id + 1
        if temp < int(sim_turn) and sim_turn < horizon + 1:
            print(dis)
        temp = int(sim_turn)
        window_simu_info(screen, drone_map.nb_drones, int(sim_turn), horizon)
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()


def main():
    path_map, capacity = take_arg()
    drone_map = parser_file(path_map)
    display_interface(drone_map, capacity)


if __name__ == "__main__":
    try:
        main()
    except FileExistsError as e:
        print(e)
