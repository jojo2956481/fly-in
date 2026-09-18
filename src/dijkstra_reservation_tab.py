
import heapq
from collections import defaultdict
from typing import Optional
from src.parsing_map import Connection, Map_format, Hub


def zone_cost(zone: str) -> Optional[int]:
    """return the cost of a 'blocked' or 'restricted' zone"""
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1


class ReservationTable:

    def __init__(self) -> None:
        self.hub_occupancy: dict[tuple[str, int], int] = defaultdict(int)
        self.link_usage: dict[tuple[str, str, int], int] = defaultdict(int)

    def hub_has_room(
        self, hub_name: str, turn: int, capacity: int
    ) -> bool:
        return self.hub_occupancy[(hub_name, turn)] < capacity

    def link_has_room(
        self, src: str, dst: str, turn: int, capacity: int
    ) -> bool:
        return self.link_usage[(src, dst, turn)] < capacity

    def reserve_path(self, path: list[tuple[str, int]]) -> None:
        for hub_name, turn in path:
            self.hub_occupancy[(hub_name, turn)] += 1
        for (src, t_src), (dst, t_dst) in zip(path, path[1:]):
            if src != dst:
                self.link_usage[(src, dst, t_src)] += 1


def relax(
    dist: dict[tuple[str, int], tuple[int, int]],
    parent: dict[tuple[str, int], Optional[tuple[str, int]]],
    pq: list[tuple[tuple[int, int], tuple[str, int]]],
    current_state: tuple[str, int],
    next_state: tuple[str, int],
    penalty: int,
) -> None:
    inf: tuple[int, int] = (10**9, 10**9)
    new_key: tuple[int, int] = (next_state[1], penalty)
    if new_key < dist.get(next_state, inf):
        dist[next_state] = new_key
        parent[next_state] = current_state
        heapq.heappush(pq, (new_key, next_state))


def _reconstruct(
    parent: dict[tuple[str, int], Optional[tuple[str, int]]],
    end_state: tuple[str, int],
) -> list[tuple[str, int]]:
    path: list[tuple[str, int]] = []
    state: Optional[tuple[str, int]] = end_state
    while state is not None:
        path.append(state)
        state = parent[state]
    path.reverse()
    return path


def _find_connection(drone_map: Map_format, src: str, dst: str) -> Connection:
    return next(
        c for c in drone_map.connections
        if {c.src, c.dst} == {src, dst}
    )


def dijkstra_spacetime(
    drone_map: Map_format,
    start_name: str,
    end_name: str,
    reservation: ReservationTable,
    max_horizon: int,
) -> Optional[list[tuple[str, int]]]:
    hub_by_name = {h.name: h for h in drone_map.hubs}
    start_state: tuple[str, int] = (start_name, 0)
    dist: dict[tuple[str, int], tuple[int, int]] = {start_state: (0, 0)}
    parent: dict[
        tuple[str, int],
        Optional[tuple[str, int]]] = {start_state: None}
    pq: list[tuple[tuple[int, int], tuple[str, int]]] = [((0, 0), start_state)]
    inf: tuple[int, int] = (10**9, 10**9)
    while pq:
        key, (hub_name, t) = heapq.heappop(pq)

        if hub_name == end_name:
            return _reconstruct(parent, (hub_name, t))

        if key > dist.get((hub_name, t), inf):
            continue
        if t >= max_horizon:
            continue
        hub = hub_by_name[hub_name]
        nt = t + 1
        if reservation.hub_has_room(hub_name, nt, hub.max_drones):
            relax(
                dist, parent, pq,
                (hub_name, t), (hub_name, nt), key[1],
            )
        for neighbor_name in hub.neighbors:
            neighbor = hub_by_name[neighbor_name]
            cost = zone_cost(neighbor.zone)
            if cost is None:
                continue
            nt = t + cost
            if nt > max_horizon:
                continue
            conn = _find_connection(drone_map, hub_name, neighbor_name)
            if not reservation.link_has_room(
                hub_name, neighbor_name, t, conn.max_link_capacity
            ):
                continue
            if not reservation.hub_has_room(
                neighbor_name, nt, neighbor.max_drones
            ):
                continue
            penalty = key[1] + (
                0 if neighbor.zone == "priority" else 1
            )
            relax(
                dist, parent, pq,
                (hub_name, t), (neighbor_name, nt), penalty,
            )
    return None


def annotate_path(
        path: list[tuple[str, int]],
        hub_by_name: dict[str, Hub]) -> list[tuple[str, int, str]]:
    if not path:
        return []
    annotated: list[tuple[str, int, str]] = [(path[0][0], path[0][1], "move")]
    for (hub_a, t_a), (hub_b, t_b) in zip(path, path[1:]):
        if hub_a == hub_b:
            annotated.append((hub_b, t_b, "wait"))
        elif t_b - t_a == 2 and hub_by_name[hub_b].zone == "restricted":
            name = f"{hub_a}-{hub_b}"
            annotated.append((name, t_a + 1, "transit"))
            annotated.append((hub_b, t_b, "move"))
        else:
            annotated.append((hub_b, t_b, "move"))
    return annotated


def schedule_drones(
    drone_map: Map_format,
    max_horizon: int = 200,
) -> tuple[list[list[tuple[str, int, str]]], int]:
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")
    hub_by_name = {h.name: h for h in drone_map.hubs}
    reservation = ReservationTable()
    drone_paths: list[list[tuple[str, int, str]]] = []

    for _ in range(drone_map.nb_drones):
        path = dijkstra_spacetime(
            drone_map, start.name, end.name,
            reservation, max_horizon,
        )
        if path is None:
            continue
        reservation.reserve_path(path)
        drone_paths.append(annotate_path(path, hub_by_name))

    horizon = max(
        (p[-1][1] for p in drone_paths), default=0
    )
    return drone_paths, horizon
