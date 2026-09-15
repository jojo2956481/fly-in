import heapq
from collections import defaultdict
from typing import Optional


# un état spatio-temporel : (nom_hub, tour)
State = tuple[str, int]
# clé de coût : (tour_arrivee, penalite_non_priority)
Key = tuple[int, int]
# étape enrichie d'un chemin : (nom, tour, etat)
Step = tuple[str, int, str]


def zone_cost(zone: str) -> Optional[int]:
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1


class ReservationTable:

    def __init__(self) -> None:
        self.hub_occupancy: dict[State, int] = defaultdict(int)
        self.link_usage: dict[tuple[str, str, int], int] = defaultdict(int)

    def hub_has_room(
        self, hub_name: str, turn: int, capacity: int
    ) -> bool:
        return self.hub_occupancy[(hub_name, turn)] < capacity

    def link_has_room(
        self, src: str, dst: str, turn: int, capacity: int
    ) -> bool:
        return self.link_usage[(src, dst, turn)] < capacity

    def reserve_path(self, path: list[State]) -> None:
        for hub_name, turn in path:
            self.hub_occupancy[(hub_name, turn)] += 1
        for (src, t_src), (dst, t_dst) in zip(path, path[1:]):
            if src != dst:
                self.link_usage[(src, dst, t_src)] += 1


def relax(
    dist: dict[State, Key],
    parent: dict[State, Optional[State]],
    pq: list[tuple[Key, State]],
    current_state: State,
    next_state: State,
    penalty: int,
) -> None:
    inf: Key = (10**9, 10**9)
    new_key: Key = (next_state[1], penalty)
    if new_key < dist.get(next_state, inf):
        dist[next_state] = new_key
        parent[next_state] = current_state
        heapq.heappush(pq, (new_key, next_state))


def _reconstruct(
    parent: dict[State, Optional[State]],
    end_state: State,
) -> list[State]:
    path: list[State] = []
    state: Optional[State] = end_state
    while state is not None:
        path.append(state)
        state = parent[state]
    path.reverse()
    return path


def _find_connection(drone_map, src: str, dst: str):
    return next(
        c for c in drone_map.connections
        if {c.src, c.dst} == {src, dst}
    )


def dijkstra_spacetime(
    drone_map,
    start_name: str,
    end_name: str,
    reservation: ReservationTable,
    max_horizon: int,
) -> Optional[list[State]]:
    hub_by_name = {h.name: h for h in drone_map.hubs}
    start_state: State = (start_name, 0)
    dist: dict[State, Key] = {start_state: (0, 0)}
    parent: dict[State, Optional[State]] = {start_state: None}
    pq: list[tuple[Key, State]] = [((0, 0), start_state)]
    inf: Key = (10**9, 10**9)

    while pq:
        key, (hub_name, t) = heapq.heappop(pq)

        if hub_name == end_name:
            return _reconstruct(parent, (hub_name, t))

        if key > dist.get((hub_name, t), inf):
            continue
        if t >= max_horizon:
            continue

        hub = hub_by_name[hub_name]

        # rester sur place
        nt = t + 1
        if reservation.hub_has_room(hub_name, nt, hub.max_drones):
            relax(
                dist, parent, pq,
                (hub_name, t), (hub_name, nt), key[1],
            )

        # se déplacer
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


def annotate_path(path: list[State], hub_by_name: dict) -> list[Step]:
    if not path:
        return []

    annotated: list[Step] = [(path[0][0], path[0][1], "move")]
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
    drone_map,
    max_horizon: int = 200,
) -> tuple[list[list[Step]], int]:
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")
    hub_by_name = {h.name: h for h in drone_map.hubs}
    reservation = ReservationTable()
    drone_paths: list[list[Step]] = []

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
