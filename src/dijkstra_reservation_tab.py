
import heapq
from collections import defaultdict
from typing import Optional


State = tuple[str, int]
Key = tuple[int, int]
Step = tuple[str, int, str]

INF: Key = (10**9, 10**9)


def zone_cost(zone: str) -> Optional[int]:
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1


class ReservationTable:
    """Mémorise les créneaux (hub, tour) et (lien, tour) déjà réservés,
    pour faire respecter les capacités entre drones successifs."""

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


class Scheduler:
    """Planifie le déplacement de tous les drones, un par un, via un
    Dijkstra spatio-temporel couplé à une table de réservation."""

    def __init__(self, drone_map, max_horizon: int = 200) -> None:
        self.drone_map = drone_map
        self.max_horizon = max_horizon
        self.hub_by_name = {h.name: h for h in drone_map.hubs}
        self.reservation = ReservationTable()

    def _find_connection(self, src: str, dst: str):
        return next(
            c for c in self.drone_map.connections
            if {c.src, c.dst} == {src, dst}
        )

    def _relax(
        self,
        dist: dict[State, Key],
        parent: dict[State, Optional[State]],
        pq: list[tuple[Key, State]],
        current_state: State,
        next_state: State,
        penalty: int,
    ) -> None:
        new_key: Key = (next_state[1], penalty)
        if new_key < dist.get(next_state, INF):
            dist[next_state] = new_key
            parent[next_state] = current_state
            heapq.heappush(pq, (new_key, next_state))

    def _dijkstra(
        self, start_name: str, end_name: str
    ) -> Optional[list[State]]:
        start_state: State = (start_name, 0)
        dist: dict[State, Key] = {start_state: (0, 0)}
        parent: dict[State, Optional[State]] = {start_state: None}
        pq: list[tuple[Key, State]] = [((0, 0), start_state)]

        while pq:
            key, (hub_name, t) = heapq.heappop(pq)

            if hub_name == end_name:
                return self._reconstruct(parent, (hub_name, t))

            if key > dist.get((hub_name, t), INF):
                continue
            if t >= self.max_horizon:
                continue

            self._expand(dist, parent, pq, key, hub_name, t)

        return None

    def _expand(
        self,
        dist: dict[State, Key],
        parent: dict[State, Optional[State]],
        pq: list[tuple[Key, State]],
        key: Key,
        hub_name: str,
        t: int,
    ) -> None:
        hub = self.hub_by_name[hub_name]

        # rester sur place
        nt = t + 1
        if self.reservation.hub_has_room(hub_name, nt, hub.max_drones):
            self._relax(
                dist, parent, pq,
                (hub_name, t), (hub_name, nt), key[1],
            )

        # se déplacer vers chaque voisin
        for neighbor_name in hub.neighbors:
            neighbor = self.hub_by_name[neighbor_name]
            cost = zone_cost(neighbor.zone)
            if cost is None:
                continue
            nt = t + cost
            if nt > self.max_horizon:
                continue
            conn = self._find_connection(hub_name, neighbor_name)
            if not self.reservation.link_has_room(
                hub_name, neighbor_name, t, conn.max_link_capacity
            ):
                continue
            if not self.reservation.hub_has_room(
                neighbor_name, nt, neighbor.max_drones
            ):
                continue
            penalty = key[1] + (
                0 if neighbor.zone == "priority" else 1
            )
            self._relax(
                dist, parent, pq,
                (hub_name, t), (neighbor_name, nt), penalty,
            )

    @staticmethod
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

    def _annotate(self, path: list[State]) -> list[Step]:
        if not path:
            return []
        annotated: list[Step] = [(path[0][0], path[0][1], "move")]
        for (hub_a, t_a), (hub_b, t_b) in zip(path, path[1:]):
            if hub_a == hub_b:
                annotated.append((hub_b, t_b, "wait"))
            elif (
                t_b - t_a == 2
                and self.hub_by_name[hub_b].zone == "restricted"
            ):
                name = f"{hub_a}-{hub_b}"
                annotated.append((name, t_a + 1, "transit"))
                annotated.append((hub_b, t_b, "move"))
            else:
                annotated.append((hub_b, t_b, "move"))
        return annotated

    def run(self) -> tuple[list[list[Step]], int]:
        start = next(
            h for h in self.drone_map.hubs if h.kind == "start"
        )
        end = next(
            h for h in self.drone_map.hubs if h.kind == "end"
        )
        drone_paths: list[list[Step]] = []

        for _ in range(self.drone_map.nb_drones):
            path = self._dijkstra(start.name, end.name)
            if path is None:
                continue
            self.reservation.reserve_path(path)
            drone_paths.append(self._annotate(path))

        horizon = max(
            (p[-1][1] for p in drone_paths), default=0
        )
        return drone_paths, horizon
