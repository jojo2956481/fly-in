
import heapq
from collections import defaultdict


def zone_cost(zone):
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1


class ReservationTable:

    def __init__(self):
        self.hub_occupancy = defaultdict(int)
        self.link_usage = defaultdict(int)

    def hub_has_room(self, hub_name, turn, capacity):
        return self.hub_occupancy[(hub_name, turn)] < capacity

    def link_has_room(self, src, dst, turn, capacity):
        return self.link_usage[(src, dst, turn)] < capacity

    def reserve_path(self, path):
        for hub_name, turn in path:
            self.hub_occupancy[(hub_name, turn)] += 1
        for (src, t_src), (dst, t_dst) in zip(path, path[1:]):
            if src != dst:
                self.link_usage[(src, dst, t_src)] += 1


def dijkstra_spacetime(drone_map, start_name, end_name, reservation, max_horizon):
    hub_by_name = {h.name: h for h in drone_map.hubs}
    start_state = (start_name, 0)
    dist = {start_state: 0}
    parent = {start_state: None}
    pq = [(0, start_state)]
    while pq:
        turn, (hub_name, t) = heapq.heappop(pq)

        if hub_name == end_name:
            path = []
            state = (hub_name, t)
            while state is not None:
                path.append(state)
                state = parent[state]
            path.reverse()
            return path

        if turn > dist.get((hub_name, t), float("inf")):
            continue 
        if t >= max_horizon:
            continue

        hub = hub_by_name[hub_name]
        nt = t + 1
        if reservation.hub_has_room(hub_name, nt, hub.max_drones):
            _relax(dist, parent, pq, (hub_name, t), (hub_name, nt))
        for neighbor_name in hub.neighbors:
            neighbor = hub_by_name[neighbor_name]
            cost = zone_cost(neighbor.zone)
            if cost is None:
                continue  
            nt = t + cost
            if nt > max_horizon:
                continue
            conn = next(
                c for c in drone_map.connections
                if {c.src, c.dst} == {hub_name, neighbor_name}
            )
            if not reservation.link_has_room(hub_name, neighbor_name, t, conn.max_link_capacity):
                continue
            if not reservation.hub_has_room(neighbor_name, nt, neighbor.max_drones):
                continue
            _relax(dist, parent, pq, (hub_name, t), (neighbor_name, nt))

    return None


def _relax(dist, parent, pq, current_state, next_state):
    new_turn = next_state[1]
    if new_turn < dist.get(next_state, float("inf")):
        dist[next_state] = new_turn
        parent[next_state] = current_state
        heapq.heappush(pq, (new_turn, next_state))


def schedule_drones(drone_map, max_horizon=200):
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")

    reservation = ReservationTable()
    drone_paths = []

    for _ in range(drone_map.nb_drones):
        path = dijkstra_spacetime(
            drone_map, start.name, end.name, reservation, max_horizon
        )
        if path is None:
            continue
        reservation.reserve_path(path)
        drone_paths.append(path)

    horizon = max((p[-1][1] for p in drone_paths), default=0)
    return drone_paths, horizon
