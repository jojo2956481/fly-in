
from collections import defaultdict, deque


def zone_cost(zone):
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1


def build_time_expanded_graph(drone_map, horizon):

    hub_by_name = {h.name: h for h in drone_map.hubs}
    capacity = defaultdict(lambda: defaultdict(int))
    for t in range(horizon):
        for hub in drone_map.hubs:
            if hub.zone == "blocked":
                continue
            if hub.kind == "end":
                continue
            capacity[(hub.name, t)][hub.name, t + 1] += hub.max_drones
            for neighbor_name in hub.neighbors:
                neighbor = hub_by_name[neighbor_name]
                cost = zone_cost(neighbor.zone)
                if cost is None:
                    continue
                if t + cost > horizon:
                    continue
                conn = next(
                     c for c in drone_map.connections
                     if {c.src, c.dst} == {hub.name, neighbor_name}
                )
                capacity[(hub.name, t)][(neighbor_name, t + cost)] += conn.max_link_capacity
    return capacity


def build_edge_costs(drone_map, capacity):
    priority_hubs = {h.name for h in drone_map.hubs if h.zone == "priority"}
    cost = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v in capacity[u]:
            dest_hub_name = v[0]
            cost[u][v] = -1 if dest_hub_name in priority_hubs else 0
    return cost


def bellman_ford_shortest_path(residual, cost, source, sink):
    dist = defaultdict(lambda: float("inf"))
    dist[source] = 0
    parent = {source: None}
    nodes = list(residual.keys())
    for _ in range(len(nodes) + 1):
        updated = False
        for u in list(residual.keys()):
            if dist[u] == float("inf"):
                continue
            for v, cap in residual[u].items():
                if cap > 0 and dist[u] + cost[u][v] < dist[v]:
                    dist[v] = dist[u] + cost[u][v]
                    parent[v] = u
                    updated = True
        if not updated:
            break
    if dist[sink] == float("inf"):
        return None
    path = []
    node = sink
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path


def min_cost_max_flow(capacity, cost, source, sink):
    residual = defaultdict(lambda: defaultdict(int))
    residual_cost = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v, cap in capacity[u].items():
            residual[u][v] += cap
            residual_cost[u][v] = cost[u][v]
            residual_cost[v][u] = -cost[u][v]
    max_flow = 0
    total_cost = 0
    while True:
        path = bellman_ford_shortest_path(residual, residual_cost, source, sink)
        if path is None:
            break
        bottleneck = min(residual[u][v] for u, v in zip(path, path[1:]))
        path_cost = sum(residual_cost[u][v] for u, v in zip(path, path[1:]))
        for u, v in zip(path, path[1:]):
            residual[u][v] -= bottleneck
            residual[v][u] += bottleneck
        max_flow += bottleneck
        total_cost += bottleneck * path_cost
    return max_flow, total_cost, residual


def flow_at_horizon(drone_map, horizon, start, end):
    capacity = build_time_expanded_graph(drone_map, horizon)
    cost = build_edge_costs(drone_map, capacity)
    source = ("__source__", -1)
    sink = ("__sink__", -1)
    capacity[source][(start.name, 0)] = drone_map.nb_drones
    for t in range(horizon + 1):
        capacity[(end.name, t)][sink] += drone_map.nb_drones
    max_flow, total_cost, residual = min_cost_max_flow(capacity, cost, source, sink)
    return max_flow, residual, capacity, source, sink


def schedule_drones(drone_map, max_horizon=500):
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")
    target = drone_map.nb_drones
    horizon = 0
    while horizon <= max_horizon:
        horizon += 1
        flow, residual, capacity, source, sink = flow_at_horizon(drone_map, horizon, start, end)
        if flow >= target:
            return flow, horizon, residual, capacity, source, sink
    return flow, max_horizon, residual, capacity, source, sink


def decompose_drone_paths(capacity, residual, source, sink):
    flow_used = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v, cap in capacity[u].items():
            used = cap - residual[u][v]
            if used > 0:
                flow_used[u][v] = used
    grouped_paths = []
    while True:
        parent = {source: None}
        stack = [source]
        while stack:
            u = stack.pop()
            if u == sink:
                break
            for v, f in flow_used[u].items():
                if f > 0 and v not in parent:
                    parent[v] = u
                    stack.append(v)
        if sink not in parent:
            break
        path = []
        node = sink
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()
        bottleneck = min(flow_used[u][v] for u, v in zip(path, path[1:]))
        for u, v in zip(path, path[1:]):
            flow_used[u][v] -= bottleneck
        clean_path = [node for node in path if node[0] not in ("__source__", "__sink__")]
        grouped_paths.append((clean_path, bottleneck))
    drone_paths = []
    for path, count in grouped_paths:
        for _ in range(count):
            drone_paths.append(path)

    return drone_paths

def simulation(drone_map):
    return schedule_drones(drone_map)
