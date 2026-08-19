"""
Planification multi-drones tour par tour via un graphe temporellement
étendu (time-expanded graph) + flot maximum.

Principe : un noeud = (hub, tour). Un drone "présent" en (hub, t) peut :
  - rester sur place  -> arc (hub, t) -> (hub, t+1), capacité = max_drones
  - se déplacer        -> arc (u, t) -> (v, t + cout_zone(v)), capacité = max_link_capacity

Comme chaque noeud (v, t) représente l'état APRES les mouvements du tour t,
un drone qui quitte v ne consomme plus sa capacité à ce tour : la règle
"un drone qui part libère la place pour un autre qui arrive au même tour"
est gérée automatiquement par la construction, sans code spécial.

Hypothèses (à confirmer / ajuster si besoin) :
  - La capacité d'une connexion pour un trajet 2 tours (zone restricted)
    est consommée au moment du DEPART du trajet, pas doublement sur les 2 tours.
  - L'horizon temporel n'est pas fixe : on ajoute des tours jusqu'à ce que
    le flot max atteigne nb_drones ou plafonne (plus aucune amélioration).
"""

from collections import defaultdict, deque


def zone_cost(zone):
    """Nombre de tours pour ENTRER dans un hub de cette zone. None = infranchissable."""
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1  # normal ou priority


def build_time_expanded_graph(drone_map, horizon):
    """Construit le graphe de capacités sur les tours 0..horizon.

    Un seul noeud par (hub, tour). Un drone peut enchaîner ses déplacements
    sans délai forcé, tant que les capacités (hub et connexion) le
    permettent -> pas d'attente obligatoire après une arrivée.
    """
    hub_by_name = {h.name: h for h in drone_map.hubs}
    capacity = defaultdict(lambda: defaultdict(int))

    for t in range(horizon):
        for hub in drone_map.hubs:
            if hub.zone == "blocked":
                continue

            # rester sur place
            capacity[(hub.name, t)][(hub.name, t + 1)] += hub.max_drones

            # se déplacer vers chaque voisin
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
                # arrivée chez le voisin, directement au tour d'arrivée
                capacity[(hub.name, t)][(neighbor_name, t + cost)] += conn.max_link_capacity

    return capacity


def build_edge_costs(drone_map, capacity):
    """Associe un coût à chaque arc de mouvement du graphe : -1 pour un
    arc qui mène vers un hub 'priority' (préférence à coût de tours égal),
    0 sinon (normal, rester sur place, restricted). Le coût est indexé sur
    le NOM du hub de destination (indépendant du tour), donc valable pour
    tout arc (u, t) -> (v, t') quel que soit t'."""
    priority_hubs = {h.name for h in drone_map.hubs if h.zone == "priority"}

    cost = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v in capacity[u]:
            dest_hub_name = v[0]
            cost[u][v] = -1 if dest_hub_name in priority_hubs else 0
    return cost


def _bellman_ford_shortest_path(residual, cost, source, sink):
    """Cherche le chemin de coût minimal source->sink dans le graphe
    résiduel (arcs de capacité résiduelle > 0 uniquement). Le graphe étant
    un DAG (le temps croît strictement le long de tout chemin), aucun
    cycle négatif n'est possible : Bellman-Ford termine proprement."""
    dist = defaultdict(lambda: float("inf"))
    dist[source] = 0
    parent = {source: None}

    # les nœuds forment un DAG : |V|-1 relaxations suffisent, on prend
    # une marge large plutôt que de trier topologiquement
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
    """Flot maximum, et parmi les flots atteignant ce maximum, celui de
    coût total minimal (préfère les chemins passant par des zones
    'priority' à quantité de flot égale)."""
    residual = defaultdict(lambda: defaultdict(int))
    residual_cost = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v, cap in capacity[u].items():
            residual[u][v] += cap
            residual_cost[u][v] = cost[u][v]
            residual_cost[v][u] = -cost[u][v]  # arc retour : coût opposé

    max_flow = 0
    total_cost = 0
    while True:
        path = _bellman_ford_shortest_path(residual, residual_cost, source, sink)
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


def _bfs_augmenting_path(residual, source, sink):
    parent = {source: None}
    queue = deque([source])
    while queue:
        u = queue.popleft()
        if u == sink:
            break
        for v, cap in residual[u].items():
            if cap > 0 and v not in parent:
                parent[v] = u
                queue.append(v)
    if sink not in parent:
        return None
    path = []
    node = sink
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path


def edmonds_karp(capacity, source, sink):
    residual = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v, cap in capacity[u].items():
            residual[u][v] += cap
            residual[v][u] += 0

    max_flow = 0
    while True:
        path = _bfs_augmenting_path(residual, source, sink)
        if path is None:
            break
        bottleneck = min(residual[u][v] for u, v in zip(path, path[1:]))
        for u, v in zip(path, path[1:]):
            residual[u][v] -= bottleneck
            residual[v][u] += bottleneck
        max_flow += bottleneck

    return max_flow, residual


def _flow_at_horizon(drone_map, horizon, start, end):
    """Calcule le flot maximum atteignable en exactement `horizon` tours."""
    capacity = build_time_expanded_graph(drone_map, horizon)
    source = ("__source__", -1)
    sink = ("__sink__", -1)
    capacity[source][(start.name, 0)] = drone_map.nb_drones
    for t in range(horizon + 1):
        capacity[(end.name, t)][sink] += drone_map.nb_drones
    max_flow, residual = edmonds_karp(capacity, source, sink)
    return max_flow, residual, capacity, source, sink


def schedule_drones(drone_map, max_horizon=500):
    """Trouve le nombre MINIMAL de tours pour que tous les nb_drones
    atteignent 'end'.

    Recherche exponentielle (double l'horizon jusqu'à atteindre nb_drones
    ou détecter un plafond structurel), puis recherche dichotomique pour
    trouver le T exact minimal (le flot est croissant avec le nombre de
    tours disponibles, donc dichotomie valide).

    Retourne (flot_obtenu, horizon_minimal, residual, capacity, source, sink).
    Si drone_map.nb_drones n'est jamais atteignable (goulot structurel),
    flot_obtenu < nb_drones et horizon_minimal = max_horizon.
    """
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")
    target = drone_map.nb_drones

    # 1) recherche exponentielle d'un horizon "hi" qui atteint target.
    #    Pas de détection anticipée de "plafond" ici : un flot nul ou
    #    stable sur quelques petits horizons est normal (pas encore assez
    #    de tours pour qu'un seul drone arrive), pas forcément un vrai
    #    goulot structurel. On se fie uniquement à max_horizon.
    lo = 0
    hi = 1
    result = None
    flow = residual = capacity = source = sink = None

    while hi <= max_horizon:
        flow, residual, capacity, source, sink = _flow_at_horizon(drone_map, hi, start, end)
        if flow >= target:
            result = (flow, residual, capacity, source, sink)
            break
        lo = hi
        hi *= 2

    if result is None:
        # jamais atteint target avant max_horizon : goulot réel, ou
        # max_horizon trop petit -> augmenter max_horizon si besoin
        return flow, max_horizon, residual, capacity, source, sink

    hi_flow, hi_residual, hi_capacity, hi_source, hi_sink = result

    # 2) dichotomie entre lo (flot < target, ou 0) et hi (flot >= target)
    #    pour trouver le T minimal exact
    best = (hi_flow, hi, hi_residual, hi_capacity, hi_source, hi_sink)
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        flow, residual, capacity, source, sink = _flow_at_horizon(drone_map, mid, start, end)
        if flow >= target:
            hi = mid
            best = (flow, mid, residual, capacity, source, sink)
        else:
            lo = mid

    flow, horizon, residual, capacity, source, sink = best

    # 3) T minimal trouvé : re-résout à CE T exact en min-cost max-flow,
    #    pour départager les chemins de coût (tours) égal en préférant
    #    ceux qui passent par des zones 'priority'
    cost = build_edge_costs(drone_map, capacity)
    flow, total_cost, residual = min_cost_max_flow(capacity, cost, source, sink)

    return flow, horizon, residual, capacity, source, sink


if __name__ == "__main__":
    from src.parsing_map import parser_file

    drone_map = parser_file("maps/challenger/01_the_impossible_dream.txt")
    max_flow, horizon, residual, capacity, source, sink = schedule_drones(drone_map)

    print(f"Demande            : {drone_map.nb_drones} drones")
    print(f"Flot obtenu        : {max_flow} drones")
    print(f"Horizon utilisé    : {horizon} tours")
    if max_flow < drone_map.nb_drones:
        print("-> Impossible de router tous les drones : capacité réseau insuffisante.")
