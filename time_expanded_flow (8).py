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

from collections import defaultdict


def zone_cost(zone):
    """Nombre de tours pour ENTRER dans un hub de cette zone. None = infranchissable."""
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1  # normal ou priority


def extend_time_expanded_graph(drone_map, capacity, cost, t):
    """Ajoute au graphe existant (capacity, cost) UNIQUEMENT les arcs
    partant du tour t (la nouvelle tranche temporelle), sans toucher à ce
    qui existe déjà pour les tours précédents. Permet d'agrandir l'horizon
    sans tout reconstruire depuis zéro à chaque essai.

    Retourne la liste des arcs (u, v) nouvellement ajoutés."""
    hub_by_name = {h.name: h for h in drone_map.hubs}
    priority_hubs = {h.name for h in drone_map.hubs if h.zone == "priority"}
    added_edges = []

    for hub in drone_map.hubs:
        if hub.zone == "blocked":
            continue
        if hub.kind == "end":
            continue

        # rester sur place
        u = (hub.name, t)
        v = (hub.name, t + 1)
        capacity[u][v] += hub.max_drones
        cost[u][v] = -1 if hub.name in priority_hubs else 0
        added_edges.append((u, v))

        # se déplacer vers chaque voisin
        for neighbor_name in hub.neighbors:
            neighbor = hub_by_name[neighbor_name]
            move_cost = zone_cost(neighbor.zone)
            if move_cost is None:
                continue

            conn = next(
                c for c in drone_map.connections
                if {c.src, c.dst} == {hub.name, neighbor_name}
            )
            v = (neighbor_name, t + move_cost)
            capacity[u][v] += conn.max_link_capacity
            cost[u][v] = -1 if neighbor_name in priority_hubs else 0
            added_edges.append((u, v))

    return added_edges


def build_time_expanded_graph(drone_map, horizon):
    """Construit le graphe de capacités sur les tours 0..horizon, tranche
    par tranche (usage ponctuel, pas incrémental d'un appel à l'autre)."""
    capacity = defaultdict(lambda: defaultdict(int))
    cost = defaultdict(lambda: defaultdict(int))
    for t in range(horizon):
        extend_time_expanded_graph(drone_map, capacity, cost, t)
    return capacity, cost


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


def init_residual(capacity, cost):
    """Crée le graphe résiduel de travail (copie de capacity + cost, avec
    les arcs retour) à partir de zéro. Appelé UNE SEULE FOIS au début."""
    residual = defaultdict(lambda: defaultdict(int))
    residual_cost = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v, cap in capacity[u].items():
            residual[u][v] += cap
            residual_cost[u][v] = cost[u][v]
            residual_cost[v][u] = -cost[u][v]
    return residual, residual_cost


def sync_residual_with_new_capacity(capacity, cost, residual, residual_cost, added_edges):
    """Ajoute au graphe résiduel EXISTANT les nouveaux arcs listés dans
    added_edges (liste de (u, v)), sans toucher au reste -> évite de
    perdre le travail (flot déjà poussé) accumulé aux essais précédents."""
    for u, v in added_edges:
        cap = capacity[u][v]
        residual[u][v] += cap
        residual_cost[u][v] = cost[u][v]
        residual_cost[v][u] = -cost[u][v]


def augment_min_cost_flow(residual, residual_cost, source, sink):
    """Pousse autant de flot que possible dans le graphe résiduel ACTUEL
    (qui peut déjà contenir du flot poussé lors d'appels précédents).
    Retourne le flot et le coût ajoutés lors de CET appel."""
    added_flow = 0
    added_cost = 0
    while True:
        path = _bellman_ford_shortest_path(residual, residual_cost, source, sink)
        if path is None:
            break
        bottleneck = min(residual[u][v] for u, v in zip(path, path[1:]))
        path_cost = sum(residual_cost[u][v] for u, v in zip(path, path[1:]))
        for u, v in zip(path, path[1:]):
            residual[u][v] -= bottleneck
            residual[v][u] += bottleneck
        added_flow += bottleneck
        added_cost += bottleneck * path_cost

    return added_flow, added_cost


def min_cost_max_flow(capacity, cost, source, sink):
    """Version 'tout en un', pour un usage ponctuel (pas incrémental) :
    initialise le résiduel puis pousse tout le flot possible d'un coup."""
    residual, residual_cost = init_residual(capacity, cost)
    max_flow, total_cost = augment_min_cost_flow(residual, residual_cost, source, sink)
    return max_flow, total_cost, residual


def schedule_drones(drone_map, max_horizon=500):
    """Trouve le nombre MINIMAL de tours pour que tous les nb_drones
    atteignent 'end'.

    Version incrémentale : le graphe résiduel ("table de réservation") est
    construit et étendu PROGRESSIVEMENT, tranche de tour par tranche de
    tour, sans jamais tout reconstruire/recalculer depuis zéro à chaque
    nouvel essai d'horizon -> le travail déjà fait (flot déjà poussé) est
    conservé et réutilisé d'un essai au suivant.

    Retourne (flot_obtenu, horizon_minimal, residual, capacity, source, sink).
    """
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")
    target = drone_map.nb_drones

    capacity = defaultdict(lambda: defaultdict(int))
    cost = defaultdict(lambda: defaultdict(int))
    source = ("__source__", -1)
    sink = ("__sink__", -1)

    capacity[source][(start.name, 0)] = target
    cost[source][(start.name, 0)] = 0

    residual, residual_cost = init_residual(capacity, cost)
    total_flow = 0

    horizon = 0
    while horizon <= max_horizon:
        horizon += 1
        t = horizon - 1

        # 1) étend le graphe d'une tranche (arcs partant du tour t)
        added_edges = extend_time_expanded_graph(drone_map, capacity, cost, t)

        # 2) connecte le nouveau tour d'arrivée (end, horizon) au puits
        capacity[(end.name, horizon)][sink] += target
        added_edges.append(((end.name, horizon), sink))

        # 3) synchronise le résiduel avec ces nouveaux arcs seulement
        sync_residual_with_new_capacity(capacity, cost, residual, residual_cost, added_edges)

        # 4) pousse le flot supplémentaire rendu possible par cette extension
        added_flow, added_cost = augment_min_cost_flow(residual, residual_cost, source, sink)
        total_flow += added_flow

        if total_flow >= target:
            return total_flow, horizon, residual, capacity, source, sink

    return total_flow, max_horizon, residual, capacity, source, sink


def decompose_drone_paths(capacity, residual, source, sink, nb_drones):
    """Extrait, à partir du flot final, la trajectoire concrète de chaque
    drone individuel : une liste de listes de (hub_name, tour).

    Le flot ne "sait" que combien de drones empruntent chaque tronçon —
    cette fonction retrouve les chemins réellement utilisés en cherchant,
    répétitivement, un chemin source->sink dont l'usage réel (capacité
    initiale - capacité résiduelle restante) est encore positif.
    """
    flow_used = defaultdict(lambda: defaultdict(int))
    for u in capacity:
        for v, cap in capacity[u].items():
            used = cap - residual[u][v]
            if used > 0:
                flow_used[u][v] = used

    grouped_paths = []  # liste de (chemin, nb_drones_sur_ce_chemin)
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

        # retire source/sink (artificiels), garde juste les (hub, tour) réels
        clean_path = [node for node in path if node[0] not in ("__source__", "__sink__")]
        grouped_paths.append((clean_path, bottleneck))

    # explose chaque groupe en trajectoires individuelles (un drone = un chemin)
    drone_paths = []
    for path, count in grouped_paths:
        for _ in range(count):
            drone_paths.append(path)

    return drone_paths
