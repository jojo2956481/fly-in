"""
Planification multi-drones par Dijkstra spatio-temporel + table de
réservation (approche "Cooperative Dijkstra").

Principe :
  - On route les drones UN PAR UN.
  - Pour chaque drone, un Dijkstra dans l'espace-temps (nœuds = (hub, tour))
    trouve le chemin arrivant le PLUS TÔT possible à 'end', en évitant les
    cases/liens déjà réservés par les drones précédents (capacités respectées).
  - Le chemin trouvé est ensuite RÉSERVÉ dans la table, puis on passe au
    drone suivant.

Règles :
  - zone blocked : infranchissable
  - zone restricted : 2 tours pour l'atteindre ; normal/priority : 1 tour
  - capacité de hub (max_drones) : nb max de drones présents au même tour
  - capacité de connexion (max_link_capacity) : nb max de départs sur ce
    lien au même tour
  - start et end : capacité = nb_drones (jamais limitants)
  - arriver à 'end' à n'importe quel tour = livré

Limite connue : approche gloutonne (greedy), pas garantie globalement
optimale — l'ordre de routage des drones influe sur le résultat. Simple,
rapide, et suffisant en pratique pour des horizons raisonnables.
"""

import heapq
from collections import defaultdict


def zone_cost(zone):
    """Nombre de tours pour ENTRER dans un hub de cette zone. None = infranchissable."""
    if zone == "blocked":
        return None
    if zone == "restricted":
        return 2
    return 1  # normal ou priority


class ReservationTable:
    """Mémorise les occupations déjà réservées, pour faire respecter les
    capacités entre drones successifs."""

    def __init__(self):
        # (hub_name, turn) -> nb de drones présents à ce hub à ce tour
        self.hub_occupancy = defaultdict(int)
        # (src, dst, turn) -> nb de drones ayant démarré ce lien à ce tour
        self.link_usage = defaultdict(int)

    def hub_has_room(self, hub_name, turn, capacity):
        return self.hub_occupancy[(hub_name, turn)] < capacity

    def link_has_room(self, src, dst, turn, capacity):
        return self.link_usage[(src, dst, turn)] < capacity

    def reserve_path(self, path):
        """path : liste de (hub_name, turn). Réserve la présence à chaque
        hub et l'usage de chaque lien emprunté."""
        for hub_name, turn in path:
            self.hub_occupancy[(hub_name, turn)] += 1
        for (src, t_src), (dst, t_dst) in zip(path, path[1:]):
            if src != dst:  # vrai déplacement (pas un "rester sur place")
                self.link_usage[(src, dst, t_src)] += 1


def dijkstra_spacetime(drone_map, start_name, end_name, reservation, max_horizon):
    """Trouve le chemin arrivant le plus tôt possible de start à end dans
    l'espace-temps, en respectant la table de réservation.

    Retourne une liste de (hub_name, turn), ou None si aucun chemin trouvé
    dans la limite max_horizon."""
    hub_by_name = {h.name: h for h in drone_map.hubs}

    # état = (hub_name, turn). Coût = turn (on veut arriver au plus tôt).
    start_state = (start_name, 0)
    dist = {start_state: 0}
    parent = {start_state: None}
    pq = [(0, start_state)]  # (turn, (hub, turn))

    while pq:
        turn, (hub_name, t) = heapq.heappop(pq)

        if hub_name == end_name:
            # reconstruit le chemin
            path = []
            state = (hub_name, t)
            while state is not None:
                path.append(state)
                state = parent[state]
            path.reverse()
            return path

        if turn > dist.get((hub_name, t), float("inf")):
            continue  # entrée obsolète dans la pq

        if t >= max_horizon:
            continue  # ne pas explorer au-delà de l'horizon

        hub = hub_by_name[hub_name]

        # option 1 : rester sur place (hub_name, t) -> (hub_name, t+1)
        nt = t + 1
        if reservation.hub_has_room(hub_name, nt, hub.max_drones):
            _relax(dist, parent, pq, (hub_name, t), (hub_name, nt))

        # option 2 : se déplacer vers chaque voisin
        for neighbor_name in hub.neighbors:
            neighbor = hub_by_name[neighbor_name]
            cost = zone_cost(neighbor.zone)
            if cost is None:
                continue  # zone blocked
            nt = t + cost
            if nt > max_horizon:
                continue
            conn = next(
                c for c in drone_map.connections
                if {c.src, c.dst} == {hub_name, neighbor_name}
            )
            # capacité du lien (au tour de départ t) ET du hub d'arrivée (au tour nt)
            if not reservation.link_has_room(hub_name, neighbor_name, t, conn.max_link_capacity):
                continue
            if not reservation.hub_has_room(neighbor_name, nt, neighbor.max_drones):
                continue
            _relax(dist, parent, pq, (hub_name, t), (neighbor_name, nt))

    return None  # aucun chemin trouvé


def _relax(dist, parent, pq, current_state, next_state):
    """Met à jour la distance vers next_state si passer par current_state
    donne un tour d'arrivée plus petit."""
    new_turn = next_state[1]  # le coût EST le tour d'arrivée
    if new_turn < dist.get(next_state, float("inf")):
        dist[next_state] = new_turn
        parent[next_state] = current_state
        heapq.heappush(pq, (new_turn, next_state))


def schedule_drones(drone_map, max_horizon=200):
    """Route tous les drones un par un via Dijkstra + réservation.

    Retourne (drone_paths, horizon) :
      - drone_paths : liste de chemins (un par drone livré), chaque chemin
        étant une liste de (hub_name, turn).
      - horizon : tour d'arrivée du dernier drone livré.
    Si un drone ne peut être routé, il est ignoré (non ajouté à drone_paths)."""
    start = next(h for h in drone_map.hubs if h.kind == "start")
    end = next(h for h in drone_map.hubs if h.kind == "end")

    reservation = ReservationTable()
    drone_paths = []

    for _ in range(drone_map.nb_drones):
        path = dijkstra_spacetime(
            drone_map, start.name, end.name, reservation, max_horizon
        )
        if path is None:
            continue  # ce drone n'a pas pu être routé dans l'horizon
        reservation.reserve_path(path)
        drone_paths.append(path)

    horizon = max((p[-1][1] for p in drone_paths), default=0)
    return drone_paths, horizon


if __name__ == "__main__":
    from src.parsing_map import parser_file

    drone_map = parser_file("maps/challenger/01_the_impossible_dream.txt")
    drone_paths, horizon = schedule_drones(drone_map)

    print(f"Demande         : {drone_map.nb_drones} drones")
    print(f"Drones routés   : {len(drone_paths)}")
    print(f"Horizon (dernier arrivé) : {horizon} tours")
    print()
    for i, path in enumerate(drone_paths):
        print(f"drone {i}: " + " -> ".join(f"{h}@{t}" for h, t in path))
