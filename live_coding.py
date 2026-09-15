

finish = capacity_info(drone_map, drone_paths, int(sim_turn), finish)

def capacity_info(drone_map, drone_paths, turn, finish):
    print(f"--- tour {turn} ---")
    lst_hub = []
    for hub in drone_map.hubs:
        if hub.zone == "restricted":
            conn = copy.deepcopy(hub)
            conn.name = f"{lst_hub[-1].name}-{hub.name}"
            lst_hub.append(conn)
        lst_hub.append(hub)
    for hub in lst_hub:
        count_hub = sum(
            1
            for path in drone_paths
            for (name, t, state) in path
            if name == hub.name and t == turn
        )
        label = f"{hub.name}: {count_hub} / {hub.max_drones}"
        if hub.kind == "end" and count_hub > 0:
            finish += 1
            label = f"{hub.name}: {finish} / {hub.max_drones}"
        print(label)
    return finish