# Algorithm

Fly-in routes drones one at a time using a space-time Dijkstra combined with
a reservation table. Each drone takes the fastest path still available, then
reserves it before the next drone is routed.

## Overview

```mermaid
flowchart TD
    A[For each drone] --> B[Space-time Dijkstra<br/>fastest free path]
    B --> C[Reservation table<br/>blocks taken slots]
    C --> D[Reserve the path<br/>occupy hubs and links]
    D -->|next drone| A
    D --> E[All drones routed<br/>horizon = last arrival]
```

## Detailed view

```mermaid
flowchart TD
    A[For each drone] --> B{Priority queue<br/>state: hub, turn}
    B --> C[Explore neighbors<br/>wait or move]
    C --> D{Slot/link free?<br/>capacity respected}
    D -->|no| C
    D -->|yes| E[Relax: turn then priority]
    E --> F{Reached end?}
    F -->|no| B
    F -->|yes| G[Reserve the found path]
    G -->|next drone| A
    G --> H[Horizon = last arrival]
```
