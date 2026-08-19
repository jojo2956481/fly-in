from pydantic import BaseModel, Field, field_validator
from typing import Optional, Literal
import re


class Hub(BaseModel):
    name: str
    x: int
    y: int
    kind: Literal["start", "hub", "end"] = "hub"
    color: Optional[str] = None
    zone: str = "normal"
    max_drones: int = 1
    neighbors: list[str] = Field(default_factory=list)

    @field_validator("max_drones")
    @classmethod
    def positive_capacity(cls, v):
        if v is not None and v <= 0:
            raise ValueError("max_drones must be over 0")
        return v


class Connection(BaseModel):
    src: str
    dst: str
    max_link_capacity: int = 1

    @field_validator("max_link_capacity")
    @classmethod
    def positive_capacity(cls, v):
        if v is not None and v <= 0:
            raise ValueError("max_link_capacity must be over 0")
        return v


class Map_format(BaseModel):
    nb_drones: int
    hubs: list[Hub] = Field(default_factory=list)
    connections: list[Connection] = Field(default_factory=list)

    @field_validator("nb_drones")
    @classmethod
    def positive_capacity(cls, v):
        if v is not None and v <= 0:
            raise ValueError("nb_drones must be over 0")
        return v


def parse_attrs(attr_str: str, nb_line: int) -> dict:
    if not attr_str:
        return {}
    attr_str = attr_str.strip().lstrip("[").rstrip("]")
    if attr_str.startswith(' ') or attr_str.endswith(' '):
        raise ValueError(
            f"Line {nb_line} the metadata must "
            "not be start and finish by space")
    if "= " in attr_str or " =" in attr_str:
        raise ValueError(f"Line {nb_line} unknown : ' '")
    attrs = {}
    for pair in attr_str.split():
        if "=" in pair:
            key, value = pair.split("=", 1)
            if key not in ("color", "zone", "max_drones"):
                raise ValueError(f"Line {nb_line} unknown : {key}")
            if key == "zone":
                if value not in (
                        "normal", "blocked",
                        "restricted", "priority"):
                    raise ValueError(f"Line {nb_line} unknown : {value}")
            attrs[key] = value
    return attrs


def parse_attrs_conn(attr_str: str, nb_line: int) -> dict:
    if not attr_str:
        return {}
    attr_str = attr_str.strip().lstrip("[").rstrip("]")
    if attr_str.startswith(' ') or attr_str.endswith(' '):
        raise ValueError(
            f"Line {nb_line} the metadata must "
            "not be start and finish by space")
    if "= " in attr_str or " =" in attr_str:
        raise ValueError(f"Line {nb_line} unknown : ' '")
    attrs = {}
    for pair in attr_str.split():
        if "=" in pair:
            key, value = pair.split("=", 1)
            if key not in ("max_link_capacity",):
                raise ValueError(f"Line {nb_line} unknown : {key}")
            attrs[key] = value
    return attrs


def load_map(path_map):
    try:
        with open(path_map, "r", encoding="utf-8") as map_file:
            return map_file.readlines()
    except Exception as e:
        raise ValueError(e)


RE_NB_DRONES = re.compile(r"^nb_drones:\s*(\d+)\s*$")

RE_HUB = re.compile(
    r"^(start_hub|hub|end_hub):\s*"
    r"(\S+)\s+"
    r"(-?\d+)\s+"
    r"(-?\d+)"
    r"(?:\s*(\[.*\]))?"
    r"\s*$"
)

RE_CONNECTION = re.compile(
    r"^connection:\s*"
    r"(\S+?)-(\S+?)"
    r"(?:\s*(\[.*\]))?"
    r"\s*$"
)


def parser_file(path_map: str) -> Map_format:
    nb_drones: int = None
    hubs: list[Hub] = []
    conn_lines: list[int] = []
    seen_names: set[str] = set()
    connections: list[Connection] = []
    map = load_map(path_map)
    for nb_line, raw_line in enumerate(map, start=1):
        idx = raw_line.find("#")
        if idx != -1:
            raw_line = raw_line[:idx]
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "  " in line:
            raise ValueError(f"Line {nb_line} unknown : {line}")

        if match := RE_NB_DRONES.match(line):
            nb_drones = int(match.group(1))
            continue

        if match := RE_HUB.match(line):
            kind_raw, name, x, y, attr_str = match.groups()
            if name in seen_names:
                raise ValueError(
                    f"Line {nb_line}: duplicate hub name '{name}'")
            seen_names.add(name)
            attrs = parse_attrs(attr_str or "", nb_line)
            kind = {"start_hub": "start", "hub": "hub",
                    "end_hub": "end"}[kind_raw]
            if attrs.get("zone") == "blocked":
                default_max_drones = 0
            elif kind in ("start", "end"):
                default_max_drones = nb_drones
            else:
                default_max_drones = 1
            hubs.append(Hub(
                name=name,
                x=int(x),
                y=int(y),
                kind=kind,
                color=attrs.get("color"),
                zone=attrs.get("zone", "normal"),
                max_drones=int(attrs["max_drones"])
                if "max_drones" in attrs else default_max_drones,
            ))
            continue

        if match := RE_CONNECTION.match(line):
            default_max_link_capacity: int = 1
            src, dst, attr_conn = match.groups()
            attrs = parse_attrs_conn(attr_conn or "", nb_line)
            connections.append(Connection(
                src=src,
                dst=dst,
                max_link_capacity=attrs.get("max_link_capacity")
                if "max_link_capacity" in attrs else default_max_link_capacity,
            ))
            conn_lines.append(nb_line)
            continue
        raise ValueError(f"Line {nb_line} unknown : {line!r}")
    if nb_drones is None:
        raise ValueError("Missing 'nb_drones:' line in map file")
    hub_names = {hub.name for hub in hubs}
    hub_by_name = {hub.name: hub for hub in hubs}
    seen_pairs = set()
    for conn, cl in zip(connections, conn_lines):
        if conn.src not in hub_names:
            raise ValueError(
                f"Line {cl}: connection references unknown hub '{conn.src}'")
        if conn.dst not in hub_names:
            raise ValueError(
                f"Line {cl}: connection references unknown hub '{conn.dst}'")
        if conn.src == conn.dst:
            raise ValueError(
                f"Line {cl}: self-loop connection not allowed "
                f"'{conn.src}-{conn.dst}'")
        pair = frozenset((conn.src, conn.dst))
        if pair in seen_pairs:
            raise ValueError(
                f"Line {cl}: duplicate connection between "
                f"'{conn.src}' and '{conn.dst}'")
        seen_pairs.add(pair)
        hub_by_name[conn.src].neighbors.append(conn.dst)
        hub_by_name[conn.dst].neighbors.append(conn.src)
    return Map_format(nb_drones=nb_drones, hubs=hubs, connections=connections)
