import re
from typing import Optional, Literal

from pydantic import BaseModel, Field, field_validator


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
    def positive_capacity(cls, v: int) -> int:
        if v is not None and v <= 0:
            raise ValueError("max_drones must be over 0")
        return v


class Connection(BaseModel):
    src: str
    dst: str
    max_link_capacity: int = 1

    @field_validator("max_link_capacity")
    @classmethod
    def positive_capacity(cls, v: int) -> int:
        if v is not None and v <= 0:
            raise ValueError("max_link_capacity must be over 0")
        return v


class Map_format(BaseModel):
    nb_drones: int
    hubs: list[Hub] = Field(default_factory=list)
    connections: list[Connection] = Field(default_factory=list)

    @field_validator("nb_drones")
    @classmethod
    def positive_capacity(cls, v: int) -> int:
        if v is not None and v <= 0:
            raise ValueError("nb_drones must be over 0")
        return v


class Patterns:
    NB_DRONES = re.compile(r"^nb_drones:\s*(\d+)\s*$")
    HUB = re.compile(
        r"^(start_hub|hub|end_hub):\s*"
        r"(\S+)\s+"
        r"(-?\d+)\s+"
        r"(-?\d+)"
        r"(?:\s*(\[.*\]))?"
        r"\s*$"
    )
    CONNECTION = re.compile(
        r"^connection:\s*"
        r"(\S+?)-(\S+?)"
        r"(?:\s*(\[.*\]))?"
        r"\s*$"
    )


KIND_MAP: dict[str, str] = {
    "start_hub": "start",
    "hub": "hub",
    "end_hub": "end",
}

HUB_KEYS: tuple[str, ...] = ("color", "zone", "max_drones")
CONN_KEYS: tuple[str, ...] = ("max_link_capacity",)
ZONE_VALUES: tuple[str, ...] = (
    "normal", "blocked", "restricted", "priority",
)


def _check_spacing(attr_str: str, nb_line: int) -> None:
    if attr_str.startswith(" ") or attr_str.endswith(" "):
        raise ValueError(
            f"Line {nb_line} the metadata must "
            "not be start and finish by space"
        )
    if "= " in attr_str or " =" in attr_str:
        raise ValueError(f"Line {nb_line} unknown : ' '")


def parse_attrs(attr_str: str, nb_line: int) -> dict[str, str]:
    if not attr_str:
        return {}
    attr_str = attr_str.strip().lstrip("[").rstrip("]")
    _check_spacing(attr_str, nb_line)
    attrs: dict[str, str] = {}
    for pair in attr_str.split():
        if "=" not in pair:
            continue
        key, value = pair.split("=", 1)
        if key not in HUB_KEYS:
            raise ValueError(f"Line {nb_line} unknown : {key}")
        if key == "zone" and value not in ZONE_VALUES:
            raise ValueError(f"Line {nb_line} unknown : {value}")
        if key in attrs:
            raise ValueError(f"Line {nb_line} duplicate : {key}")
        attrs[key] = value
    return attrs


def parse_attrs_conn(attr_str: str, nb_line: int) -> dict[str, str]:
    if not attr_str:
        return {}
    attr_str = attr_str.strip().lstrip("[").rstrip("]")
    _check_spacing(attr_str, nb_line)
    attrs: dict[str, str] = {}
    for pair in attr_str.split():
        if "=" not in pair:
            continue
        key, value = pair.split("=", 1)
        if key not in CONN_KEYS:
            raise ValueError(f"Line {nb_line} unknown : {key}")
        if key in attrs:
            raise ValueError(f"Line {nb_line} duplicate : {key}")
        attrs[key] = value
    return attrs


def load_map(path_map: str) -> list[str]:
    try:
        with open(path_map, "r", encoding="utf-8") as map_file:
            return map_file.readlines()
    except Exception as e:
        raise ValueError(e)


def _strip_comment(raw_line: str) -> str:
    idx = raw_line.find("#")
    if idx != -1:
        raw_line = raw_line[:idx]
    return raw_line.strip()


def _hub_default_capacity(
    kind: str,
    zone: Optional[str],
    nb_drones: int,
) -> int:
    if zone == "blocked":
        return 0
    if kind in ("start", "end"):
        return nb_drones
    return 1


def _build_hub(
    match: "re.Match[str]",
    nb_line: int,
    nb_drones: int,
    seen_names: set[str],
    seen_coords: set[tuple[int, int]],
) -> Hub:
    kind_raw, name, x_str, y_str, attr_str = match.groups()
    if name in seen_names:
        raise ValueError(
            f"Line {nb_line}: duplicate hub name '{name}'"
        )
    seen_names.add(name)

    coord = (int(x_str), int(y_str))
    if coord in seen_coords:
        raise ValueError(
            f"Line {nb_line}: duplicate hub coordinates {coord}"
        )
    seen_coords.add(coord)

    attrs = parse_attrs(attr_str or "", nb_line)
    kind = KIND_MAP[kind_raw]
    default_cap = _hub_default_capacity(
        kind, attrs.get("zone"), nb_drones
    )
    max_drones = (
        int(attrs["max_drones"])
        if "max_drones" in attrs
        else default_cap
    )
    return Hub(
        name=name,
        x=int(x_str),
        y=int(y_str),
        kind=kind,
        color=attrs.get("color"),
        zone=attrs.get("zone", "normal"),
        max_drones=max_drones,
    )


def _build_connection(
    match: "re.Match[str]",
    nb_line: int,
) -> Connection:
    src, dst, attr_conn = match.groups()
    attrs = parse_attrs_conn(attr_conn or "", nb_line)
    default_cap = 1
    max_link_capacity = (
        int(attrs["max_link_capacity"])
        if "max_link_capacity" in attrs
        else default_cap
    )
    return Connection(
        src=src,
        dst=dst,
        max_link_capacity=max_link_capacity,
    )


def _validate_connections(
    hubs: list[Hub],
    connections: list[Connection],
    conn_lines: list[int],
) -> None:
    hub_names = {hub.name for hub in hubs}
    hub_by_name = {hub.name: hub for hub in hubs}
    seen_pairs: set[frozenset[str]] = set()
    for conn, cl in zip(connections, conn_lines):
        if conn.src not in hub_names:
            raise ValueError(
                f"Line {cl}: connection references "
                f"unknown hub '{conn.src}'"
            )
        if conn.dst not in hub_names:
            raise ValueError(
                f"Line {cl}: connection references "
                f"unknown hub '{conn.dst}'"
            )
        if conn.src == conn.dst:
            raise ValueError(
                f"Line {cl}: self-loop connection not allowed "
                f"'{conn.src}-{conn.dst}'"
            )
        pair = frozenset((conn.src, conn.dst))
        if pair in seen_pairs:
            raise ValueError(
                f"Line {cl}: duplicate connection between "
                f"'{conn.src}' and '{conn.dst}'"
            )
        seen_pairs.add(pair)
        hub_by_name[conn.src].neighbors.append(conn.dst)
        hub_by_name[conn.dst].neighbors.append(conn.src)


def parser_file(path_map: str) -> Map_format:
    nb_drones: Optional[int] = None
    hubs: list[Hub] = []
    conn_lines: list[int] = []
    seen_names: set[str] = set()
    seen_coords: set[tuple[int, int]] = set()
    connections: list[Connection] = []

    for nb_line, raw_line in enumerate(load_map(path_map), start=1):
        line = _strip_comment(raw_line)
        if not line:
            continue
        if "  " in line:
            raise ValueError(f"Line {nb_line} unknown : {line}")

        if match := Patterns.NB_DRONES.match(line):
            nb_drones = int(match.group(1))
            continue

        if match := Patterns.HUB.match(line):
            drones_so_far = nb_drones if nb_drones is not None else 1
            hubs.append(
                _build_hub(
                    match, nb_line, drones_so_far,
                    seen_names, seen_coords,
                )
            )
            continue

        if match := Patterns.CONNECTION.match(line):
            connections.append(_build_connection(match, nb_line))
            conn_lines.append(nb_line)
            continue

        raise ValueError(f"Line {nb_line} unknown : {line!r}")

    if nb_drones is None:
        raise ValueError("Missing 'nb_drones:' line in map file")

    _validate_connections(hubs, connections, conn_lines)
    return Map_format(
        nb_drones=nb_drones,
        hubs=hubs,
        connections=connections,
    )
