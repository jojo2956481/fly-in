import re
from pydantic import BaseModel, Field, field_validator
from typing import Optional, Literal, cast


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


class MapParser:
    """Lit un fichier de map, valide sa syntaxe et ses contraintes, et
    produit un Map_format (hubs, connexions, voisins renseignés)."""

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

    def __init__(self, path_map: str) -> None:
        self.path_map = path_map
        self.nb_drones: Optional[int] = None
        self.hubs: list[Hub] = []
        self.connections: list[Connection] = []
        self.conn_lines: list[int] = []
        self.seen_names: set[str] = set()
        self.seen_coords: set[tuple[int, int]] = set()

    def _load(self) -> list[str]:
        try:
            with open(self.path_map, "r", encoding="utf-8") as fp:
                return fp.readlines()
        except Exception as e:
            raise ValueError(e)

    @staticmethod
    def _strip_comment(raw_line: str) -> str:
        idx = raw_line.find("#")
        if idx != -1:
            raw_line = raw_line[:idx]
        return raw_line.strip()

    @staticmethod
    def _check_spacing(attr_str: str, nb_line: int) -> None:
        if attr_str.startswith(" ") or attr_str.endswith(" "):
            raise ValueError(
                f"Line {nb_line} the metadata must "
                "not be start and finish by space"
            )
        if "= " in attr_str or " =" in attr_str:
            raise ValueError(f"Line {nb_line} unknown : ' '")

    def _parse_attrs(
        self,
        attr_str: str,
        nb_line: int,
        allowed: tuple[str, ...],
        check_zone: bool,
    ) -> dict[str, str]:
        if not attr_str:
            return {}
        attr_str = attr_str.strip().lstrip("[").rstrip("]")
        self._check_spacing(attr_str, nb_line)
        attrs: dict[str, str] = {}
        for pair in attr_str.split():
            if "=" not in pair:
                continue
            key, value = pair.split("=", 1)
            if key not in allowed:
                raise ValueError(f"Line {nb_line} unknown : {key}")
            if (
                check_zone
                and key == "zone"
                and value not in self.ZONE_VALUES
            ):
                raise ValueError(f"Line {nb_line} unknown : {value}")
            if key in attrs:
                raise ValueError(f"Line {nb_line} duplicate : {key}")
            attrs[key] = value
        return attrs

    def _default_capacity(
        self, kind: str, zone: Optional[str]
    ) -> int:
        if zone == "blocked":
            return 0
        if kind in ("start", "end"):
            return self.nb_drones if self.nb_drones is not None else 1
        return 1

    def _build_hub(self, match: "re.Match[str]", nb_line: int) -> Hub:
        kind_raw, name, x_str, y_str, attr_str = match.groups()
        if name in self.seen_names:
            raise ValueError(
                f"Line {nb_line}: duplicate hub name '{name}'"
            )
        self.seen_names.add(name)

        coord = (int(x_str), int(y_str))
        if coord in self.seen_coords:
            raise ValueError(
                f"Line {nb_line}: duplicate hub coordinates {coord}"
            )
        self.seen_coords.add(coord)

        attrs = self._parse_attrs(
            attr_str or "", nb_line, self.HUB_KEYS, True
        )
        kind = self.KIND_MAP[kind_raw]
        default_cap = self._default_capacity(kind, attrs.get("zone"))
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
        self, match: "re.Match[str]", nb_line: int
    ) -> Connection:
        src, dst, attr_conn = match.groups()
        attrs = self._parse_attrs(
            attr_conn or "", nb_line, self.CONN_KEYS, False
        )
        max_link_capacity = (
            int(attrs["max_link_capacity"])
            if "max_link_capacity" in attrs
            else 1
        )
        return Connection(
            src=src, dst=dst, max_link_capacity=max_link_capacity
        )

    def _validate_connections(self) -> None:
        hub_names = {hub.name for hub in self.hubs}
        hub_by_name = {hub.name: hub for hub in self.hubs}
        seen_pairs: set[frozenset[str]] = set()
        for conn, cl in zip(self.connections, self.conn_lines):
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

    def parse(self) -> Map_format:
        for nb_line, raw_line in enumerate(self._load(), start=1):
            line = self._strip_comment(raw_line)
            if not line:
                continue
            if "  " in line:
                raise ValueError(f"Line {nb_line} unknown : {line}")

            if match := self.NB_DRONES.match(line):
                self.nb_drones = int(match.group(1))
                continue

            if match := self.HUB.match(line):
                self.hubs.append(self._build_hub(match, nb_line))
                continue

            if match := self.CONNECTION.match(line):
                self.connections.append(
                    self._build_connection(match, nb_line)
                )
                self.conn_lines.append(nb_line)
                continue

            raise ValueError(f"Line {nb_line} unknown : {line!r}")

        if self.nb_drones is None:
            raise ValueError("Missing 'nb_drones:' line in map file")

        self._validate_connections()
        return Map_format(
            nb_drones=self.nb_drones,
            hubs=self.hubs,
            connections=self.connections,
        )
