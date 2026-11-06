# SPDX-License-Identifier: BSD-3-Clause
"""Immutable array inputs. Bytes, bits, kelvin, nanometres and seconds."""

from dataclasses import dataclass
from types import SimpleNamespace

from .cacti_params import CactiParams


@dataclass(frozen=True)
class Technology:
    node_nm: float = 40
    temperature: float = 300
    device_type: int = 2
    interconnect_type: int = 1
    embedded: bool = True
    longer_channel: bool = True
    core_ooo: bool = False
    vdd: float = 0

    def parameters(self, temperature=None):
        t = self.temperature if temperature is None else temperature
        if not 300 <= t <= 400 or t % 10:
            raise ValueError(
                "technology temperature must be a 300–400 K sample"
            )
        if self.device_type not in (0, 1, 2):
            raise ValueError("device_type must be HP=0, LSTP=1 or LOP=2")
        if self.interconnect_type not in (0, 1):
            raise ValueError("interconnect_type must be 0 or 1")
        if self.node_nm > 90 and self.device_type != 0:
            raise ValueError("LSTP/LOP technology is available through 90 nm")
        config = dict(
            tech_node=self.node_nm,
            temperature=t,
            device_type=self.device_type,
            interconnect_type=self.interconnect_type,
            embedded=int(self.embedded),
            longer_chan_dev=int(self.longer_channel),
            vdd=self.vdd,
            core0={"machine_type": 0 if self.core_ooo else 1},
        )
        return CactiParams(SimpleNamespace(_config_params=config))


@dataclass(frozen=True)
class Partition:
    nspd: float = 1
    ndwl: int = 1
    ndbl: int = 1
    ndcm: int = 1
    ndsam1: int = 1
    ndsam2: int = 1
    mat_width: float = 0
    mat_height: float = 0

    @property
    def integers(self):
        return (
            self.nspd,
            self.ndwl,
            self.ndbl,
            self.ndcm,
            self.ndsam1,
            self.ndsam2,
        )


@dataclass(frozen=True)
class SearchOptions:
    opt_for_clk: bool = False
    opt_local: bool = False
    throughput: float | None = None
    latency: float | None = None
    scaling: str = "raw"
    partition: Partition | None = None

    def validate(self):
        if self.scaling not in ("raw", "mcpat"):
            raise ValueError("scaling must be raw or mcpat")
        if self.opt_for_clk and self.opt_local:
            if not self.throughput or not self.latency:
                raise ValueError("timing optimization requires timing targets")
            if self.throughput <= 0 or self.latency <= 0:
                raise ValueError("timing targets must be positive seconds")


@dataclass(frozen=True)
class ArrayConfig:
    capacity: int = 32768
    line_bytes: int = 64
    data_width: int = 512
    tag_width: int = 0
    associativity: int = 1
    banks: int = 1
    rw_ports: int = 1
    read_ports: int = 0
    write_ports: int = 0
    search_ports: int = 0
    pure_cam: bool = False
    has_tag: bool = False
    is_tag: bool = False
    add_ecc: bool = True
    data_associativity: int | None = None
    sequential_access: bool = False
    fast_access: bool = False
    wire_inside: int | None = None
    wire_outside: int | None = None
    wire_overhead: int | None = None
    device: str = "core"

    def validate(self):
        for key in ("capacity", "line_bytes", "data_width", "banks"):
            value = getattr(self, key)
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f"{key} must be a positive integer")
        if self.capacity < 64:
            raise ValueError("capacity must be at least 64 bytes")
        if self.capacity < self.line_bytes * self.banks:
            raise ValueError(
                "capacity must contain at least one line per bank"
            )
        if self.associativity < 0 or self.tag_width < 0:
            raise ValueError("associativity and tag_width must be nonnegative")
        ports = (
            self.rw_ports,
            self.read_ports,
            self.write_ports,
            self.search_ports,
        )
        if any(not isinstance(p, int) or p < 0 for p in ports) or not sum(
            ports
        ):
            raise ValueError(
                "ports must be nonnegative with at least one port"
            )
        if self.has_tag and (not self.tag_width or self.associativity == 0):
            raise ValueError(
                "paired cache requires a tag width and associativity"
            )
        if self.pure_cam and (
            self.associativity != 0
            or not self.tag_width
            or self.has_tag
            or not self.search_ports
        ):
            raise ValueError(
                "pure CAM requires assoc=0, tag width, search ports and no paired tag"
            )
        if self.is_tag and not self.tag_width:
            raise ValueError("tag array requires tag_width")
        if self.device not in ("core", "uncore", "llc"):
            raise ValueError("device must be core, uncore or llc")
