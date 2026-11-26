# SPDX-License-Identifier: BSD-3-Clause
"""Shared array search and fixed-organization conversion."""

from .array_spec import CactiArraySpec
from .cacti_memory import CactiMemory


def _ports(cfg):
    return tuple(
        cfg[k]
        for k in (
            "num_rw_ports",
            "num_rd_ports",
            "num_wr_ports",
            "num_search_ports",
        )
    )


class _ArraySearch:
    """Partition searches at one `sys_params` and core type, with McPAT's
    Layer-3 gate inputs (`opt_for_clk`, `opt_local`; both off is the plain
    Layer-1/2 search). Raises on a rejected geometry."""

    def __init__(
        self,
        sys_params,
        core_ooo,
        opt_for_clk=False,
        opt_local=False,
        timing_log=None,
        buffer_policy="mcpat",
        tag_widths=None,
    ):
        self.timing_log = [] if timing_log is None else timing_log
        self.buffer_policy = buffer_policy
        self.tag_widths = dict(tag_widths or {})
        if set(self.tag_widths) - {"icache", "dcache", "L2"} or any(
            not isinstance(v, int) or v <= 0 for v in self.tag_widths.values()
        ):
            raise ValueError(
                "tag_widths requires positive bit counts for icache/dcache/L2"
            )
        self.sys_params = sys_params
        self.config = sys_params._machine_config._config_params
        self.core_ooo = core_ooo
        self.opt_for_clk = opt_for_clk
        self.opt_local = opt_local

    def cycles(self, throughput_cycles, latency_cycles=None):
        """Layer-3 targets `N / clockRate` seconds at the core clock (core.cc
        `1.0/clockRate`); empty when opt_for_clk is off."""
        if not self.opt_for_clk:
            return {}
        if latency_cycles is None:
            latency_cycles = throughput_cycles
        clk = self.config["core0"]["clock_rate"] * 1e6
        return dict(
            throughput=throughput_cycles / clk, latency=latency_cycles / clk
        )

    def _log_timing(self, name, mem, throughput, latency):
        """Appends the array's Layer-3 outcome (read-only) to `timing_log`.
        McPAT prints its "cannot satisfy" warnings only for assoc > 0
        (array.cc:176-181); CAM/fully-associative rows can miss silently."""
        row = dict(
            mem.timing,
            name=name,
            assoc=mem._geom["assoc"],
            throughput=throughput,
        )
        row["latency"] = latency
        row["satisfied"] = row["throughput_ok"] and row["latency_ok"]
        self.timing_log.append(row)

    def _memory(self, core_ooo, **kwargs):
        return CactiMemory(
            _sys_params=self.sys_params,
            _core_ooo=self.core_ooo if core_ooo is None else core_ooo,
            _partition="search",
            _opt_for_clk=self.opt_for_clk,
            _opt_local=self.opt_local,
            **kwargs,
        )

    def array(
        self,
        name,
        capacity,
        block_sz,
        ports,
        tag_w=0,
        out_w=None,
        assoc=None,
        specific_tag=None,
        device_ty="core",
        core_ooo=None,
        wire_is_mat_type=None,
        wire_os_mat_type=None,
        throughput=None,
        latency=None,
    ):
        """CactiArraySpec searched on a live-derived config, bypassing the
        partition table. `ports` is (rw, rd, wr, search); a `tag_w` > 0 makes
        a CAM (assoc 0, specific tag), else a RAM (assoc 1). `capacity` gets
        ArrayST's 64-byte minimum (array.cc:63). A None `wire_is_mat_type`
        takes the Embedded-derived setting."""
        rw, rd, wr, search = ports
        mem = self._memory(
            core_ooo,
            _capacity=max(capacity, 64),
            _block_sz=block_sz,
            _assoc=(0 if tag_w else 1) if assoc is None else assoc,
            _out_w=block_sz * 8 if out_w is None else out_w,
            _tag_w=tag_w,
            _specific_tag=tag_w > 0 if specific_tag is None else specific_tag,
            _has_tag=False,
            _num_rw_ports=rw,
            _num_rd_ports=rd,
            _num_wr_ports=wr,
            _num_search_ports=search,
            _wire_is_mat_type=wire_is_mat_type,
            _wire_os_mat_type=wire_os_mat_type,
            _device_ty=device_ty,
            _throughput=throughput,
            _latency=latency,
        )
        array = mem.to_array()
        array.name = name
        self._log_timing(name, mem, throughput, latency)
        return array

    def like(
        self,
        anchor,
        entries=None,
        block_sz=None,
        port_overrides=None,
        name=None,
        **timing,
    ):
        """`anchor`'s shape re-searched at `entries` * bytes-per-entry (the
        anchor's capacity when `entries` is None). `block_sz` replaces the
        anchor's block_sz and out_w; `port_overrides` ({"num_rd_ports": ..})
        replaces port counts."""
        cfg = anchor._cfg_kwargs
        block = cfg["block_sz"] if block_sz is None else block_sz
        return self.array(
            anchor.name if name is None else name,
            cfg["capacity"] if entries is None else block * entries,
            block,
            _ports(dict(cfg, **(port_overrides or {}))),
            cfg["tag_w"],
            cfg["out_w"] if block_sz is None else block_sz * 8,
            assoc=cfg["assoc"],
            specific_tag=cfg["specific_tag"],
            device_ty=anchor._device_ty,
            core_ooo=self.core_ooo,
            **timing,
        )

    def cache_pair(
        self,
        name,
        capacity,
        block_sz,
        assoc,
        out_w,
        tag_w,
        ports,
        nbanks=1,
        device_ty="core",
        data_assoc=None,
        wire_os_mat_type=None,
        is_seq_acc=False,
        throughput=None,
        latency=None,
    ):
        """(data, tag) CactiArraySpec pair from a real `tag_cfg` search, the
        two-array shape of machine_model's ICACHE/ICACHE_TAG etc."""
        tag_w = self.tag_widths.get(name, tag_w)
        rw, rd, wr, search = ports
        mem = self._memory(
            None,
            _capacity=capacity,
            _block_sz=block_sz,
            _assoc=assoc,
            _nbanks=nbanks,
            _out_w=out_w,
            _tag_w=tag_w,
            _has_tag=True,
            _num_rw_ports=rw,
            _num_rd_ports=rd,
            _num_wr_ports=wr,
            _num_search_ports=search,
            _data_assoc=data_assoc,
            _wire_os_mat_type=wire_os_mat_type,
            _is_seq_acc=is_seq_acc,
            _device_ty=device_ty,
            _throughput=throughput,
            _latency=latency,
        )
        data_array = mem.to_array()
        data_array.name = name
        self._log_timing(name, mem, throughput, latency)
        wire = data_array._cfg_kwargs
        p = mem.tag_partition
        tag_array = CactiArraySpec(
            name + "_tag",
            capacity=capacity,
            block_sz=block_sz,
            assoc=assoc,
            nbanks=nbanks,
            out_w=out_w,
            wire_is_mat_type=wire["wire_is_mat_type"],
            wire_os_mat_type=wire["wire_os_mat_type"],
            wt_overhead=wire["wt_overhead"],
            data_assoc=data_assoc,
            is_seq_acc=is_seq_acc,
            is_tag=True,
            specific_tag=True,
            tag_w=tag_w,
            num_rw_ports=rw,
            num_rd_ports=rd,
            num_wr_ports=wr,
            num_search_ports=search,
            Nspd=p[0],
            Ndwl=p[1],
            Ndbl=p[2],
            Ndcm=p[3],
            Ndsam_lev_1=p[4],
            Ndsam_lev_2=p[5],
            mat_area_w=mem.tag_mat_area[0],
            mat_area_h=mem.tag_mat_area[1],
            device_ty=device_ty,
            core_ooo=self.core_ooo,
        )
        return data_array, tag_array

    def anchor_cache_pair(self, name, anchor, wire_os_mat_type=None):
        """`anchor` Cache's data/tag shape re-searched at this node."""
        c = anchor.data_array._cfg_kwargs
        return self.cache_pair(
            name,
            c["capacity"],
            c["block_sz"],
            c["assoc"],
            c["out_w"],
            anchor.tag_array._cfg_kwargs["tag_w"],
            _ports(c),
            nbanks=c["nbanks"],
            device_ty=anchor.data_array._device_ty,
            data_assoc=c["data_assoc"],
            wire_os_mat_type=wire_os_mat_type,
            is_seq_acc=c["is_seq_acc"],
        )
