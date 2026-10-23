# SPDX-License-Identifier: BSD-3-Clause
"""Centralized ARM A9 technology and CPU proxy defaults."""

# main.py's ARM_A9 (40nm/LOP, embedded, single-core) validation anchor.
_BASE_MACHINE_PARAMS = {
    "num_cores": 1,
    "num_l2s": 1,
    "private_l2": 0,
    "num_l1_dirs": 2,
    "num_l2_dirs": 0,
    "num_nocs": 1,
    "tech_node": 40,
    "temperature": 300,
    "device_type": 2,
    "interconnect_type": 1,
    "longer_chan_dev": 1,
    "embedded": 1,
    "machine_bits": 32,
    "phy_addr_width": 32,
    "virt_addr_width": 32,
    "vm_pg_size": 4096,
}

_BASE_CORE_PARAMS_INORDER = {
    "clock_rate": 2000,
    "inst_len": 32,
    "opcode_width": 7,
    "isX86": 0,
    "micro_opcode_width": 8,
    "machine_type": 1,
    "num_threads": 1,
    "fetch_width": 2,
    "num_ifetch_ports": 1,
    "decode_width": 2,
    "issue_width": 4,
    "peak_issue_width": 7,
    "commit_width": 4,
    "fp_issue_width": 1,
    "prediction_width": 1,
    "pipelines_per_core": "1,1",
    "pipeline_depth": "8,8",
    # The anchor XMLs have ALU_per_core=1; the live bridge falls back to this
    # value when the CPU has no readable FU pool (e.g. TimingSimpleCPU).
    "alus_per_core": 1,
    "muls_per_core": 1,
    "fpu_per_core": 1,
    "inst_buffer_size": 32,
    "dec_stream_buffer_sz": 16,
    "inst_window_scheme": 0,
    "inst_window_size": 20,
    "fp_inst_window_size": 15,
    "rob_size": 0,
    "archi_regs_irf_size": 32,
    "archi_regs_frf_size": 32,
    "phys_regs_irf_size": 64,
    "phys_regs_frf_size": 64,
    "rename_scheme": 1,
    "chkpt_depth": 1,
    "reg_window_size": 0,
    "lsu_order": "inorder",
    "store_buffer_sz": 4,
    "load_buffer_sz": 1,
    "mem_ports": 1,
    "ras_sz": 4,
}

_BASE_CORE_PARAMS_O3 = dict(_BASE_CORE_PARAMS_INORDER)
_BASE_CORE_PARAMS_O3.update(
    {
        "machine_type": 0,
        "rob_size": 32,
        "phys_regs_irf_size": 64,
        "phys_regs_frf_size": 64,
        # RAM-based RAT, as the O3 anchor XMLs (and gem5's direct-indexed
        # rename map); the RENAMINGU arrays branch on it.
        "rename_scheme": 0,
    }
)


def _is_ooo(cpu_type):
    """True for the one gem5 cpu_type this bridge treats as McPAT's OOO core
    shape; every `cpu_type == "o3"` check in this package goes through it."""
    return cpu_type == "o3"


# Timing and Minor both use McPAT's Inorder shape and differ in pipeline_depth
# (Timing has no discrete stages; Minor has Fetch1/Fetch2/Decode/Execute).
_BASE_CORE_PARAMS_TIMING = dict(_BASE_CORE_PARAMS_INORDER)
_BASE_CORE_PARAMS_TIMING["pipeline_depth"] = "1,1"

_BASE_CORE_PARAMS_MINOR = dict(_BASE_CORE_PARAMS_INORDER)
_BASE_CORE_PARAMS_MINOR["pipeline_depth"] = "4,4"

# Explicit so it cannot drift with the inorder default; every O3 anchor XML
# carries "8,8".
_BASE_CORE_PARAMS_O3["pipeline_depth"] = "8,8"


def _base_core_params(cpu_type):
    """Base McPAT core-param dict for a gem5 cpu_type. Returns the module
    dict itself; callers must dict()-copy before mutating."""
    if cpu_type == "o3":
        return dict(_BASE_CORE_PARAMS_O3)
    if cpu_type == "minor":
        return dict(_BASE_CORE_PARAMS_MINOR)
    if cpu_type == "timing":
        return dict(_BASE_CORE_PARAMS_TIMING)
    raise ValueError(
        f"unknown cpu_type {cpu_type!r} (expected 'timing'/'minor'/'o3')"
    )
