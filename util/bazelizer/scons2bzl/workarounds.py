# Copyright 2025 Google, Inc.
# All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""The data used by various workarounds from the implementation.

Ideally this file should be empty.
"""

# See "Dealing with circular dependency" in build_tools/bazel/README.md
hdrs_cut_circular_dep = [
    "base/logging.hh",
    "base/imgwriter.hh",
    "base/statistics.hh",
    "base/loader/object_file.hh",
    "sim/probe/probe.hh",
    "sim/sim_object.hh",
    "sim/system.hh",
    "sim/sub_system.hh",
    "sim/process.hh",
    "sim/workload.hh",  # or sim/debug.hh
    "sim/fd_array.hh",
    "sim/mem_state.hh",
    "sim/vma.hh",
    "sim/clocked_object.hh",
    "sim/clock_domain.hh",
    "sim/power_state.hh",  # or sim/power_domain.hh
    "sim/power/power_model.hh",
    "sim/power/thermal_domain.hh",
    "mem/se_translating_port_proxy.hh",
    "mem/cache/compressors/base_delta.hh",
    "mem/cache/compressors/base.hh",
    "mem/cache/base.hh",
    "mem/cache/mshr.hh",
    "mem/cache/write_queue_entry.hh",
    "mem/qos/mem_ctrl.hh",
    "mem/packet.hh",
    "mem/mem_ctrl.hh",
    "mem/port_proxy.hh",
    "mem/page_table.hh",
    "mem/abstract_mem.hh",  # or mem/physical.hh
    "cpu/pc_event.hh",
    "cpu/thread_context.hh",
    "cpu/simple_thread.hh",
    "cpu/minor/cpu.hh",
    "cpu/minor/pipeline.hh",
    "cpu/minor/fetch1.hh",
    "cpu/minor/fetch2.hh",
    "cpu/minor/lsq.hh",
    "cpu/minor/decode.hh",
    "cpu/minor/execute.hh",
    "cpu/o3/dyn_inst.hh",
    "cpu/o3/regfile.hh",
    "cpu/o3/lsq_unit.hh",
    "cpu/o3/cpu.hh",
    "cpu/o3/lsq.hh",
    "cpu/o3/fetch.hh",
    "cpu/o3/rename_map.hh",
    "cpu/o3/inst_queue.hh",
    "cpu/o3/mem_dep_unit.hh",
    "cpu/o3/rob.hh",
    "cpu/o3/iew.hh",
    "cpu/o3/commit.hh",
    "cpu/o3/decode.hh",
    "cpu/o3/rename.hh",
    "cpu/o3/thread_state.hh",
    "cpu/simple/base.hh",
    "cpu/base.hh",  # or cpu/simple_thread.hh
    "cpu/kvm/vm.hh",
    "arch/generic/mmu.hh",
    "arch/riscv/isa.hh",
    "arch/riscv/pmp.hh",
    "arch/riscv/tlb.hh",
    "arch/riscv/mmu.hh",
    "arch/riscv/pma_checker.hh",
    "dev/net/etherpkt.hh",
    "dev/pci/device.hh",
    "dev/pci/host.hh",
    "dev/storage/ide_ctrl.hh",
]

# These subdirectories are skipped recursively
# This implies that, for example, RUBY cannot be enabled
exclude_recursive = [
    # No need to traverse
    "build",
    "ext",
    "util",
    # Not yet implemented
    "src/arch/arm/fastmodel",
    "src/mem/ruby",
    "src/systemc",
    "src/test_objects",
]

# These subdirectories are skipped
exclude = [
    # Not yet implemented
    "src",
    "src/arch/x86",
]

# These SimObjects are excluded from linking
exclude_sim_objects = [
    "//src/dev/arm:sim_objects",  # TODO(hchsiao): add arch/arm:... to main
    "//src/arch/x86:sim_objects",
    "//src/arch/sparc:sim_objects",
    "//src/arch/arm:sim_objects",
    "//src/arch/power:sim_objects",
    "//src/arch/mips:sim_objects",
]

# These objects are excluded from linking
exclude_lib = [
    "//src/systemc/channel:lib",
    "//src/systemc/dt/fx:lib",
    "//src/systemc/dt/bit:lib",
    "//src/systemc/dt:lib",
    "//src/systemc/dt/int:lib",
    "//src/systemc/dt/misc:lib",
    "//src/systemc/tlm_bridge:lib",
    "//src/systemc/python:lib",
    "//src/systemc/utils:lib",
    "//src/systemc/tests:lib",
    "//src/systemc/tlm_utils:lib",
    "//src/systemc/tlm_core/2/generic_payload:lib",
    "//src/systemc/tlm_core/2/quantum:lib",
    "//src/systemc/core:lib",
    "//src/systemc:lib",
    "//src/base/gtest:lib",  # TODO(hchsiao): Need to omit mock impl
    "//src/dev/arm:lib",  # TODO(hchsiao): Needs GdbXml()
    "//src/dev/riscv:lib",
    "//src/arch/riscv:lib",
    "//src/arch/x86:lib",
    "//src/arch/sparc:lib",
    "//src/arch/arm:lib",
    "//src/arch/power:lib",
    "//src/arch/mips:lib",
]


def merge_config_settings(tags_config_setting, cond_config_setting):
    """Workaround."""
    config_setting = tags_config_setting
    if isinstance(cond_config_setting, str):
        if isinstance(config_setting, str):
            if (
                config_setting == "use_arm_isa_is_true"
                and cond_config_setting
                == "ruby_protocol_mesi_three_level_htm_is_true"
            ):
                cond_config_setting = config_setting
            if (
                config_setting == "use_arm_isa_is_true"
                and cond_config_setting == "use_capstone_is_true"
            ):
                cond_config_setting = config_setting
            if config_setting == "use_x86_isa_is_true":
                cond_config_setting = config_setting
            assert config_setting == cond_config_setting
        elif config_setting:
            config_setting = cond_config_setting
    elif not cond_config_setting:
        config_setting = False
    return config_setting


def hack_sim_object(f):
    def _f(self, spec):
        config_setting = self.get_precondition_from_cond(spec["condition"])
        assert config_setting
        if config_setting == "_is_false":
            pass
        else:
            f(self, spec)

    return _f


def sim_object_param_filter(py_name):
    return py_name in ["SimObject", "Root", "SimpleObject"]
