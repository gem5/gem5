#!/usr/bin/env python3
# Copyright (c) 2026 The Regents of the University of California
# All rights reserved.

"""Tests for x86-multicore-soc-hello.py configuration script."""

import os
import sys
import tempfile
import types
import unittest

# Mock gem5 modules before importing
CPUTypes = types.SimpleNamespace()
CPUTypes.KVM = "kvm"
CPUTypes.ATOMIC = "atomic"
CPUTypes.TIMING = "timing"
CPUTypes.O3 = "o3"
CPUTypes.MINOR = "minor"

# Create mock gem5 modules
sys.modules["gem5"] = types.ModuleType("gem5")
sys.modules["gem5.coherence_protocol"] = types.ModuleType(
    "gem5.coherence_protocol"
)
sys.modules["gem5.coherence_protocol"].CoherenceProtocol = (
    types.SimpleNamespace()
)
sys.modules["gem5.coherence_protocol"].CoherenceProtocol.MESI_TWO_LEVEL = (
    "mesi_two_level"
)
sys.modules["gem5.components"] = types.ModuleType("gem5.components")
sys.modules["gem5.components.boards"] = types.ModuleType(
    "gem5.components.boards"
)
sys.modules["gem5.components.boards.x86_board"] = types.ModuleType(
    "gem5.components.boards.x86_board"
)
sys.modules["gem5.components.boards.x86_board"].X86Board = type(
    "X86Board", (), {}
)
sys.modules["gem5.components.cachehierarchies"] = types.ModuleType(
    "gem5.components.cachehierarchies"
)
sys.modules["gem5.components.cachehierarchies.ruby"] = types.ModuleType(
    "gem5.components.cachehierarchies.ruby"
)
sys.modules[
    "gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy"
] = types.ModuleType(
    "gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy"
)
sys.modules[
    "gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy"
].MESITwoLevelCacheHierarchy = type("MESITwoLevelCacheHierarchy", (), {})
sys.modules["gem5.components.memory"] = types.ModuleType(
    "gem5.components.memory"
)
sys.modules["gem5.components.memory.single_channel"] = types.ModuleType(
    "gem5.components.memory.single_channel"
)
sys.modules["gem5.components.memory.single_channel"].SingleChannelDDR4_2400 = (
    type("SingleChannelDDR4_2400", (), {})
)
sys.modules["gem5.components.processors"] = types.ModuleType(
    "gem5.components.processors"
)
sys.modules["gem5.components.processors.cpu_types"] = types.ModuleType(
    "gem5.components.processors.cpu_types"
)
sys.modules["gem5.components.processors.cpu_types"].CPUTypes = CPUTypes
sys.modules["gem5.components.processors.simple_switchable_processor"] = (
    types.ModuleType("gem5.components.processors.simple_switchable_processor")
)
sys.modules[
    "gem5.components.processors.simple_switchable_processor"
].SimpleSwitchableProcessor = type("SimpleSwitchableProcessor", (), {})
sys.modules["gem5.isas"] = types.ModuleType("gem5.isas")
sys.modules["gem5.isas"].ISA = types.SimpleNamespace()
sys.modules["gem5.isas"].ISA.X86 = "x86"
sys.modules["gem5.resources"] = types.ModuleType("gem5.resources")
sys.modules["gem5.resources.resource"] = types.ModuleType(
    "gem5.resources.resource"
)
sys.modules["gem5.resources.resource"].obtain_resource = lambda x: None
sys.modules["gem5.simulate"] = types.ModuleType("gem5.simulate")
sys.modules["gem5.simulate.exit_event"] = types.ModuleType(
    "gem5.simulate.exit_event"
)
sys.modules["gem5.simulate.exit_event"].ExitEvent = types.SimpleNamespace()
sys.modules["gem5.simulate.exit_event"].ExitEvent.EXIT = "exit"
sys.modules["gem5.simulate.simulator"] = types.ModuleType(
    "gem5.simulate.simulator"
)
sys.modules["gem5.simulate.simulator"].Simulator = type("Simulator", (), {})
sys.modules["gem5.utils"] = types.ModuleType("gem5.utils")
sys.modules["gem5.utils.requires"] = types.ModuleType("gem5.utils.requires")
sys.modules["gem5.utils.requires"].requires = lambda **kwargs: None

# Add configs directory to path and import the script
import importlib.util

spec = importlib.util.spec_from_file_location(
    "x86_multicore_soc_hello",
    os.path.join(
        os.path.dirname(__file__),
        "../../configs/example/gem5_library/x86-multicore-soc-hello.py",
    ),
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

parse_args = module.parse_args
get_cpu_type = module.get_cpu_type
create_hello_world_workload = module.create_hello_world_workload


class TestMultiCoreSoCConfig(unittest.TestCase):
    """Test multi-core SoC configuration."""

    def test_parse_default_args(self):
        """Test parsing default arguments."""
        old_argv = sys.argv
        try:
            sys.argv = ["x86-multicore-soc-hello.py"]
            args = parse_args()
            self.assertEqual(args.num_cores, 2)
            self.assertEqual(args.boot_cpu, "kvm")
            self.assertEqual(args.exec_cpu, "timing")
            self.assertEqual(args.l1d_size, "32KiB")
            self.assertEqual(args.l1i_size, "32KiB")
            self.assertEqual(args.l2_size, "512KiB")
            self.assertEqual(args.memory_size, "3GiB")
            self.assertEqual(args.clk_freq, "3GHz")
            self.assertFalse(args.no_switch)
            self.assertFalse(args.kvm_perf)
        finally:
            sys.argv = old_argv

    def test_parse_custom_args(self):
        """Test parsing custom arguments."""
        test_args = [
            "x86-multicore-soc-hello.py",
            "--num-cores",
            "4",
            "--boot-cpu",
            "atomic",
            "--exec-cpu",
            "o3",
            "--no-switch",
            "--l1d-size",
            "64KiB",
            "--l1i-size",
            "64KiB",
            "--l2-size",
            "1MiB",
            "--memory-size",
            "8GiB",
            "--clk-freq",
            "4GHz",
            "--kvm-perf",
        ]

        old_argv = sys.argv
        try:
            sys.argv = test_args
            args = parse_args()
            self.assertEqual(args.num_cores, 4)
            self.assertEqual(args.boot_cpu, "atomic")
            self.assertEqual(args.exec_cpu, "o3")
            self.assertTrue(args.no_switch)
            self.assertEqual(args.l1d_size, "64KiB")
            self.assertEqual(args.l1i_size, "64KiB")
            self.assertEqual(args.l2_size, "1MiB")
            self.assertEqual(args.memory_size, "8GiB")
            self.assertEqual(args.clk_freq, "4GHz")
            self.assertTrue(args.kvm_perf)
        finally:
            sys.argv = old_argv

    def test_get_cpu_type(self):
        """Test CPU type mapping."""
        self.assertEqual(get_cpu_type("kvm"), CPUTypes.KVM)
        self.assertEqual(get_cpu_type("atomic"), CPUTypes.ATOMIC)
        self.assertEqual(get_cpu_type("timing"), CPUTypes.TIMING)
        self.assertEqual(get_cpu_type("o3"), CPUTypes.O3)
        self.assertEqual(get_cpu_type("minor"), CPUTypes.MINOR)

    def test_create_hello_world_workload(self):
        """Test hello world workload creation."""
        workload_path = create_hello_world_workload()

        # Check file exists
        self.assertTrue(os.path.exists(workload_path))

        # Check file is executable
        self.assertTrue(os.access(workload_path, os.X_OK))

        # Check content
        with open(workload_path) as f:
            content = f.read()
            self.assertIn("Hello from gem5 Multi-CPU X86 SoC!", content)
            self.assertIn("m5 exit", content)

        # Cleanup
        os.unlink(workload_path)


if __name__ == "__main__":
    unittest.main()
