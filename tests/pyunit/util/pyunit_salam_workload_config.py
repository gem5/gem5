# Copyright (c) 2026 Akanksha Chaudhari
# All rights reserved.
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

import ast
import contextlib
import io
import os
import sys
import tempfile
import unittest

_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), *[os.pardir] * 3)
)
sys.path.insert(
    0, os.path.join(_ROOT, "util", "SALAM-tools", "SALAM-Configurator")
)

import config_parser
import workload_config

_BENCH = os.path.join(
    _ROOT, "configs", "example", "gem5_library", "salam-benchmarks"
)


def _accelerator(name, extra=()):
    lines = [
        "    - Accelerator:",
        f"          - Name: {name}",
        f"            IrPath: hw/{name}.ll",
        "            Profile: k",
        "            PIOSize: 5",
        "            PIOMaster: LocalBus",
    ]
    return lines + [f"            {line}" for line in extra]


def _variable(lines):
    return ["          - Var:"] + [f"                {line}" for line in lines]


class SalamWorkloadConfigTestSuite(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, rel, lines):
        path = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        return path

    def config(self, hw_config=(), accel_extra=(), variables=(), clocks=()):
        lines = ["acc_cluster:", "    - Name: clstr"]
        lines += list(clocks)
        lines += _accelerator("Kernel", accel_extra)
        for var in variables:
            lines += _variable(var)
        lines += _accelerator("Other")
        return self.write("config.yml", lines + list(hw_config))

    def counts(self):
        out = {}
        for cluster in config_parser.load_clusters(self.root):
            for acc in cluster.accs:
                out[acc.name] = config_parser.resolve_cycle_counts(
                    acc.hw_config_document,
                    acc.hw_config_path,
                    acc.profile_name,
                    acc.hw_config_source,
                )
        return out

    def rejects(self, message, **kwargs):
        self.config(**kwargs)
        with self.assertRaises(workload_config.WorkloadConfigError) as ctx:
            self.counts()
        self.assertIn(message, str(ctx.exception))
        return ctx.exception

    def test_tracked_bfs_configs_load(self):
        for kind in ("src", "workloads"):
            root = os.path.join(_BENCH, kind, "bfs")
            with contextlib.redirect_stderr(io.StringIO()):
                clusters = config_parser.load_clusters(root)
            accs = {a.name: a for c in clusters for a in c.accs}
            counts = {
                name: config_parser.resolve_cycle_counts(
                    a.hw_config_document,
                    a.hw_config_path,
                    a.profile_name,
                    a.hw_config_source,
                )
                for name, a in accs.items()
            }
            self.assertIsNone(counts["top"], kind)
            self.assertEqual(counts["bfs"]["add"], 1, kind)

    def test_null_and_partial_profiles(self):
        self.config(
            hw_config=[
                "hw_config:",
                "    k:",
                "        instructions:",
                "            fadd:",
                "                runtime_cycles: 7",
                "                functional_unit: 6",
            ]
        )
        self.assertEqual(self.counts()["kernel"], {"fadd": 7})
        self.config(hw_config=["hw_config:", "    k:"])
        self.assertIsNone(self.counts()["kernel"])

    def _entry(self, name, *fields):
        return [
            "hw_config:",
            "    k:",
            "        instructions:",
            f"            {name}:",
        ] + [f"                {field}" for field in fields]

    def test_rejects_invalid_instruction_entries(self):
        cases = [
            (self._entry("fneg", "runtime_cycles: 1"), "unknown instruction"),
            (self._entry("add", "runtime_cycles: true"), "must be an integer"),
            (self._entry("add", "runtime_cycles: -1"), "must be an integer"),
            (self._entry("add", "runtime_cycles: 1.5"), "must be an integer"),
            (self._entry("add", "runtime_cycles: '3'"), "must be an integer"),
            (
                self._entry("add", "runtime_cycles: 4294967296"),
                "must be an integer",
            ),
            (self._entry("add", "opcode_num: 13"), "has no runtime_cycles"),
            (
                self._entry("add", "runtime_cycles: 1", "latency: 2"),
                "unknown field 'latency'",
            ),
            (
                [
                    "hw_config:",
                    "    k:",
                    "        instructions:",
                    "            add: 3",
                ],
                "must be a mapping",
            ),
            (
                ["hw_config:", "    k:", "        instructions: [add]"],
                "instructions mapping",
            ),
            (["hw_config:", "    k: {}"], "instructions mapping"),
            (
                [
                    "hw_config:",
                    "    k:",
                    "        instructions: {}",
                    "        extra: 1",
                ],
                "instructions mapping",
            ),
            (["hw_config: [k]"], "mapping of profiles"),
        ]
        for hw_config, message in cases:
            with self.subTest(message=message, hw_config=hw_config):
                self.rejects(message, hw_config=hw_config)

    def test_rejects_memory_instruction_latency(self):
        for name in ("load", "store"):
            with self.subTest(name=name):
                self.rejects(
                    f"nonzero runtime_cycles for {name!r} is not supported",
                    hw_config=self._entry(name, "runtime_cycles: 1"),
                )
        self.config(hw_config=self._entry("load", "runtime_cycles: 0"))
        self.assertEqual(self.counts()["kernel"], {"load": 0})

    def test_external_hwpath_uses_the_same_rules(self):
        self.write("t/cycles.yml", self._entry("add", "runtime_cycles: -2"))
        err = self.rejects(
            "must be an integer", accel_extra=["HWPath: t/cycles.yml"]
        )
        self.assertEqual(err.path, os.path.join(self.root, "t", "cycles.yml"))

    def test_register_bank_rejects_extra_connections(self):
        self.rejects(
            "unsupported field Connections",
            variables=[
                [
                    "- Name: R",
                    "  Type: RegisterBank",
                    "  Size: 8",
                    "  Connections: Other:1",
                ]
            ],
        )

    def _spm(self, connections):
        return [
            [
                "- Name: S",
                "  Type: SPM",
                "  Size: 64",
                f"  Connections: {connections}",
            ]
        ]

    def test_spm_connections(self):
        for value, message in (
            ("Other", "must be Name:N"),
            ("Other:0", "must be Name:N"),
            ("Other:x", "must be Name:N"),
            (":2", "must be Name:N"),
            ("'3'", "must be Name:N"),
            ("Ghost:1", "unknown accelerator 'Ghost'"),
        ):
            with self.subTest(value=value):
                self.rejects(message, variables=self._spm(value))
        self.config(variables=self._spm("other:2"))
        clusters = config_parser.load_clusters(self.root)
        spm = [
            v
            for c in clusters
            for a in c.accs
            for v in a.variables
            if v.name == "S"
        ][0]
        self.assertEqual(
            [(c.conName, c.numPorts) for c in spm.connections],
            [("Kernel", 1), ("other", 2)],
        )

    def clocks(self, lines):
        self.config(clocks=lines)
        (cluster,) = config_parser.load_clusters(self.root)
        return cluster.clocks

    def test_clocks_default_and_override(self):
        self.assertEqual(
            self.clocks([]), {"Compute": "100MHz", "Fabric": "1GHz"}
        )
        self.assertEqual(
            self.clocks(["    - Clocks:", "          Compute: 50MHz"]),
            {"Compute": "50MHz", "Fabric": "1GHz"},
        )
        self.assertEqual(
            self.clocks(
                [
                    "    - Clocks:",
                    "          Compute: 2.5ns",
                    "          Fabric: 2GHz",
                ]
            ),
            {"Compute": "2.5ns", "Fabric": "2GHz"},
        )

    def test_clocks_rejections(self):
        for lines, message in (
            (["    - Clocks: 100MHz"], "Clocks must be a mapping"),
            (
                ["    - Clocks:", "          Memory: 1GHz"],
                "unknown field 'Memory' on Clocks",
            ),
            (
                ["    - Clocks:", "          Compute: 100"],
                "Clocks Compute must be a positive frequency",
            ),
            (
                ["    - Clocks:", "          Fabric: 0GHz"],
                "Clocks Fabric must be a positive frequency",
            ),
            (
                ["    - Clocks:", "          Compute: 100 MHz"],
                "Clocks Compute must be a positive frequency",
            ),
            (
                ["    - Clocks:", "          Compute: 10"],
                "Clocks Compute must be a positive frequency",
            ),
        ):
            with self.subTest(lines=lines):
                self.config(clocks=lines)
                with self.assertRaises(
                    workload_config.WorkloadConfigError
                ) as ctx:
                    config_parser.load_clusters(self.root)
                self.assertIn(message, str(ctx.exception))

    def test_legacy_accelerator_keys_are_ignored_with_a_warning(self):
        self.config(
            hw_config=["hw_config:", "    k:"],
            accel_extra=["TopName: top", "ClockPeriod_ns: 0.1"],
        )
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.counts()
        self.assertIn("ignoring TopName", stderr.getvalue())
        self.assertIn(
            "ignoring ClockPeriod_ns; set the cluster's Clocks Compute clock",
            stderr.getvalue(),
        )

    def test_cycle_count_params_match_simobject(self):
        path = os.path.join(_ROOT, "src", "salam", "CycleCounts.py")
        tree = ast.parse(open(path).read())
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "CycleCounts":
                for stmt in node.body:
                    if (
                        isinstance(stmt, ast.Assign)
                        and isinstance(stmt.value, ast.Call)
                        and "Param" in ast.dump(stmt.value.func)
                    ):
                        names.add(stmt.targets[0].id)
        self.assertEqual(names, set(workload_config.CYCLE_COUNT_PARAMS))
