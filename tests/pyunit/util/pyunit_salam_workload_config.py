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

import yaml

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

    def test_rejects_duplicate_explicit_yaml_keys(self):
        for content in (
            "hw_config: {k: null, k: null}",
            "hw_config: {k: {instructions: {add: {runtime_cycles: 1, "
            "runtime_cycles: 2}}}}",
            "hw_config: {k: &k {instructions: {}, instructions: {}}}",
            "hw_config: {k: {<<: {instructions: {}}, <<: {instructions: {}}}}",
        ):
            with self.subTest(content=content):
                path = self.write("duplicate.yml", [content])
                with self.assertRaisesRegex(
                    workload_config.WorkloadConfigError,
                    "duplicate explicit key",
                ) as ctx:
                    workload_config.load_documents(path)
                self.assertEqual(ctx.exception.path, path)
                self.assertIn("line 1", str(ctx.exception))
        path = self.write("cycles.yml", ["hw_config: {k: null, k: null}"])
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "duplicate explicit key"
        ):
            config_parser.resolve_cycle_counts(None, path, "k")

    def test_yaml_merges_keep_precedence_without_sharing_mutable_records(self):
        path = self.write(
            "merges.yml",
            [
                "hw_config:",
                "  first: &first {instructions: {add: {runtime_cycles: 2}}}",
                "  second: &second {instructions: {add: {runtime_cycles: 3}}}",
                "  merged: {<<: [*first, *second]}",
                "  explicit:",
                "    <<: *first",
                "    instructions: {add: {runtime_cycles: 5}}",
                "  alias: *first",
            ],
        )
        (document,) = workload_config.load_documents(path)
        nodes = document["hw_config"]
        self.assertEqual(
            nodes["merged"]["instructions"]["add"]["runtime_cycles"], 2
        )
        self.assertEqual(
            nodes["explicit"]["instructions"]["add"]["runtime_cycles"], 5
        )
        nodes["alias"]["instructions"]["add"]["runtime_cycles"] = 9
        self.assertEqual(
            nodes["first"]["instructions"]["add"]["runtime_cycles"], 2
        )
        self.assertEqual(
            nodes["merged"]["instructions"]["add"]["runtime_cycles"], 2
        )

    def test_yaml_errors_have_workload_diagnostics(self):
        for content in ("hw_config: [", "hw_config: &self {k: *self}"):
            with self.subTest(content=content):
                path = self.write("invalid.yml", [content])
                with self.assertRaises(
                    workload_config.WorkloadConfigError
                ) as ctx:
                    workload_config.load_documents(path)
                self.assertEqual(ctx.exception.path, path)

    def test_memory_characterization_is_relative_to_declaring_yaml(self):
        for kind in ("SPM", "RegisterBank"):
            with self.subTest(kind=kind):
                fields = [
                    "- Name: S",
                    f"  Type: {kind}",
                    "  Size: 64",
                    "  Characterization: coefficients/storage.yml",
                ]
                if kind == "SPM":
                    fields.append("  Connections: Kernel:1")
                self.config(
                    hw_config=["hw_config:", "    k:"], variables=[fields]
                )
                clusters = config_parser.load_clusters(self.root)
                variables = [
                    v for c in clusters for a in c.accs for v in a.variables
                ]
                self.assertEqual(len(variables), 1)
                self.assertEqual(
                    variables[0].characterization,
                    os.path.join(self.root, "coefficients", "storage.yml"),
                )

    def test_memory_characterization_must_be_a_nonempty_path(self):
        for value in ("''", "null", "true", "5"):
            with self.subTest(value=value):
                self.rejects(
                    "Characterization must name an offline record",
                    variables=[
                        [
                            "- Name: S",
                            "  Type: SPM",
                            "  Size: 64",
                            "  Connections: Kernel:1",
                            f"  Characterization: {value}",
                        ]
                    ],
                )

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

    def test_rejects_terminator_latency(self):
        for name in ("br", "ret", "switch_inst"):
            with self.subTest(name=name):
                self.rejects(
                    f"nonzero runtime_cycles for {name!r} is not supported",
                    hw_config=self._entry(name, "runtime_cycles: 7"),
                )
                self.config(hw_config=self._entry(name, "runtime_cycles: 0"))
                self.assertEqual(self.counts()["kernel"], {name: 0})
        self.config(hw_config=self._entry("call", "runtime_cycles: 7"))
        self.assertEqual(self.counts()["kernel"], {"call": 7})

    def test_external_hwpath_rejects_terminator_latency(self):
        self.write("t/cycles.yml", self._entry("br", "runtime_cycles: 7"))
        err = self.rejects(
            "terminator latency is not modeled",
            accel_extra=["HWPath: t/cycles.yml"],
            hw_config=["hw_config:", "    k:"],
        )
        self.assertEqual(err.path, os.path.join(self.root, "t", "cycles.yml"))

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
            (
                ["    - Clocks:", '          Compute: "100MHz\\n"'],
                "Clocks Compute must be a positive frequency",
            ),
            (
                ["    - Clocks:", '          Fabric: "10ns\\n"'],
                "Clocks Fabric must be a positive frequency",
            ),
        ):
            with self.subTest(lines=lines):
                self.config(clocks=lines)
                with self.assertRaises(
                    workload_config.WorkloadConfigError
                ) as ctx:
                    config_parser.load_clusters(self.root)
                self.assertIn(message, str(ctx.exception))

    def test_legacy_accelerator_keys_are_rejected(self):
        for field, value, hint in (
            ("TopName", "top", "entry function is top"),
            ("ClockPeriod_ns", "0.1", "Clocks.Compute"),
        ):
            with self.subTest(field=field):
                err = self.rejects(
                    f"unsupported field {field}",
                    hw_config=["hw_config:", "    k:"],
                    accel_extra=[f"{field}: {value}"],
                )
                self.assertEqual(err.field, field)
                self.assertIn(hint, str(err))

    def variable(self, kind, **fields):
        record = {"Name": "V", "Type": kind, **fields}
        self.config(
            hw_config=["hw_config:", "    k:"],
            variables=[yaml.safe_dump([record], sort_keys=False).splitlines()],
        )
        cluster = config_parser.load_clusters(self.root)[0]
        return cluster.accs[0].variables[0]

    def test_memory_sizes_require_positive_integer_bytes(self):
        for kind in ("SPM", "RegisterBank"):
            for value in (None, 0, -1, True, 1.5, "64", 1 << 64):
                with self.subTest(kind=kind, value=value):
                    with self.assertRaises(
                        workload_config.WorkloadConfigError
                    ) as ctx:
                        self.variable(kind, Size=value)
                    self.assertEqual(ctx.exception.field, "Size")
            self.assertEqual(self.variable(kind, Size=17).size, 17)
            with self.assertRaises(workload_config.WorkloadConfigError):
                self.variable(kind)

    def test_cache_size_keeps_supported_prefix_forms(self):
        for value, expected in (
            (32768, 32768),
            ("32768", 32768),
            ("32Ki", 32768),
            ("32k", 32768),
            ("0x8000", 32768),
            ("1Mi", 1 << 20),
            ("+32Ki", 32768),
            ("0o40Ki", 32768),
            ("0b100000Ki", 32768),
        ):
            with self.subTest(value=value):
                self.assertEqual(
                    self.variable("Cache", Size=value).size, expected
                )
        for value in (None, 0, -1, True, 1.5, "1.5Ki", "32KiB", "1KB"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(
                    workload_config.WorkloadConfigError, "Cache Size"
                ):
                    self.variable("Cache", Size=value)

    def test_spm_flags_normalize_without_truthiness(self):
        fields = {
            "ReadyMode": "readyMode",
            "ResetOnRead": "resetOnRead",
            "ReadOnInvalid": "readOnInvalid",
            "WriteOnValid": "writeOnValid",
        }
        default = self.variable("SPM", Size=64)
        self.assertEqual(
            [getattr(default, name) for name in fields.values()],
            [False, True, False, True],
        )
        for field, attr in fields.items():
            for value, expected in (
                (True, True),
                (False, False),
                (1, True),
                (0, False),
                ("yes", True),
                ("FALSE", False),
                ("t", True),
                ("0", False),
            ):
                with self.subTest(field=field, value=value):
                    self.assertIs(
                        getattr(
                            self.variable("SPM", Size=64, **{field: value}),
                            attr,
                        ),
                        expected,
                    )
            for value in (None, "nonsense", 2, [], 1.0):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(
                        workload_config.WorkloadConfigError
                    ) as ctx:
                        self.variable("SPM", Size=64, **{field: value})
                    self.assertEqual(ctx.exception.field, field)

    def test_stream_sizes_defaults_and_endpoints(self):
        fields = dict(StreamSize=8, InCon="KERNEL", OutCon="Other")
        stream = self.variable("Stream", **fields)
        self.assertEqual((stream.streamSize, stream.bufferSize), (8, None))
        self.assertEqual((stream.inCon, stream.outCon), ("kernel", "other"))
        self.assertEqual(
            self.variable("Stream", **fields, BufferSize=512).bufferSize, 512
        )
        for field in ("StreamSize", "BufferSize"):
            for value in (None, 0, -1, True, 1.5, "8", 1 << 64):
                with self.subTest(field=field, value=value):
                    bad = dict(fields, **{field: value})
                    with self.assertRaises(
                        workload_config.WorkloadConfigError
                    ) as ctx:
                        self.variable("Stream", **bad)
                    self.assertEqual(ctx.exception.field, field)
        for field in ("InCon", "OutCon"):
            for value in (None, "", "Ghost", 1):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(
                        workload_config.WorkloadConfigError
                    ) as ctx:
                        self.variable(
                            "Stream", **dict(fields, **{field: value})
                        )
                    self.assertEqual(ctx.exception.field, field)
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "StreamSize"
        ):
            self.variable("Stream", InCon="Kernel", OutCon="Other")

    def test_variable_names_and_child_collisions(self):
        for name in (None, "", "a.b", "bad-name", "_private", 1):
            with self.subTest(name=name):
                with self.assertRaises(
                    workload_config.WorkloadConfigError
                ) as ctx:
                    self.variable("SPM", Name=name, Size=64)
                self.assertEqual(ctx.exception.field, "Name")
        for name in ("Kernel", "other", "Local_Bus", "compute_clk_domain"):
            with self.subTest(name=name):
                with self.assertRaisesRegex(
                    workload_config.WorkloadConfigError, "collides"
                ):
                    self.variable("SPM", Name=name, Size=64)
        # Caches retain their existing case-sensitive ownership name.
        self.assertEqual(
            self.variable("Cache", Name="Kernel", Size="32Ki").name, "Kernel"
        )

    def test_containers_and_repeated_fields_have_schema_errors(self):
        self.config(hw_config=["hw_config:", "    k:"])
        path = os.path.join(self.root, "config.yml")
        with open(path) as f:
            original = yaml.safe_load(f)
        import copy

        for value in (None, {}, "bad", 3):
            document = copy.deepcopy(original)
            document["acc_cluster"][1]["Accelerator"].append({"Var": value})
            self.write("config.yml", yaml.safe_dump(document).splitlines())
            with self.assertRaisesRegex(
                workload_config.WorkloadConfigError, "Var must be a list"
            ):
                config_parser.load_clusters(self.root)
        for field, value in (
            ("Name", "Again"),
            ("PIOSize", 5),
            ("Profile", "k"),
        ):
            document = copy.deepcopy(original)
            document["acc_cluster"][1]["Accelerator"].append({field: value})
            self.write("config.yml", yaml.safe_dump(document).splitlines())
            with self.assertRaisesRegex(
                workload_config.WorkloadConfigError, f"duplicate {field}"
            ):
                config_parser.load_clusters(self.root)
        for container in (None, {}, 1):
            self.write(
                "config.yml",
                yaml.safe_dump({"acc_cluster": container}).splitlines(),
            )
            with self.assertRaisesRegex(
                workload_config.WorkloadConfigError,
                "acc_cluster must be a list",
            ):
                config_parser.load_clusters(self.root)

    def test_allocation_overflow_is_rejected_before_construction(self):
        self.config(hw_config=["hw_config:", "    k:"])
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "64-bit address"
        ):
            config_parser.load_clusters(self.root, base_address=(1 << 64) - 32)
        for base in (-1, True, 1 << 64):
            with self.subTest(base=base):
                with self.assertRaises(workload_config.WorkloadConfigError):
                    config_parser.load_clusters(self.root, base_address=base)
        # The parser can describe a custom location; the ARM builder
        # separately checks its supported routing aperture.
        self.assertEqual(
            config_parser.load_clusters(self.root, base_address=0x1000)[
                0
            ].base_address,
            0x1000,
        )
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "64-bit address"
        ):
            self.variable("SPM", Size=(1 << 64) - 1)

    def test_unaligned_custom_base_keeps_intercluster_alignment(self):
        self.config(hw_config=["hw_config:", "    k:"])
        path = os.path.join(self.root, "config.yml")
        with open(path) as stream:
            original = stream.read()
        second = original.replace("Name: clstr", "Name: second")
        self.write("config.yml", (original + "\n---\n" + second).splitlines())
        first, second = config_parser.load_clusters(
            self.root, base_address=0x1003
        )
        self.assertEqual(first.top_address, 0x1083)
        self.assertEqual(second.base_address, 0x10C0)

    def test_stream_targets_reject_bound_buffers(self):
        self.config(
            hw_config=["hw_config:", "    k:"],
            variables=[
                [
                    "- Name: Flow",
                    "  Type: Stream",
                    "  StreamSize: 8",
                    "  InCon: Kernel",
                    "  OutCon: Other",
                ]
            ],
        )
        path = os.path.join(self.root, "config.yml")
        with open(path) as stream:
            document = yaml.safe_load(stream)
        other = document["acc_cluster"][2]["Accelerator"][0]
        for field in ("StreamIn", "StreamOut"):
            with self.subTest(field=field):
                other[field] = "FLOW"
                self.write("config.yml", yaml.safe_dump(document).splitlines())
                with self.assertRaises(
                    workload_config.WorkloadConfigError
                ) as ctx:
                    config_parser.load_clusters(self.root)
                self.assertEqual(ctx.exception.field, field)
                self.assertIn("unsupported target", str(ctx.exception))
                del other[field]

    def connection_config(self, kernel=None, other=None):
        path = self.config(hw_config=["hw_config:", "    k:"])
        with open(path) as stream:
            document = yaml.safe_load(stream)
        items = document["acc_cluster"]
        items[1]["Accelerator"][0].update(kernel or {})
        items[2]["Accelerator"][0].update(other or {})
        items.append(
            {
                "DMA": [
                    {
                        "Name": name,
                        "Type": kind,
                        "PIOMaster": "LocalBus",
                    }
                    for name, kind in (
                        ("NDMA", "NonCoherent"),
                        ("SDMA", "Stream"),
                    )
                ]
            }
        )
        self.write("config.yml", yaml.safe_dump(document).splitlines())
        return config_parser.load_clusters(self.root)[0]

    def test_local_slaves_require_local_bus(self):
        for target in ("LocalBus", "localbus", "LOCAL_BUS"):
            with self.subTest(target=target):
                cluster = self.connection_config({"LocalSlaves": target})
                self.assertEqual(
                    cluster.accs[0].local_connections, ["LocalBus"]
                )
        for target in ("Kernel", "Other", "NDMA", "SDMA"):
            with self.subTest(target=target):
                with self.assertRaises(
                    workload_config.WorkloadConfigError
                ) as ctx:
                    self.connection_config({"LocalSlaves": target})
                self.assertEqual(ctx.exception.field, "LocalSlaves")
                self.assertIn("unsupported target", str(ctx.exception))

    def test_stream_dma_directions_can_have_distinct_owners(self):
        cluster = self.connection_config(
            {"StreamIn": "sDMA"}, {"StreamOut": "SDMA"}
        )
        self.assertEqual(cluster.accs[0].stream_in, ["sdma"])
        self.assertEqual(cluster.accs[1].stream_out, ["sdma"])
        cluster = self.connection_config(
            {"StreamIn": "sDMA", "StreamOut": "SDMA"}
        )
        self.assertEqual(cluster.accs[0].stream_in, ["sdma"])
        self.assertEqual(cluster.accs[0].stream_out, ["sdma"])

    def test_stream_dma_direction_rejects_duplicate_owners(self):
        for field in ("StreamIn", "StreamOut"):
            for kernel, other in (
                ({field: "SDMA"}, {field: "sdma"}),
                ({field: "SDMA,sdma"}, {}),
            ):
                with self.subTest(field=field, kernel=kernel, other=other):
                    with self.assertRaises(
                        workload_config.WorkloadConfigError
                    ) as ctx:
                        self.connection_config(kernel, other)
                    self.assertEqual(ctx.exception.field, field)
                    self.assertIn("already connected", str(ctx.exception))

    def test_duplicate_cluster_and_accelerator_names(self):
        self.config(hw_config=["hw_config:", "    k:"])
        path = os.path.join(self.root, "config.yml")
        with open(path) as f:
            original = yaml.safe_load(f)
        original["acc_cluster"].extend(
            [
                {
                    "Accelerator": [
                        {
                            "Name": "KERNEL",
                            "Profile": "k",
                            "IrPath": "hw/k.ll",
                            "PIOSize": 5,
                            "PIOMaster": "LocalBus",
                        }
                    ]
                }
            ]
        )
        self.write("config.yml", yaml.safe_dump(original).splitlines())
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "collides"
        ):
            config_parser.load_clusters(self.root)
        self.config(hw_config=["hw_config:", "    k:"])
        with open(path) as f:
            text = f.read()
        self.write("config.yml", (text + "\n---\n" + text).splitlines())
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "duplicate cluster Name"
        ):
            config_parser.load_clusters(self.root)

    def dma(self, dma_type, **fields):
        record = {
            "Name": "dma",
            "Type": dma_type,
            "PIOMaster": "LocalBus",
            **fields,
        }
        self.write(
            "config.yml",
            yaml.safe_dump(
                {"acc_cluster": [{"Name": "cluster"}, {"DMA": [record]}]}
            ).splitlines(),
        )
        return config_parser.load_clusters(self.root)[0].dmas[0]

    def test_dma_defaults_and_explicit_fields(self):
        for dma_type in ("NonCoherent", "Stream"):
            with self.subTest(dma_type=dma_type):
                dma = self.dma(dma_type)
                self.assertIsNone(dma.size)
                self.assertIsNone(dma.maxReq)
                dma = self.dma(dma_type, BufferSize=512, MaxReqSize=64)
                self.assertEqual(dma.size, 512)
                self.assertEqual(dma.maxReq, 64)
        self.assertEqual(self.dma("NonCoherent", InterruptNum=0).int_num, 0)
        dma = self.dma("Stream", ReadInt=0, WriteInt=211)
        self.assertEqual((dma.rd_int, dma.wr_int), (0, 211))

    def test_dma_rejects_invalid_types_and_fields(self):
        for dma_type in ("Typo", "NonCoherent-extra", "Stream-extra", None, 1):
            with self.subTest(dma_type=dma_type):
                with self.assertRaisesRegex(
                    workload_config.WorkloadConfigError,
                    "Type must be NonCoherent or Stream",
                ):
                    self.dma(dma_type)
        for dma_type, field in (
            ("NonCoherent", "ReadInt"),
            ("NonCoherent", "WriteInt"),
            ("Stream", "InterruptNum"),
        ):
            with self.subTest(dma_type=dma_type, field=field):
                with self.assertRaisesRegex(
                    workload_config.WorkloadConfigError,
                    f"unsupported field {field}",
                ):
                    self.dma(dma_type, **{field: 1})
        for field in ("Name", "PIOMaster"):
            for value in (None, "", True, 1):
                with self.subTest(field=field, value=value):
                    with self.assertRaisesRegex(
                        workload_config.WorkloadConfigError,
                        f"{field} must be a nonempty string",
                    ):
                        self.dma("NonCoherent", **{field: value})

    def test_dma_rejects_invalid_numeric_fields(self):
        for dma_type in ("NonCoherent", "Stream"):
            for field, upper in (
                ("BufferSize", 1 << 64),
                ("MaxReqSize", 1 << 32),
            ):
                for value in (0, -1, True, 1.5, "64", None, upper):
                    with self.subTest(
                        dma_type=dma_type, field=field, value=value
                    ):
                        with self.assertRaises(
                            workload_config.WorkloadConfigError
                        ) as ctx:
                            self.dma(dma_type, **{field: value})
                        self.assertEqual(ctx.exception.field, field)
                        self.assertEqual(
                            ctx.exception.path,
                            os.path.join(self.root, "config.yml"),
                        )
        for dma_type, field in (
            ("NonCoherent", "InterruptNum"),
            ("Stream", "ReadInt"),
            ("Stream", "WriteInt"),
        ):
            for value in (-1, True, 1.5, "200", None, 1 << 32):
                with self.subTest(dma_type=dma_type, field=field, value=value):
                    with self.assertRaisesRegex(
                        workload_config.WorkloadConfigError,
                        f"{field} must be an integer",
                    ):
                        self.dma(dma_type, **{field: value})

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
