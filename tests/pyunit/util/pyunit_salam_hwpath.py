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

import os
import sys
import tempfile
import textwrap
import unittest

_CONFIGURATOR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    os.pardir,
    os.pardir,
    os.pardir,
    "util",
    "SALAM-tools",
    "SALAM-Configurator",
)
sys.path.insert(0, os.path.normpath(_CONFIGURATOR))

import config_parser
import workload_config


def _write(root, rel, text):
    path = os.path.join(root, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(textwrap.dedent(text))
    return path


def _cluster(accelerators):
    lines = ["acc_cluster:", "    - Name: clstr"]
    for name, extra in accelerators:
        lines += [
            "    - Accelerator:",
            f"          - Name: {name}",
            f"            IrPath: hw/{name}.ll",
            "            Profile: k",
            "            PIOSize: 5",
            "            PIOMaster: LocalBus",
        ]
        lines += [f"            {line}" for line in extra]
    return "\n".join(lines) + "\n"


def _timing(add):
    return f"hw_config:\n    k:\n        instructions:\n            add:\n                runtime_cycles: {add}\n"


def _counts(root):
    accs = {}
    for cluster in config_parser.load_clusters(root):
        for acc in cluster.accs:
            accs[acc.name] = acc
    return accs


class SalamHWPathTestSuite(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = os.path.realpath(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def resolve(self, acc):
        return config_parser.resolve_cycle_counts(
            acc.hw_config_document,
            acc.hw_config_path,
            acc.profile_name,
            acc.hw_config_source,
        )

    def test_missing_accelerator_hwpath_names_declaring_file(self):
        config = _write(
            self.root, "config.yml", _cluster([("a", ["HWPath: t/no.yml"])])
        )
        acc = _counts(self.root)["a"]
        with self.assertRaises(workload_config.WorkloadConfigError) as ctx:
            self.resolve(acc)
        err = ctx.exception
        self.assertEqual(err.path, config)
        self.assertEqual(err.value, os.path.join(self.root, "t", "no.yml"))
        self.assertEqual(err.field, "HWPath")
        self.assertIsInstance(err.__cause__, FileNotFoundError)

    def test_missing_inherited_hwpath_names_including_file(self):
        config = _write(
            self.root,
            "config.yml",
            "acc_cluster:\n"
            "    - SysPath: sub/c.yml\n"
            "      HWPath: hw/no.yml\n",
        )
        _write(self.root, "sub/c.yml", _cluster([("a", [])]))
        acc = _counts(self.root)["a"]
        with self.assertRaises(workload_config.WorkloadConfigError) as ctx:
            self.resolve(acc)
        err = ctx.exception
        self.assertEqual(err.path, config)
        self.assertEqual(err.value, os.path.join(self.root, "hw", "no.yml"))
        self.assertIsInstance(err.__cause__, FileNotFoundError)

    def test_selection_precedence_is_unchanged(self):
        _write(
            self.root,
            "config.yml",
            "acc_cluster:\n"
            "    - SysPath: sub/c.yml\n"
            "      HWPath: cluster.yml\n",
        )
        _write(self.root, "cluster.yml", _timing(3))
        _write(self.root, "acc.yml", _timing(5))
        _write(
            self.root,
            "sub/c.yml",
            _cluster([("a", ["HWPath: ../acc.yml"]), ("b", [])]) + _timing(9),
        )
        accs = _counts(self.root)
        self.assertEqual(self.resolve(accs["a"]), {"add": 5})
        self.assertEqual(self.resolve(accs["b"]), {"add": 3})

    def test_file_errors_omit_missing_profile(self):
        err = config_parser.CycleCountError("x.yml", None, "bad file")
        self.assertEqual(str(err), "SALAM config: x.yml: bad file")

    def test_nested_paths_keep_their_documented_bases(self):
        _write(
            self.root, "config.yml", "acc_cluster:\n    - SysPath: sub/c.yml\n"
        )
        _write(
            self.root,
            "sub/c.yml",
            "acc_cluster:\n    - SysPath: leaf/d.yml\n      HWPath: cycles.yml\n",
        )
        _write(self.root, "sub/cycles.yml", _timing(7))
        _write(
            self.root,
            "leaf/d.yml",
            _cluster([("a", [])]) + "          - Var:\n"
            "                - Name: Buffer\n"
            "                  Type: SPM\n"
            "                  Size: 64\n"
            "                  Characterization: coefficients/mem.yml\n",
        )
        acc = _counts(self.root)["a"]
        self.assertEqual(acc.working_dir, self.root)
        self.assertEqual(acc.ir_path, "hw/a.ll")
        self.assertEqual(self.resolve(acc), {"add": 7})
        self.assertEqual(
            acc.variables[0].characterization,
            os.path.join(self.root, "leaf/coefficients/mem.yml"),
        )
        self.assertEqual(
            acc.hw_config_path, os.path.join(self.root, "sub/cycles.yml")
        )

    def test_absolute_config_and_resource_paths(self):
        included = _write(self.root, "sub/c.yml", _cluster([("a", [])]))
        main = _write(
            self.root,
            "entry.yml",
            f"acc_cluster:\n    - SysPath: {included}\n",
        )
        (cluster,) = config_parser.load_clusters(self.root, main)
        self.assertEqual(cluster.accs[0].config_path, included)
        cycles = _write(self.root, "cycles.yml", _timing(9))
        _write(
            self.root, "config.yml", _cluster([("a", [f"HWPath: {cycles}"])])
        )
        self.assertEqual(self.resolve(_counts(self.root)["a"]), {"add": 9})

    def test_recursive_syspath_is_a_configuration_error(self):
        _write(
            self.root,
            "config.yml",
            "acc_cluster:\n    - SysPath: config.yml\n",
        )
        with self.assertRaisesRegex(
            workload_config.WorkloadConfigError, "recursive SysPath"
        ):
            _counts(self.root)
