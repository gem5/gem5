# Copyright (c) 2026 The Regents of the University of California
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

import configparser
import runpy
import tempfile
import unittest
import warnings
from pathlib import Path


class ArmSme2CheckpointTestSuite(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        util_dir = Path(__file__).resolve().parents[3] / "util"
        cls.checkpoint_tools = runpy.run_path(
            str(util_dir / "cpt_upgrader.py")
        )
        cls.upgrader_class = cls.checkpoint_tools["Upgrader"]
        # The existing loader uses bare open(). Keep its ResourceWarnings
        # separate from the migration behavior under test.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            cls.upgrader_class.load_all()

    def _checkpoint(self, cpu="board.processor.cores0.core"):
        checkpoint = configparser.ConfigParser()
        checkpoint.optionxform = str
        checkpoint["root"] = {"isa": "arm"}
        checkpoint["root.globals"] = {
            "version_tags": " ".join(
                sorted(self.upgrader_class.tag_set - {"arm-sme2"})
            )
        }
        checkpoint[cpu + ".isa"] = {"isaName": "arm"}
        checkpoint[cpu + ".xc.0"] = {
            # v25 has 44 256-byte vectors and 44 * 64 8-byte elements.
            "regs.vector": " ".join(["7"] * 11264),
            "regs.vector_element": " ".join(["11"] * 22528),
        }
        return checkpoint

    def _upgrade(self, checkpoint):
        tags = set(checkpoint["root.globals"]["version_tags"].split())
        self.upgrader_class.get("arm-sme2").update(checkpoint, tags)

    def _check_arrays(self, checkpoint, section):
        for field, old_size, new_size, value in (
            ("regs.vector", 11264, 11520, "7"),
            ("regs.vector_element", 22528, 23040, "11"),
        ):
            with self.subTest(field=field):
                actual = checkpoint[section][field].split()
                self.assertEqual(len(actual), new_size)
                self.assertEqual(actual[:old_size], [value] * old_size)
                self.assertEqual(
                    actual[old_size:], ["0"] * (new_size - old_size)
                )

    def test_stdlib_arrays(self):
        checkpoint = self._checkpoint()
        self._upgrade(checkpoint)
        self._check_arrays(checkpoint, "board.processor.cores0.core.xc.0")

    def test_legacy_and_custom_cpu_names(self):
        for cpu in ("system.cpu", "system.cpu0", "machine.cluster.worker"):
            with self.subTest(cpu=cpu):
                checkpoint = self._checkpoint(cpu)
                self._upgrade(checkpoint)
                self._check_arrays(checkpoint, cpu + ".xc.0")

    def test_multiple_threads(self):
        checkpoint = self._checkpoint("system.cpu")
        isa = checkpoint["system.cpu.isa"]
        checkpoint["system.cpu.isa0"] = dict(isa)
        checkpoint["system.cpu.isa1"] = dict(isa)
        checkpoint.remove_section("system.cpu.isa")
        checkpoint["system.cpu.xc.1"] = dict(checkpoint["system.cpu.xc.0"])
        self._upgrade(checkpoint)
        for thread in (0, 1):
            self._check_arrays(checkpoint, f"system.cpu.xc.{thread}")

    def test_non_arm_and_unrelated_sections_unchanged(self):
        checkpoint = self._checkpoint()
        checkpoint["board.processor.cores0.core.isa"]["isaName"] = "riscv"
        checkpoint["other.xc.0"] = {"unrelated": "keep"}
        checkpoint["board.processor.cores0.core.xc.0.extra"] = {
            "unrelated": "keep"
        }
        before = {section: dict(checkpoint[section]) for section in checkpoint}
        self._upgrade(checkpoint)
        self.assertEqual(
            {section: dict(checkpoint[section]) for section in checkpoint},
            before,
        )

    def test_preserves_existing_lut_state_and_is_idempotent(self):
        checkpoint = self._checkpoint()
        section = "board.processor.cores0.core.xc.0"
        # The original upgrader may already have extended the vector array.
        checkpoint[section]["regs.vector"] += " 19" * 256
        self._upgrade(checkpoint)
        self.assertEqual(
            checkpoint[section]["regs.vector"].split()[-256:], ["19"] * 256
        )
        before = dict(checkpoint[section])
        self._upgrade(checkpoint)
        self.assertEqual(dict(checkpoint[section]), before)

    def test_process_checkpoint_file(self):
        checkpoint = self._checkpoint("system.cpu0")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "m5.cpt"
            with path.open("w") as stream:
                checkpoint.write(stream)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", ResourceWarning)
                self.checkpoint_tools["process_file"](str(path), backup=False)
            upgraded = configparser.ConfigParser()
            upgraded.optionxform = str
            upgraded.read(path)
            self._check_arrays(upgraded, "system.cpu0.xc.0")
            self.assertIn(
                "arm-sme2", upgraded["root.globals"]["version_tags"].split()
            )


if __name__ == "__main__":
    unittest.main()
