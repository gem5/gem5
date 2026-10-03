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

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
_CANONICALIZER_PATH = _REPO_ROOT / "util" / "stats_canonicalizer.py"
_SPEC = importlib.util.spec_from_file_location(
    "stats_canonicalizer",
    _CANONICALIZER_PATH,
)
stats_canonicalizer = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(stats_canonicalizer)


class StatsCanonicalizerTestCase(unittest.TestCase):
    def _config(self):
        return {
            "type": "Root",
            "name": None,
            "path": "root",
            "system": {
                "type": "System",
                "name": "system",
                "path": "system",
                "cpu": [
                    {
                        "type": "TimingSimpleCPU",
                        "name": "cpu0",
                        "path": "system.cpu0",
                        "dcache": {
                            "type": "Cache",
                            "name": "dcache",
                            "path": "system.cpu0.dcache",
                        },
                    },
                    {
                        "type": "TimingSimpleCPU",
                        "name": "cpu1",
                        "path": "system.cpu1",
                    },
                ],
                "single": [
                    {
                        "type": "Cache",
                        "name": "single",
                        "path": "system.single",
                    }
                ],
            },
        }

    def _canonicalize(self, text, config=None, allow_collisions=False):
        path_map = {}
        if config is not None:
            path_map = stats_canonicalizer.build_path_map(config)
        output, mapping = stats_canonicalizer.canonicalize_lines(
            text.splitlines(keepends=True),
            path_map=path_map,
            allow_collisions=allow_collisions,
        )
        return "".join(output), mapping

    def test_multi_element_vector_paths(self):
        output, mapping = self._canonicalize(
            "system.cpu0.numCycles 10 # comment\n"
            "system.cpu1.numCycles 20\n",
            config=self._config(),
        )

        self.assertEqual(
            output,
            "system.cpu[0].numCycles 10 # comment\n"
            "system.cpu[1].numCycles 20\n",
        )
        self.assertEqual(
            mapping["system.cpu0.numCycles"],
            "system.cpu[0].numCycles",
        )

    def test_length_one_vector_path(self):
        output, mapping = self._canonicalize(
            "system.single.hits 7\n",
            config=self._config(),
        )

        self.assertEqual(output, "system.single[0].hits 7\n")
        self.assertEqual(
            mapping["system.single.hits"],
            "system.single[0].hits",
        )

    def test_substat_suffix_is_preserved(self):
        output, _ = self._canonicalize(
            "system.cpu0.dcache.overallMisses::total 3\n",
            config=self._config(),
        )

        self.assertEqual(
            output,
            "system.cpu[0].dcache.overallMisses::total 3\n",
        )

    def test_distribution_columns_are_preserved(self):
        output, _ = self._canonicalize(
            "system.cpu0.issuedInstType_0::IntAlu"
            " 7437 77.92% 77.92% # Number of insts issued\n",
            config=self._config(),
        )

        self.assertEqual(
            output,
            "system.cpu[0].issuedInstType_0::IntAlu"
            " 7437 77.92% 77.92% # Number of insts issued\n",
        )

    def test_unmapped_names_and_headers_are_preserved(self):
        text = (
            "---------- Begin Simulation Statistics ----------\n"
            "system.cpu0x.numCycles 11\n"
            "simTicks 100\n"
            "---------- End Simulation Statistics   ----------\n"
        )

        output, mapping = self._canonicalize(text, config=self._config())

        self.assertEqual(output, text)
        self.assertEqual(
            mapping["system.cpu0x.numCycles"], "system.cpu0x.numCycles"
        )
        self.assertEqual(mapping["simTicks"], "simTicks")

    def test_idempotent_for_already_canonical_names(self):
        text = "system.cpu[0].numCycles 10\n"

        once, _ = self._canonicalize(text, config=self._config())
        twice, _ = self._canonicalize(once, config=self._config())

        self.assertEqual(once, text)
        self.assertEqual(twice, text)

    def test_collision_detection(self):
        text = "system.cpu0.numCycles 10\n" "system.cpu[0].numCycles 20\n"

        with self.assertRaises(stats_canonicalizer.CanonicalizationError):
            self._canonicalize(text, config=self._config())

        output, _ = self._canonicalize(
            text,
            config=self._config(),
            allow_collisions=True,
        )
        self.assertEqual(
            output,
            "system.cpu[0].numCycles 10\n" "system.cpu[0].numCycles 20\n",
        )

    def test_without_config_does_not_guess_digit_suffixes(self):
        text = "system.cpu0.numCycles 10\n"

        output, mapping = self._canonicalize(text)

        self.assertEqual(output, text)
        self.assertEqual(
            mapping["system.cpu0.numCycles"],
            "system.cpu0.numCycles",
        )

    def test_value_tokens_require_a_boundary(self):
        text = "system.cpu0.units nanosecond\n" "system.cpu0.note infinity\n"

        output, mapping = self._canonicalize(text, config=self._config())

        self.assertEqual(output, text)
        self.assertEqual(mapping, {})

    def test_cli_writes_output_and_mapping(self):
        with tempfile.TemporaryDirectory() as tempdir:
            tempdir = Path(tempdir)
            input_path = tempdir / "stats.txt"
            output_path = tempdir / "canonical-stats.txt"
            config_path = tempdir / "config.json"
            mapping_path = tempdir / "mapping.json"

            input_path.write_text("system.cpu0.numCycles 10\n")
            config_path.write_text(json.dumps(self._config()))

            stats_canonicalizer.canonicalize_file(
                input_path,
                output_path,
                config_json_path=config_path,
                mapping_output_path=mapping_path,
            )

            self.assertEqual(
                output_path.read_text(),
                "system.cpu[0].numCycles 10\n",
            )
            self.assertEqual(
                json.loads(mapping_path.read_text()),
                {
                    "system.cpu0.numCycles": "system.cpu[0].numCycles",
                },
            )


if __name__ == "__main__":
    unittest.main()
