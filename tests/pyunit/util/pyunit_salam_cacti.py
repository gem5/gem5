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

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
_TOOLS = _ROOT / "util" / "SALAM-tools"
sys.path.insert(0, str(_TOOLS / "cacti-SALAM"))
import import_cacti as importer
from power_model import (
    CharacterizationError,
    normalize,
)


class CactiImportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.csv = self.root / "out.csv"
        self.config = self.root / "input.cfg"
        self.binary = self.root / "synthetic-test-binary"
        self.binary.write_bytes(b"synthetic importer fixture, never executed")
        self.source = _ROOT / "ext" / "mcpat" / "cacti"
        self.expected = {
            "capacity_bytes": 4096,
            "word_bytes": 8,
            "banks": 2,
            "rw_ports": 1,
            "read_ports": 0,
            "write_ports": 0,
            "mask_model": "read_modify_write",
        }
        self.config.write_text(
            "\n".join(
                [
                    "-size (bytes) 4096",
                    "-block size (bytes) 8",
                    "-UCA bank count 2",
                    "-read-write port 1",
                    "-exclusive read port 0",
                    "-exclusive write port 0",
                    "-single ended read ports 0",
                    "-output/input bus width 64",
                    "-operating temperature (K) 300",
                    "-technology (u) 0.045",
                    "-associativity 1",
                    '-cache type "ram"',
                    '-Add ECC "false"',
                    '-Cache model (NUCA, UCA) "UCA"',
                    '-Data array cell type - "itrs-hp"',
                    '-Data array peripheral type - "itrs-hp"',
                ]
            )
            + "\n"
        )
        self.row = [
            "45",
            "4096",
            "2",
            "1",
            "64",
            "1.5",
            "2",
            "N/A",
            "2.5",
            "3.75",
            "0.25",
            "0.004",
            "1",
            "2",
            "1",
            "1",
            "1",
            "1",
            "50",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
            "N/A",
        ]
        self.write_csv()

    def write_csv(self, header=None):
        with self.csv.open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow([*(header or importer.HEADERS), ""])
            writer.writerow([*self.row, ""])

    def load(self):
        return importer.import_record(
            self.csv,
            self.config,
            self.source,
            self.expected,
            "synthetic-sram",
            [str(self.binary), "-infile", str(self.config)],
            self.binary,
            "gem5-cacti-6.5",
        )

    def test_units_and_bank_scope(self):
        result = normalize(self.load())
        self.assertEqual(result["area_um2"], [4000.0])
        self.assertAlmostEqual(result["leakage_w"][0], 0.0005, places=15)
        self.assertAlmostEqual(result["read_j"][0], 2.5e-9, places=20)
        self.assertAlmostEqual(result["write_j"][0], 3.75e-9, places=20)
        self.assertAlmostEqual(
            result["minimum_access_interval_s"], 2e-9, places=20
        )
        self.assertAlmostEqual(
            result["minimum_access_latency_s"], 1.5e-9, places=20
        )
        self.assertEqual(result["read_status"], ["derived"])
        self.assertEqual(result["storage_scope"], "whole_device")
        self.assertTrue(result["memory_timing_available"])

    def test_access_latency_and_cycle_interval_remain_distinct(self):
        self.row[5] = "4.5"
        self.row[6] = "2.25"
        self.write_csv()
        document = self.load()
        result = normalize(document)
        self.assertAlmostEqual(
            result["minimum_access_latency_s"], 4.5e-9, places=20
        )
        self.assertAlmostEqual(
            result["minimum_access_interval_s"], 2.25e-9, places=20
        )
        notes = json.loads(document["records"][0]["source"]["notes"])
        self.assertEqual(
            notes["access_time_s"], result["minimum_access_latency_s"]
        )
        for column in (5, 6):
            original = self.row[column]
            for invalid in ("0", "-1", "nan", "inf"):
                with self.subTest(column=column, invalid=invalid):
                    self.row[column] = invalid
                    self.write_csv()
                    with self.assertRaises(CharacterizationError):
                        self.load()
            self.row[column] = original

    def test_raw_provenance_and_geometry_retained(self):
        record = self.load()["records"][0]
        source = record["source"]
        notes = json.loads(source["notes"])
        self.assertEqual(source["sha256"], importer.digest(self.csv))
        self.assertEqual(notes["input_sha256"], importer.digest(self.config))
        self.assertEqual(notes["binary_sha256"], importer.digest(self.binary))
        self.assertEqual(notes["requested_geometry"], self.expected)
        self.assertEqual(notes["effective_geometry"]["capacity_bytes"], 4096)
        self.assertIsNone(source["voltage_v"])

    def test_no_implicit_capacity_resize(self):
        self.expected["capacity_bytes"] = 1024
        with self.assertRaisesRegex(
            CharacterizationError, "requested geometry"
        ):
            self.load()

    def test_effective_geometry_changes_rejected(self):
        for column, changed in [(1, "8192"), (2, "1"), (4, "32")]:
            with self.subTest(column=column):
                original = self.row[column]
                self.row[column] = changed
                self.write_csv()
                with self.assertRaisesRegex(
                    CharacterizationError, "effective geometry"
                ):
                    self.load()
                self.row[column] = original

    def test_capacity_echo_does_not_hide_subarray_padding(self):
        # 4088 requested bytes fit the CSV field but two banks with two
        # subarrays each round to 128 rows * 64 columns: 4096 physical bytes.
        self.expected["capacity_bytes"] = 4088
        self.config.write_text(self.config.read_text().replace("4096", "4088"))
        self.row[1] = "4088"
        self.write_csv()
        with self.assertRaisesRegex(
            CharacterizationError, "physical capacity changed"
        ):
            self.load()

    def test_effective_subarray_capacity_is_retained(self):
        notes = json.loads(self.load()["records"][0]["source"]["notes"])
        organization = notes["effective_subarray_organization"]
        self.assertEqual(organization["rows_per_subarray"], 128)
        self.assertEqual(organization["columns_per_subarray"], 64)
        self.assertEqual(organization["subarrays_per_bank"], 2)
        self.assertEqual(organization["capacity_bits"], 32768)
        self.row[14] = "0.5"
        self.write_csv()
        notes = json.loads(self.load()["records"][0]["source"]["notes"])
        self.assertEqual(
            notes["effective_subarray_organization"]["capacity_bits"], 32768
        )
        for invalid in ("0", "nan", "-1", "0.3"):
            self.row[14] = invalid
            self.write_csv()
            with self.assertRaises(CharacterizationError):
                self.load()

    def test_ports_match_existing_memory(self):
        for field in ("rw_ports", "read_ports", "write_ports"):
            with self.subTest(field=field):
                self.expected[field] += 1
                with self.assertRaisesRegex(
                    CharacterizationError, "requested geometry"
                ):
                    self.load()
                self.expected[field] -= 1

    def test_byte_enables_not_inferred(self):
        self.expected["mask_model"] = "byte_enable"
        with self.assertRaisesRegex(CharacterizationError, "byte enables"):
            self.load()
        self.expected["mask_model"] = "unsupported"
        self.assertEqual(self.load()["geometry"]["mask_model"], "unsupported")

    def test_nonfinite_and_negative_coefficients(self):
        for column in (8, 9, 10, 11):
            original = self.row[column]
            for value in ("nan", "inf", "-1"):
                with self.subTest(column=column, value=value):
                    self.row[column] = value
                    self.write_csv()
                    with self.assertRaises(CharacterizationError):
                        self.load()
            self.row[column] = original

    def test_zero_coefficient_is_available(self):
        for column in (8, 9, 10, 11):
            self.row[column] = "0"
        self.write_csv()
        result = normalize(self.load())
        self.assertEqual(result["read_j"], [0.0])
        self.assertEqual(result["read_status"], ["derived"])

    def test_energy_header_cannot_be_reinterpreted_as_power(self):
        header = list(importer.HEADERS)
        header[8] = "Dynamic read power (mW)"
        self.write_csv(header)
        with self.assertRaisesRegex(CharacterizationError, "CSV columns"):
            self.load()

    def test_multiple_rows_require_separate_records(self):
        with self.csv.open("a") as stream:
            csv.writer(stream).writerow(self.row)
        with self.assertRaisesRegex(CharacterizationError, "one result"):
            self.load()

    def test_source_changes_require_new_semantic_audit(self):
        directory = self.root / "changed-source"
        directory.mkdir()
        for filename in importer.FORMATS["gem5-cacti-6.5"]["hashes"]:
            (directory / filename).write_bytes(
                (self.source / filename).read_bytes()
            )
        with (directory / "io.cc").open("a") as stream:
            stream.write("// changed output semantics\n")
        self.source = directory
        with self.assertRaisesRegex(CharacterizationError, "source identity"):
            self.load()

    def test_configuration_duplicates_and_unsupported_models(self):
        original = self.config.read_text()
        for changed in (
            original + "-size (bytes) 8192\n",
            original.replace('-cache type "ram"', '-cache type "cache"'),
            original.replace('"UCA"', '"NUCA"'),
            original.replace(
                "-block size (bytes) 8", "-block size (bytes) 64"
            ),
            original + '-Array Power Gating - "true"\n',
            original.replace('-Add ECC "false"', '-Add ECC - "true"'),
        ):
            with self.subTest(config=changed):
                self.config.write_text(changed)
                with self.assertRaises(CharacterizationError):
                    self.load()

    def test_unknown_geometry_and_invalid_boolean(self):
        self.expected["unknown"] = 1
        with self.assertRaises(CharacterizationError):
            self.load()
        del self.expected["unknown"]
        self.expected["rw_ports"] = True
        with self.assertRaises(CharacterizationError):
            self.load()


if __name__ == "__main__":
    unittest.main()
