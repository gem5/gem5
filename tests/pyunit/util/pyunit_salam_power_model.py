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

import copy
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
_PATH = _ROOT / "util/SALAM-tools/SALAM-Configurator/power_model.py"
_SPEC = importlib.util.spec_from_file_location("salam_characterization", _PATH)
model = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(model)


def source():
    return {
        "revision": "synthetic-test",
        "path": "independent arithmetic",
        "row": "unit",
        "sha256": "0" * 64,
        "library": "synthetic",
        "datatype": "32-bit integer",
        "implementation": "test only",
        "synthetic": True,
    }


def fixture():
    return {
        "version": 1,
        "records": [
            {
                "key": "fu:adder",
                "id": "test-adder",
                "source": source(),
                "signatures": ["i32,i32->i32"],
                "area": {
                    "value": 10,
                    "unit": "um2",
                    "status": "characterized",
                },
                "leakage": {
                    "value": 2,
                    "unit": "uW",
                    "status": "characterized",
                },
            },
            {
                "key": "op:adder:add:i32,i32->i32",
                "id": "test-adder",
                "source": source(),
                "event": {"value": 5, "unit": "pJ", "status": "characterized"},
            },
        ],
    }


class SalamCharacterizationTestSuite(unittest.TestCase):
    def reject(self, document):
        with self.assertRaises(model.CharacterizationError):
            model.normalize(document)

    def test_units(self):
        result = model.normalize(fixture())
        self.assertEqual(result["area_um2"], [10, 0])
        self.assertEqual(result["leakage_w"], [2e-6, 0])
        self.assertEqual(result["event_j"], [0, 5e-12])
        self.assertEqual(
            result["area_status"], ["characterized", "unavailable"]
        )
        self.assertEqual(json.loads(result["sources"][0])["synthetic"], True)

    def test_zero_and_missing(self):
        doc = fixture()
        doc["records"][0]["area"]["value"] = 0
        result = model.normalize(doc)
        self.assertEqual(result["area_um2"], [0, 0])
        self.assertNotEqual(result["area_status"][0], result["area_status"][1])

    def test_numeric_rejections(self):
        for value in (True, False, -1, float("nan"), float("inf"), "5", None):
            with self.subTest(value=value):
                doc = fixture()
                doc["records"][1]["event"]["value"] = value
                self.reject(doc)

    def test_unknown_fields(self):
        for where in ("document", "record", "source", "quantity"):
            with self.subTest(where=where):
                doc = fixture()
                obj = {
                    "document": doc,
                    "record": doc["records"][0],
                    "source": doc["records"][0]["source"],
                    "quantity": doc["records"][0]["area"],
                }[where]
                obj["typo"] = 1
                self.reject(doc)

    def test_duplicate_bindings(self):
        doc = fixture()
        doc["records"].append(copy.deepcopy(doc["records"][0]))
        self.reject(doc)
        with self.assertRaises(model.CharacterizationError):
            model.merge(
                [model.normalize(fixture()), model.normalize(fixture())]
            )

    def test_incompatible_width(self):
        doc = fixture()
        doc["records"][1]["key"] = "op:adder:add:i64,i64->i64"
        self.reject(doc)

    def test_incompatible_implementation(self):
        doc = fixture()
        doc["records"][1]["id"] = "different-cell"
        self.reject(doc)

    def test_no_operation_proxy_inference(self):
        doc = fixture()
        doc["records"][1]["event"]["status"] = "proxy"
        self.reject(doc)
        doc["records"][1]["source"]["proxy_for"] = "explicit synthetic test"
        self.assertEqual(model.normalize(doc)["event_status"][1], "proxy")

    def test_derived_requires_conversion(self):
        doc = fixture()
        doc["records"][1]["event"]["status"] = "derived"
        self.reject(doc)
        doc["records"][1]["source"][
            "conversion"
        ] = "independent known interval"
        self.assertEqual(model.normalize(doc)["event_status"][1], "derived")

    def test_unavailable_is_not_zero(self):
        doc = fixture()
        doc["records"][0]["area"] = {"status": "unavailable", "value": 0}
        self.reject(doc)
        doc["records"][0]["area"] = {"status": "unavailable"}
        self.assertEqual(model.normalize(doc)["area_status"][0], "unavailable")

    def test_physical_fus_require_width(self):
        doc = fixture()
        del doc["records"][0]["signatures"]
        self.reject(doc)

    def test_operation_does_not_duplicate_area(self):
        doc = fixture()
        doc["records"][1]["area"] = doc["records"][0]["area"]
        self.reject(doc)

    def test_operation_signature_semantics(self):
        for operation, signature in (
            ("add", "i32,i8->i32"),
            ("add", "f32,f32->f32"),
            ("fadd", "i32,i32->i32"),
            ("icmp", "i32,i32->i32"),
            ("sub", "f999,f999->f999"),
            ("add", "i7,i7->i7"),
            ("invented", "i32,i32->i32"),
        ):
            with self.subTest(operation=operation, signature=signature):
                doc = fixture()
                doc["records"][0]["signatures"] = [signature]
                doc["records"][1]["key"] = f"op:adder:{operation}:{signature}"
                self.reject(doc)

    def test_bitcast_preserves_pointer_category(self):
        for signature in ("p32->i32", "i32->p32", "p32->f32", "f32->p32"):
            with self.subTest(signature=signature):
                doc = fixture()
                doc["records"][0]["signatures"] = [signature]
                doc["records"][1]["key"] = f"op:adder:bitcast:{signature}"
                self.reject(doc)
        for operation, signature in (
            ("bitcast", "p32->p32"),
            ("bitcast", "i32->f32"),
            ("ptrtoint", "p32->i32"),
            ("inttoptr", "i32->p32"),
        ):
            with self.subTest(operation=operation, signature=signature):
                doc = fixture()
                doc["records"][0]["signatures"] = [signature]
                doc["records"][1]["key"] = f"op:adder:{operation}:{signature}"
                self.assertEqual(len(model.normalize(doc)["keys"]), 2)

    def test_unknown_timing_is_not_inferred_from_source_interval(self):
        doc = fixture()
        doc["records"][0]["source"]["reference_interval_s"] = 1e-8
        result = model.normalize(doc)
        self.assertEqual(result["timing_available"], [False, False])
        self.assertEqual(result["minimum_latency_s"], [0.0, 0.0])
        self.assertEqual(result["minimum_issue_interval_s"], [0.0, 0.0])
        self.assertEqual(result["pipelined"], [False, False])
        self.assertEqual(result["area_status"][0], "characterized")

    def test_explicit_physical_fu_constraints(self):
        doc = fixture()
        doc["records"][0]["constraints"] = {
            "minimum_latency_s": 3e-9,
            "minimum_issue_interval_s": 1e-9,
            "pipelined": True,
        }
        result = model.normalize(doc)
        self.assertEqual(result["timing_available"], [True, False])
        self.assertEqual(result["minimum_latency_s"], [3e-9, 0.0])
        self.assertEqual(result["minimum_issue_interval_s"], [1e-9, 0.0])
        self.assertEqual(result["pipelined"], [True, False])
        doc["records"][0]["constraints"] = {
            "minimum_latency_s": 0,
            "minimum_issue_interval_s": 0,
            "pipelined": False,
        }
        result = model.normalize(doc)
        self.assertEqual(result["timing_available"], [True, False])
        self.assertEqual(result["minimum_latency_s"], [0.0, 0.0])

    def test_incomplete_or_invalid_physical_constraints_rejected(self):
        doc = fixture()
        constraints = {
            "minimum_latency_s": 3e-9,
            "minimum_issue_interval_s": 1e-9,
            "pipelined": True,
        }
        doc["records"][0]["constraints"] = constraints
        for field in constraints:
            bad = copy.deepcopy(doc)
            del bad["records"][0]["constraints"][field]
            self.reject(bad)
        for field in ("minimum_latency_s", "minimum_issue_interval_s"):
            for invalid in (True, None, -1, float("nan"), float("inf"), "1ns"):
                with self.subTest(field=field, invalid=invalid):
                    bad = copy.deepcopy(doc)
                    bad["records"][0]["constraints"][field] = invalid
                    self.reject(bad)
        for invalid in (0, 1, None, "false"):
            bad = copy.deepcopy(doc)
            bad["records"][0]["constraints"]["pipelined"] = invalid
            self.reject(bad)
        bad = copy.deepcopy(doc)
        bad["records"][0]["constraints"]["stages"] = 3
        self.reject(bad)
        bad = copy.deepcopy(doc)
        bad["records"][1]["constraints"] = constraints
        self.reject(bad)
        for invalid in ({}, None, []):
            bad = copy.deepcopy(doc)
            bad["records"][0]["constraints"] = invalid
            self.reject(bad)

    def test_json_file_preserves_exponent_numbers(self):
        doc = fixture()
        doc["records"][0]["source"]["reference_interval_s"] = 1e-8
        doc["records"][0]["constraints"] = {
            "minimum_latency_s": 1e-9,
            "minimum_issue_interval_s": 1e-9,
            "pipelined": True,
        }
        doc["records"][1]["event"] = {
            "value": 5e-12,
            "unit": "J",
            "status": "characterized",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "records.json"
            path.write_text(json.dumps(doc))
            result = model.load(path)
        self.assertEqual(result["event_j"][1], 5e-12)
        self.assertEqual(result["minimum_latency_s"][0], 1e-9)
        self.assertEqual(
            json.loads(result["sources"][0])["reference_interval_s"],
            1e-8,
        )

    def test_json_file_rejects_duplicate_keys_and_invalid_syntax(self):
        encoded = json.dumps(fixture())
        inputs = (
            encoded.replace('"version": 1', '"version": 1, "version": 1'),
            encoded.replace(
                '"synthetic": true',
                '"synthetic": true, "synthetic": true',
                1,
            ),
            '{"version": 1,',
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "records.json"
            for text in inputs:
                with self.subTest(text=text):
                    path.write_text(text)
                    with self.assertRaises(model.CharacterizationError):
                        model.load(path)

    def test_json_file_retains_numeric_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "records.json"
            for value in (float("nan"), float("inf"), True, "1e-8"):
                with self.subTest(value=value):
                    doc = fixture()
                    doc["records"][1]["event"]["value"] = value
                    path.write_text(json.dumps(doc))
                    with self.assertRaises(model.CharacterizationError):
                        model.load(path)

    def test_source_identity_required(self):
        for field in (
            "revision",
            "path",
            "row",
            "sha256",
            "library",
            "datatype",
            "implementation",
        ):
            with self.subTest(field=field):
                doc = fixture()
                del doc["records"][0]["source"][field]
                self.reject(doc)

    def test_no_runtime_units_guessing(self):
        for unit in ("mW", "cycles", "picojoules", "uW2"):
            doc = fixture()
            doc["records"][1]["event"]["unit"] = unit
            self.reject(doc)

    def test_duplicate_yaml_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yml"
            path.write_text("version: 1\nversion: 1\nrecords: []\n")
            with self.assertRaises(model.CharacterizationError):
                model.load(path)

    def test_memory_conversions(self):
        doc = {
            "version": 1,
            "records": [
                {
                    "key": "memory",
                    "id": "test-memory",
                    "source": source(),
                    "area": {
                        "value": 0.25,
                        "unit": "mm2",
                        "status": "characterized",
                    },
                    "leakage": {
                        "value": 2,
                        "unit": "mW",
                        "status": "characterized",
                    },
                    "read": {
                        "value": 3,
                        "unit": "nJ",
                        "status": "characterized",
                    },
                    "write": {
                        "value": 4,
                        "unit": "nJ",
                        "status": "characterized",
                    },
                }
            ],
            "geometry": {
                "capacity_bytes": 64,
                "word_bytes": 4,
                "banks": 1,
                "read_ports": 0,
                "write_ports": 0,
                "rw_ports": 1,
                "mask_model": "read_modify_write",
                "minimum_access_interval_s": 2e-9,
                "minimum_access_latency_s": 3e-9,
                "storage_physical": "sram",
                "storage_scope": "whole_device",
            },
        }
        result = model.normalize(doc)
        self.assertEqual(result["area_um2"], [250000])
        self.assertEqual(result["leakage_w"], [0.002])
        self.assertAlmostEqual(result["read_j"][0], 3e-9, delta=1e-23)
        self.assertEqual(result["write_j"], [4e-9])
        self.assertEqual(result["minimum_access_interval_s"], 2e-9)
        self.assertEqual(result["minimum_access_latency_s"], 3e-9)
        for field in ("minimum_access_interval_s", "minimum_access_latency_s"):
            for invalid in (None, True, -1, float("nan"), float("inf")):
                with self.subTest(field=field, invalid=invalid):
                    bad = copy.deepcopy(doc)
                    bad["geometry"][field] = invalid
                    self.reject(bad)
            bad = copy.deepcopy(doc)
            del bad["geometry"][field]
            self.reject(bad)
        ff = copy.deepcopy(doc)
        ff["geometry"]["storage_physical"] = "flip_flop"
        self.reject(ff)
        ff["geometry"]["mask_model"] = "byte_enable"
        self.assertEqual(model.normalize(ff)["storage_physical"], "flip_flop")
        for field, value in (
            ("word_bytes", 0),
            ("capacity_bytes", 63),
            ("banks", True),
            ("mask_model", "guess"),
            ("storage_physical", "sram-or-ff"),
            ("storage_scope", "unknown"),
        ):
            bad = copy.deepcopy(doc)
            bad["geometry"][field] = value
            self.reject(bad)
        cells = copy.deepcopy(doc)
        cells["geometry"]["storage_scope"] = "storage_cells"
        self.assertEqual(
            model.normalize(cells)["storage_scope"], "storage_cells"
        )
        cells["geometry"]["minimum_access_interval_s"] = None
        cells["geometry"]["minimum_access_latency_s"] = None
        unknown = model.normalize(cells)
        self.assertFalse(unknown["memory_timing_available"])
        self.assertEqual(unknown["minimum_access_interval_s"], 0.0)
        self.assertIsNone(
            json.loads(unknown["sources"][0])["declared_memory_timing"][
                "minimum_access_interval_s"
            ]
        )
        cells["geometry"]["minimum_access_latency_s"] = 0
        self.reject(cells)
        del cells["geometry"]["storage_scope"]
        self.reject(cells)
        del doc["geometry"]
        self.reject(doc)


if __name__ == "__main__":
    unittest.main()
