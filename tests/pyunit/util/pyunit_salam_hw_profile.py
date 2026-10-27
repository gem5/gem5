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


# Functional-unit fields in SALAM hardware profiles.

import ast
import dataclasses
import os
import shutil
import sys
import tempfile
import unittest

import yaml

_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), *[os.pardir] * 3)
)
sys.path.insert(0, os.path.join(_ROOT, "util", "SALAM-tools", "hw_generator"))

import SALAMGeneratedSources as g

_DEFAULT = os.path.join(_ROOT, "src", "salam", "hw_profiles", "default")


class FunctionalUnitFieldTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.profile = os.path.join(self.tmp, "profile")
        self.reset()

    def reset(self):
        # A fresh copy of the default profile.
        shutil.rmtree(self.profile, ignore_errors=True)
        shutil.copytree(_DEFAULT, self.profile)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def load(self):
        config = os.path.join(self.profile, "salam-hw-config.yml")
        return g.load_generated_sources(g.load_source_config(config))

    def edit(self, unit_alias, **changes):
        # Set (or, for None, remove) parameters of one unit's YAML.
        path = os.path.join(
            self.profile, "functional_units", unit_alias, unit_alias + ".yml"
        )
        with open(path) as f:
            document = yaml.safe_load(f)
        params = document["functional_unit"]["parameters"]
        for key, value in changes.items():
            if value is None:
                params.pop(key, None)
            else:
                params[key] = value
        with open(path, "w") as f:
            yaml.safe_dump(document, f, sort_keys=False)

    def unit(self, model, alias):
        return next(u for u in model.functional_units if u.alias == alias)

    def assertRejected(self, pattern, **changes):
        self.edit("integer_adder", **changes)
        with self.assertRaisesRegex(g.SourceConfigError, pattern):
            self.load()

    def bind_characterization(self, alias, filename):
        path = os.path.join(
            self.profile, "functional_units", alias, alias + ".yml"
        )
        with open(path) as handle:
            document = yaml.safe_load(handle)
        document["functional_unit"]["power_model"][
            "characterization"
        ] = filename
        with open(path, "w") as handle:
            yaml.safe_dump(document, handle, sort_keys=False)
        return path

    def synthetic_characterization(self, name):
        source = {
            "revision": "synthetic-test",
            "path": "profile characterization fixture",
            "row": name,
            "sha256": "0" * 64,
            "library": "synthetic",
            "datatype": "storage bit" if name == "register" else "i32",
            "implementation": "test only",
            "proxy_for": "synthetic static-coverage fixture",
            "synthetic": True,
        }
        identity = "test-" + name
        physical = {
            "key": "register:bit" if name == "register" else "fu:" + name,
            "id": identity,
            "source": source,
            "area": {"value": 10, "unit": "um2", "status": "proxy"},
            "leakage": {"value": 2, "unit": "uW", "status": "proxy"},
        }
        records = [physical]
        if name == "register":
            physical["read"] = {"status": "unavailable"}
            physical["write"] = {"status": "unavailable"}
        else:
            physical["signatures"] = ["i32,i32->i32"]
            for operation in ("add", "sub"):
                records.append(
                    {
                        "key": f"op:{name}:{operation}:i32,i32->i32",
                        "id": identity,
                        "source": source,
                        "event": {"status": "unavailable"},
                    }
                )
        target = os.path.join(self.profile, name + "-ppa.yml")
        with open(target, "w") as handle:
            yaml.safe_dump(
                {"version": 1, "records": records}, handle, sort_keys=False
            )
        return target

    def test_characterization_is_embedded_without_source_files(self):
        record = self.synthetic_characterization("integer_adder")
        self.bind_characterization(
            "integer_adder", "../../integer_adder-ppa.yml"
        )
        model = self.load()
        self.assertIn(record, model.fu_input_paths)
        expected = g.power_model.load(record)
        shutil.rmtree(self.profile)
        self.assertEqual(
            self.generated_data(model)["CHARACTERIZATION"], expected
        )

    def operation_characterization(
        self, alias, operation, signature, physical_signature=None
    ):
        source = {
            "revision": "synthetic-test",
            "path": "profile operation binding fixture",
            "row": "unit",
            "sha256": "0" * 64,
            "library": "synthetic",
            "datatype": signature,
            "implementation": "test only",
            "synthetic": True,
        }
        document = {
            "version": 1,
            "records": [
                {
                    "key": "fu:" + alias,
                    "id": "test-operation",
                    "source": source,
                    "signatures": [physical_signature or signature],
                    "area": {
                        "value": 10,
                        "unit": "um2",
                        "status": "characterized",
                    },
                },
                {
                    "key": f"op:{alias}:{operation}:{signature}",
                    "id": "test-operation",
                    "source": source,
                    "event": {
                        "value": 5,
                        "unit": "pJ",
                        "status": "characterized",
                    },
                },
            ],
        }
        path = os.path.join(self.profile, "operation-ppa.yml")
        with open(path, "w") as handle:
            yaml.safe_dump(document, handle, sort_keys=False)
        return path

    def test_characterization_operation_aliases_are_embedded(self):
        for alias, owned, operation, signature in (
            ("bitwise_operations", "and_inst", "and", "i32,i32->i32"),
            ("bitwise_operations", "or_inst", "or", "i32,i32->i32"),
            ("bitwise_operations", "xor_inst", "xor", "i32,i32->i32"),
            ("integer_adder", "gep", "getelementptr", "p32,i32->p32"),
            ("integer_adder", "add", "add", "i32,i32->i32"),
            ("integer_multiplier", "mul", "mul", "i32,i32->i32"),
        ):
            with self.subTest(operation=operation):
                self.reset()
                # Ownership keeps SALAM spelling, including the explicit
                # GEP assignment absent from the default profile.
                self.edit(alias, instructions=[owned])
                record = self.operation_characterization(
                    alias, operation, signature
                )
                self.bind_characterization(alias, "../../operation-ppa.yml")
                model = self.load()
                self.assertIn(record, model.fu_input_paths)
                self.assertEqual(
                    self.unit(model, alias).instruction_names, (owned,)
                )
                instruction = next(
                    item for item in model.instructions if item.name == owned
                )
                self.assertEqual(
                    instruction.functional_unit,
                    self.unit(model, alias).enum_value,
                )
                expected = g.power_model.load(record)
                self.assertIn(
                    f"op:{alias}:{operation}:{signature}", expected["keys"]
                )
                shutil.rmtree(self.profile)
                self.assertEqual(
                    self.generated_data(model)["CHARACTERIZATION"], expected
                )

    def test_characterization_binding_rejects_a_different_unit(self):
        self.synthetic_characterization("integer_adder")
        self.bind_characterization(
            "integer_multiplier", "../../integer_adder-ppa.yml"
        )
        with self.assertRaisesRegex(g.SourceConfigError, "FU alias mismatch"):
            self.load()

    def test_characterization_binding_rejects_unimplemented_operation(self):
        cases = [
            # Correct LLVM names still require ownership by this FU.
            (
                "integer_adder",
                "mul",
                "i32,i32->i32",
                None,
                "operation mismatch",
            ),
            (
                "integer_adder",
                "and",
                "i32,i32->i32",
                None,
                "operation mismatch",
            ),
            (
                "integer_adder",
                "getelementptr",
                "p32,i32->p32",
                None,
                "operation mismatch",
            ),
            # Do not generalize the four known aliases to arbitrary names.
            (
                "integer_adder",
                "add",
                "i32,i32->i32",
                ["add_inst"],
                "operation mismatch",
            ),
            (
                "integer_adder",
                "add_unknown",
                "i32,i32->i32",
                None,
                "unsupported FU operation binding",
            ),
            # LLVM operation/signature validation remains authoritative.
            (
                "bitwise_operations",
                "and",
                "f32,f32->f32",
                None,
                "incompatible operation/signature",
            ),
        ]
        # Coefficient keys must use LLVM spelling, not SALAM ownership names.
        for operation in ("and_inst", "or_inst", "xor_inst", "gep"):
            cases.append(
                (
                    "bitwise_operations",
                    operation,
                    "i32,i32->i32",
                    None,
                    "unsupported FU operation binding",
                )
            )
        for alias, operation, signature, owned, error in cases:
            with self.subTest(operation=operation, owned=owned):
                self.reset()
                if owned is not None:
                    self.edit(alias, instructions=owned)
                self.operation_characterization(alias, operation, signature)
                self.bind_characterization(alias, "../../operation-ppa.yml")
                with self.assertRaisesRegex(g.SourceConfigError, error):
                    self.load()

        self.reset()
        self.operation_characterization(
            "bitwise_operations", "and", "i64,i64->i64", "i32,i32->i32"
        )
        self.bind_characterization(
            "bitwise_operations", "../../operation-ppa.yml"
        )
        with self.assertRaisesRegex(
            g.SourceConfigError,
            "signature incompatible with physical implementation",
        ):
            self.load()

    def test_duplicate_register_binding_is_rejected_across_units(self):
        self.synthetic_characterization("register")
        for alias in ("integer_adder", "integer_multiplier"):
            self.bind_characterization(alias, "../../register-ppa.yml")
        with self.assertRaisesRegex(
            g.power_model.CharacterizationError, "duplicate"
        ):
            g.generate_sources(self.load())

    def test_missing_characterization_reference_is_not_ignored(self):
        self.bind_characterization("integer_adder", "missing.yml")
        with self.assertRaisesRegex(g.SourceConfigError, "missing.yml"):
            self.load()

    def test_default_profile_is_not_pipelined(self):
        model = self.load()
        for unit in model.functional_units:
            self.assertEqual(unit.initiation_interval, 0, unit.alias)

    def test_initiation_interval_reaches_the_generated_params(self):
        self.edit("integer_adder", initiation_interval=3)
        model = self.load()
        self.assertEqual(
            self.unit(model, "integer_adder").initiation_interval, 3
        )
        units = dict(self.generated_data(model)["FUNCTIONAL_UNITS"])
        self.assertEqual(units["integer_adder"]["initiation_interval"], 3)

    def test_initiation_interval_must_be_positive(self):
        for value in (0, -1, 2**32, "2", True, 1.5):
            with self.subTest(value=value):
                self.reset()
                self.assertRejected(
                    r"initiation_interval must be an integer from 1 ",
                    initiation_interval=value,
                )

    def test_enum_value_must_be_positive(self):
        self.assertRejected(
            r"enum_value must be an integer from 1 ", enum_value=0
        )

    def test_counts_must_be_unsigned_integers(self):
        for field, value in (
            ("limit", -1),
            ("limit", 1.5),
            ("stages", "1"),
            ("cycles", None),
        ):
            with self.subTest(field=field, value=value):
                self.reset()
                pattern = (
                    "missing cycles"
                    if value is None
                    else field + " must be an integer from 0 "
                )
                self.assertRejected(pattern, **{field: value})

    def test_unknown_parameter_is_rejected(self):
        self.assertRejected(
            r"unknown functional unit parameter initiation_interva ",
            initiation_interva=2,
        )

    def test_profile_replacement_preserves_inactive_files(self):
        root = os.path.join(self.tmp, "generated", "salam")
        sentinels = {}
        for directory in ("functional_units", "instructions"):
            for suffix in ("cc", "hh"):
                rel = f"HWModeling/{directory}/keep_me.{suffix}"
                path = os.path.join(root, rel)
                os.makedirs(os.path.dirname(path), exist_ok=True)
                sentinels[rel] = (
                    "#error This unrelated file must stay inactive\n"
                )
                with open(path, "w") as f:
                    f.write(sentinels[rel])

        for name in ("FunctionalUnits.py", "InstConfig.py"):
            sentinels[name] = (
                "raise RuntimeError('inactive generated module')\n"
            )
            with open(os.path.join(root, name), "w") as stream:
                stream.write(sentinels[name])

        def generate():
            outputs = g.generate_sources(self.load())
            targets = [os.path.join(root, name) for name in outputs]
            g.generate_sources_action(
                targets,
                [],
                {
                    "SALAM_HW_CONFIG": os.path.join(
                        self.profile, "salam-hw-config.yml"
                    )
                },
            )
            for rel, expected in {**outputs, **sentinels}.items():
                with open(os.path.join(root, rel)) as f:
                    self.assertEqual(f.read(), expected, rel)
            return outputs

        default = generate()
        directory = os.path.join(
            self.profile, "functional_units", "extra_unit"
        )
        os.makedirs(directory)
        shutil.copyfile(
            os.path.join(
                self.profile,
                "functional_units",
                "bit_register",
                "bit_register.yml",
            ),
            os.path.join(directory, "extra_unit.yml"),
        )
        self.edit(
            "extra_unit",
            alias="extra_unit",
            enum_value=99,
            instructions=["none"],
        )
        config = os.path.join(self.profile, "salam-hw-config.yml")
        with open(config) as f:
            document = yaml.safe_load(f)
        document["functional_unit_order"].append("extra_unit")
        with open(config, "w") as f:
            yaml.safe_dump(document, f, sort_keys=False)
        custom = generate()
        self.assertEqual(set(custom), set(default))
        self.assertNotEqual(custom, default)
        model = self.load()
        data = self.generated_data(model)
        self.assert_profile_records(model, data)
        self.assertEqual(data["FUNCTIONAL_UNITS"][-1][0], "extra_unit")
        self.reset()
        self.assertEqual(generate(), default)

    def generated_data(self, model):
        outputs = g.generate_sources(model)
        self.assertEqual(set(outputs), {"_salam_profile.py"})
        tree = ast.parse(outputs["_salam_profile.py"])
        values = {}
        for node in tree.body:
            self.assertIsInstance(node, ast.Assign)
            self.assertEqual(len(node.targets), 1)
            self.assertIsInstance(node.targets[0], ast.Name)
            name = node.targets[0].id
            self.assertNotIn(name, values)
            values[name] = ast.literal_eval(node.value)
        self.assertEqual(
            set(values),
            {
                "PROFILE_PATH",
                "FUNCTIONAL_UNITS",
                "INSTRUCTIONS",
                "CHARACTERIZATION",
            },
        )
        return values

    def assert_profile_records(self, model, data):
        metadata = {
            "path",
            "instruction_names",
            "characterization",
            "characterization_path",
        }
        units = tuple(
            (
                unit.alias,
                {
                    name: value
                    for name, value in dataclasses.asdict(unit).items()
                    if name not in metadata
                },
            )
            for unit in model.functional_units
        )
        instructions = tuple(
            (
                inst.name,
                {
                    name: value
                    for name, value in dataclasses.asdict(inst).items()
                    if name != "name"
                },
            )
            for inst in model.instructions
        )
        self.assertEqual(data["PROFILE_PATH"], model.spec.config_path)
        self.assertEqual(data["FUNCTIONAL_UNITS"], units)
        self.assertEqual(data["INSTRUCTIONS"], instructions)
        self.assertEqual(
            data["CHARACTERIZATION"],
            g.power_model.merge(
                unit.characterization for unit in model.functional_units
            ),
        )

    def test_default_profile_is_literal_data(self):
        model = self.load()
        self.assert_profile_records(model, self.generated_data(model))

    def test_singleton_profile_records(self):
        config_path = os.path.join(self.profile, "salam-hw-config.yml")
        with open(config_path) as stream:
            selected = yaml.safe_load(stream)
        units = os.path.join(self.profile, selected["functional_units"])
        for alias in selected["functional_unit_order"]:
            if alias != "integer_adder":
                shutil.rmtree(os.path.join(units, alias))
        selected["functional_unit_order"] = ["integer_adder"]
        with open(config_path, "w") as stream:
            yaml.safe_dump(selected, stream, sort_keys=False)
        self.edit("integer_adder", instructions=["add"])
        instruction_path = os.path.join(self.profile, selected["instructions"])
        with open(instruction_path) as stream:
            document = yaml.safe_load(stream)
        document["instructions"] = {"add": document["instructions"]["add"]}
        with open(instruction_path, "w") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        model = self.load()
        data = self.generated_data(model)
        self.assert_profile_records(model, data)
        self.assertEqual(
            [row[0] for row in data["FUNCTIONAL_UNITS"]], ["integer_adder"]
        )
        self.assertEqual([row[0] for row in data["INSTRUCTIONS"]], ["add"])

    def test_reordered_profile_records(self):
        original = self.load()
        config_path = os.path.join(self.profile, "salam-hw-config.yml")
        with open(config_path) as stream:
            selected = yaml.safe_load(stream)
        selected["functional_unit_order"].reverse()
        with open(config_path, "w") as stream:
            yaml.safe_dump(selected, stream, sort_keys=False)
        instruction_path = os.path.join(self.profile, selected["instructions"])
        with open(instruction_path) as stream:
            document = yaml.safe_load(stream)
        document["instructions"] = dict(
            reversed(list(document["instructions"].items()))
        )
        with open(instruction_path, "w") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        model = self.load()
        data = self.generated_data(model)
        self.assert_profile_records(model, data)
        self.assertEqual(
            [row[0] for row in data["FUNCTIONAL_UNITS"]],
            list(reversed(original.fu_aliases)),
        )
        self.assertEqual(
            [row[0] for row in data["INSTRUCTIONS"]],
            list(reversed(original.instruction_names)),
        )

    def test_invalid_profile_names_are_rejected(self):
        for category in ("unit", "instruction"):
            for alias in ("class", "_private", "bad-name", "trailing\n"):
                with self.subTest(category=category, alias=alias):
                    self.reset()
                    config_path = os.path.join(
                        self.profile, "salam-hw-config.yml"
                    )
                    with open(config_path) as stream:
                        selected = yaml.safe_load(stream)
                    if category == "unit":
                        selected["functional_unit_order"][0] = alias
                        with open(config_path, "w") as stream:
                            yaml.safe_dump(selected, stream, sort_keys=False)
                    else:
                        path = os.path.join(
                            self.profile, selected["instructions"]
                        )
                        with open(path) as stream:
                            document = yaml.safe_load(stream)
                        document["instructions"][alias] = dict(
                            document["instructions"]["add"]
                        )
                        with open(path, "w") as stream:
                            yaml.safe_dump(document, stream, sort_keys=False)
                    with self.assertRaisesRegex(
                        g.SourceConfigError, "invalid name"
                    ) as caught:
                        self.load()
                    self.assertIn(alias, str(caught.exception))


if __name__ == "__main__":
    unittest.main()
