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

    def edit(self, alias, **changes):
        # Set (or, for None, remove) parameters of one unit's YAML.
        path = os.path.join(
            self.profile, "functional_units", alias, alias + ".yml"
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
        text = g.generate_sources(model)["FunctionalUnits.py"]
        adder = text[text.index("class IntegerAdder(") :]
        adder = adder[: adder.index("\nclass ")]
        self.assertIn("initiation_interval = Param.UInt32(3,", adder)

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


if __name__ == "__main__":
    unittest.main()
