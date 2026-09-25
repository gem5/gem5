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

"""Exercise JSON stats comparisons without building or running gem5."""

import json
from functools import partial
from io import StringIO

from testlib import (
    TestFunction,
    TestSuite,
    constants,
    verifier,
)


def check_stats(params, trusted, actual, diagnostic):
    check = verifier.MatchJSONStats("trusted.json", "output.json")
    try:
        check._compare_stats(
            StringIO(json.dumps(trusted)), StringIO(json.dumps(actual))
        )
    except AssertionError as error:
        if diagnostic is None:
            raise
        message = str(error)
        assert diagnostic in message, message
        assert "trusted.json" in message and "output.json" in message
    else:
        assert diagnostic is None, "Expected a statistics mismatch"


cases = [
    ("equal", {"count": 4}, {"count": 4}, None),
    ("extra-stat", {"count": 4}, {"count": 4, "other": 5}, None),
    ("changed-stat", {"count": 4}, {"count": 5}, "$.count:"),
    ("missing-stat", {"count": 4}, {}, "test_value: <missing>"),
    ("null-value", {"count": None}, {"count": None}, None),
    ("missing-null", {"count": None}, {}, "test_value: <missing>"),
    ("unexpected-null", {"count": 4}, {"count": None}, "test_value: None"),
    (
        "nested-metadata",
        {"count": {"value": 4}},
        {"count": {"value": 4, "unit": "Count"}, "creation_time": "now"},
        None,
    ),
    (
        "nested-mismatch",
        {"count": {"value": 4}},
        {"count": {"value": 5, "unit": "Count"}},
        "$.count.value:",
    ),
    (
        "nested-missing",
        {"count": {"value": 4}},
        {"count": {"unit": "Count"}},
        "$.count.value:",
    ),
    ("wrong-object-type", {"count": {}}, {"count": []}, "$.count:"),
    ("wrong-list-type", {"cores": []}, {"cores": {}}, "$.cores:"),
    ("empty-list", {"cores": []}, {"cores": []}, None),
    (
        "core-metadata",
        {"cores": [{"value": 4}, {"value": 5}]},
        {"cores": [{"value": 4, "name": "generator"}, {"value": 5}]},
        None,
    ),
    (
        "first-core-mismatch",
        {"cores": [{"value": 4}, {"value": 5}]},
        {"cores": [{"value": 6}, {"value": 5}]},
        "$.cores[0].value:",
    ),
    (
        "second-core-mismatch",
        {"cores": [{"value": 4}, {"value": 5}]},
        {"cores": [{"value": 4}, {"value": 6}]},
        "$.cores[1].value:",
    ),
    (
        "missing-core",
        {"cores": [{"value": 4}, {"value": 5}]},
        {"cores": [{"value": 5}]},
        "$.cores.length:",
    ),
    (
        "extra-core",
        {"cores": [{"value": 4}]},
        {"cores": [{"value": 4}, {"value": 5}]},
        "$.cores.length:",
    ),
    (
        "reordered-cores",
        {"cores": [{"value": 4}, {"value": 5}]},
        {"cores": [{"value": 5}, {"value": 4}]},
        "$.cores[0].value:",
    ),
]

for host in constants.supported_hosts:
    for variant in constants.supported_variants:
        TestSuite(
            name=f"json-stats-verifier-{host}-{variant}",
            tests=[
                TestFunction(
                    partial(
                        check_stats,
                        trusted=trusted,
                        actual=actual,
                        diagnostic=diagnostic,
                    ),
                    name=f"json-stats-{name}",
                )
                for name, trusted, actual, diagnostic in cases
            ],
            tags=(
                constants.all_compiled_tag,
                constants.quick_tag,
                variant,
                host,
            ),
        )
