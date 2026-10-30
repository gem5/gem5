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


"""Require every selected SALAM suite and test to pass."""

import glob
import os
import sys
import xml.etree.ElementTree as ET


def main(listing_path, results_path, suite_dir=None):
    errors = []
    listed = {}
    with open(listing_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            fields = line.split(":", 2)
            if len(fields) != 3 or fields[0] != "SuiteUID":
                errors.append(f"unexpected listing line: {line}")
                continue
            _kind, path, name = fields
            if name in listed:
                errors.append(f"duplicate suite name: {name}")
            listed[name] = path
    if not listed:
        errors.append("no suites selected")
    if suite_dir is not None:
        for test_file in sorted(
            glob.glob(os.path.join(suite_dir, "test_*.py"))
        ):
            base = os.path.basename(test_file)
            if not any(os.path.basename(p) == base for p in listed.values()):
                errors.append(f"{base} contributed no suites")

    if not os.path.isfile(results_path):
        # TestLib fails to save results after a fixture setup error.
        print(
            f"{results_path} is missing; see the TestLib log above",
            file=sys.stderr,
        )
        return 1
    ran = {}
    for suite in ET.parse(results_path).getroot().iter("testsuite"):
        cases = list(suite.iter("testcase"))
        ran[suite.get("name")] = len(cases)
        for case in cases:
            if case.get("status") != "Passed":
                errors.append(f"{case.get('name')}: {case.get('status')}")
    for name in sorted(listed):
        if ran.get(name, 0) == 0:
            errors.append(f"suite {name} did not run")
    total = sum(ran.values())
    if total == 0:
        errors.append("no tests ran")

    for error in errors:
        print(error, file=sys.stderr)
    print(f"{len(listed)} suites listed, {total} tests run")
    return 1 if errors else 0


if __name__ == "__main__":
    if len(sys.argv) not in (3, 4):
        sys.exit("usage: check_salam_results.py LISTING XML [SUITE_DIR]")
    sys.exit(main(*sys.argv[1:]))
