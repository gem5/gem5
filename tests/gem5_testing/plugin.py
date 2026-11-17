# Copyright (c) 2026 The Regents of The University of California
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

import json
import subprocess
import tempfile
from pathlib import (
    Path,
    PurePosixPath,
)

import pytest

LENGTHS = ("quick", "long", "very-long")


def pytest_addoption(parser):
    group = parser.getgroup("gem5")
    group.addoption(
        "--length",
        choices=(*LENGTHS, "all"),
        help="Select a gem5 test length (default: quick).",
    )
    group.addoption(
        "--tag",
        action="append",
        default=[],
        help="Require a gem5 tag. Repeat to require all specified tags.",
    )
    group.addoption(
        "--gem5-plan",
        action="store_true",
        help="Show the selected gem5 plan with --collect-only.",
    )
    group.addoption(
        "--gem5-plan-file",
        metavar="FILE",
        help="Show the selected gem5 plan and write it to a JSON file "
        "with --collect-only.",
    )
    parser.addini("gem5_tags", "Registered gem5 selection tags", type="args")


def pytest_configure(config):
    if (
        config.option.gem5_plan or config.option.gem5_plan_file is not None
    ) and not config.option.collectonly:
        raise pytest.UsageError(
            "--gem5-plan and --gem5-plan-file require --collect-only"
        )

    registered = set(config.getini("gem5_tags"))
    unknown = set(config.option.tag) - registered
    if unknown:
        raise pytest.UsageError(
            f"Unknown gem5 tag(s): {', '.join(sorted(unknown))}. "
            f"Registered tags: {', '.join(sorted(registered))}."
        )


def pytest_sessionstart(session):
    config = session.config
    suite_root = config.rootpath.resolve() / "tests" / "gem5"
    for arg in config.args:
        path = (config.invocation_params.dir / arg.split("::", 1)[0]).resolve()
        if path.is_file() and (
            not path.is_relative_to(suite_root)
            or not path.name.endswith("_pytest.py")
        ):
            raise pytest.UsageError(
                f"{arg} is outside the gem5 pytest suite. Select a "
                "*_pytest.py file under tests/gem5 or use TestLib."
            )


def pytest_ignore_collect(collection_path, config):
    # Keep a broad path selection (such as tests/) in the opted-in suite.
    suite_root = config.rootpath.resolve() / "tests" / "gem5"
    path = collection_path.resolve()
    if not path.is_relative_to(suite_root) and not suite_root.is_relative_to(
        path
    ):
        return True
    return None


def _metadata(item):
    marker = item.get_closest_marker("gem5")
    required = {"length", "tags", "build", "environment", "resources"}
    if marker is None or marker.args or marker.kwargs.keys() != required:
        raise pytest.UsageError(
            f"{item.nodeid} requires @pytest.mark.gem5 with "
            f"{', '.join(sorted(required))}."
        )

    metadata = marker.kwargs
    if metadata["length"] not in LENGTHS:
        raise pytest.UsageError(f"{item.nodeid} has an invalid gem5 length.")

    tags = metadata["tags"]
    if (
        not isinstance(tags, (list, tuple))
        or not all(isinstance(tag, str) for tag in tags)
        or not set(tags) <= set(item.config.getini("gem5_tags"))
    ):
        raise pytest.UsageError(
            f"{item.nodeid} must use tags registered in pytest-gem5.ini."
        )

    build = metadata["build"]
    if not isinstance(build, str):
        raise pytest.UsageError(f"{item.nodeid} needs a build target path.")
    path = PurePosixPath(build)
    if len(path.parts) < 2 or path.parts[0] != "build" or ".." in path.parts:
        raise pytest.UsageError(
            f"{item.nodeid} must use a target under build/."
        )

    resources = metadata["resources"]
    if not isinstance(resources, dict) or not all(
        isinstance(name, str)
        and isinstance(resource, (tuple, list))
        and len(resource) == 2
        and all(isinstance(value, str) and value for value in resource)
        for name, resource in resources.items()
    ):
        raise pytest.UsageError(
            f"{item.nodeid} resources must map names to (ID, version) pairs."
        )

    if metadata["environment"] != "host":
        raise pytest.UsageError(
            f"{item.nodeid} must use the host environment in this pilot."
        )
    return metadata


def _case_id(item):
    path = item.path.resolve().relative_to(item.config.rootpath.resolve())
    return f"{path.as_posix()}::{item.nodeid.partition('::')[2]}"


def _exact_cases(config):
    for arg in config.args:
        path, separator, name = arg.partition("::")
        if separator:
            absolute = (config.invocation_params.dir / path).resolve()
            yield absolute, name


@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(config, items):
    length = config.option.length
    tags = set(config.option.tag)
    exact = set(_exact_cases(config))
    selected, deselected = [], []
    seen = set()
    for item in items:
        case_id = _case_id(item)
        if case_id in seen:
            raise pytest.UsageError(f"Duplicate gem5 case ID: {case_id}")
        seen.add(case_id)
        case = _metadata(item)
        name = item.nodeid.partition("::")[2]
        explicit = (item.path.resolve(), name) in exact
        target_length = case["length"] if explicit else "quick"
        if (length or target_length) in (
            "all",
            case["length"],
        ) and tags <= set(case["tags"]):
            selected.append(item)
        else:
            deselected.append(item)
    items[:] = selected
    if deselected:
        config.hook.pytest_deselected(items=deselected)


def pytest_sessionfinish(session, exitstatus):
    config = session.config
    target = config.option.gem5_plan_file
    if exitstatus != pytest.ExitCode.OK or (
        not config.option.gem5_plan and target is None
    ):
        return

    cases = [
        {"id": _case_id(item), **_metadata(item)}
        for item in sorted(session.items, key=_case_id)
    ]
    reporter = config.pluginmanager.get_plugin("terminalreporter")
    if reporter:
        reporter.write_line("gem5 selection plan")
        for case in cases:
            build = case["build"]
            missing = (
                "" if (config.rootpath / build).is_file() else ", missing"
            )
            reporter.write_line(
                f"  {case['id']} ({case['length']}, "
                f"{case['environment']}, {build}{missing})"
            )

    if target is None:
        return

    plan = {
        "kind": "gem5-selection",
        "schema_version": 1,
        "builds": sorted({case["build"] for case in cases}),
        "executions": cases,
    }
    path = config.invocation_params.dir / target
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as plan_file:
            temporary = Path(plan_file.name)
            json.dump(plan, plan_file, indent=2, sort_keys=True)
            plan_file.write("\n")
        temporary.replace(path)
    except OSError as error:
        pytest.exit(
            f"Could not write gem5 selection plan to {target}: {error}",
            returncode=pytest.ExitCode.USAGE_ERROR,
        )
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


@pytest.fixture
def gem5(request, tmp_path):
    """Run a case with its explicitly prebuilt simulator."""
    root = request.config.rootpath
    build = request.node.get_closest_marker("gem5").kwargs["build"]
    binary = root / build

    def run(config, *args):
        if not binary.is_file():
            pytest.fail(f"Missing {build}. Build it with 'scons {build}'.")
        if not Path(config).is_absolute():
            config = root / config
        try:
            result = subprocess.run(
                [str(binary), "-d", str(tmp_path), str(config), *args],
                cwd=root,
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
            )
        except OSError as error:
            pytest.fail(f"Could not execute {binary}: {error}", pytrace=False)
        if result.returncode != 0:
            pytest.fail(
                f"gem5 exited with status {result.returncode}. "
                f"Output directory: {tmp_path}\n"
                f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                pytrace=False,
            )
        return result

    return run
