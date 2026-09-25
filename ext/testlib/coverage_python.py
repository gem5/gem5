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
#

"""Run a gem5 file configuration with separate Python coverage contexts.

Invoked by TestLib as a gem5 configuration. Keep dependencies out of ordinary
runs and restore the original argv, script path and __m5_main__ globals.
"""

import hashlib
import json
import os
import sys
from pathlib import Path


def _identity(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _write(path, record):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, sort_keys=True) + "\n")
    temporary.replace(path)


def _finish(collector, record, path):
    collector.stop()
    collector.save()
    import coverage

    root = Path(record["source_root"])
    compiled = Path(record["compiled_root"])

    def map_path(filename):
        try:
            return str(root / Path(filename).relative_to(compiled))
        except ValueError:
            return filename

    # Embedded Python may retain the producer's absolute source filenames.
    # Keep original raw data, then map a separate report database to checkout.
    mapped_path = path.parent / ".coverage-mapped"
    mapped = coverage.CoverageData(basename=str(mapped_path))
    mapped.update(collector.get_data(), map_path=map_path)
    mapped.write()
    reporter = coverage.Coverage(data_file=str(mapped_path), config_file=False)
    reporter.load()
    json_path = path.parent / "python-contexts.json"
    reporter.json_report(outfile=str(json_path), show_contexts=True)
    report = json.loads(json_path.read_text())
    files = []
    for filename, entry in sorted(report["files"].items()):
        try:
            relative = Path(filename).resolve().relative_to(root).as_posix()
        except ValueError:
            continue
        lines = {str(line): 0 for line in entry["missing_lines"]}
        lines.update({str(line): 1 for line in entry["executed_lines"]})
        executed = {tuple(arc) for arc in entry.get("executed_branches", [])}
        missing = {tuple(arc) for arc in entry.get("missing_branches", [])}
        branches = [
            {
                "id": _identity([relative, line, target]),
                "line": line,
                "to_line": target,
                "count": int((line, target) in executed),
            }
            for line, target in sorted(executed | missing)
        ]
        files.append({"path": relative, "lines": lines, "branches": branches})
    record["files"] = files
    record["collection"] = "complete" if files else "missing"


def _main():
    import m5

    path = Path(sys.argv[1])
    sys.argv = sys.argv[2:]
    filename = sys.argv[0]
    if not m5.options.P:
        # m5.main prepended this wrapper's directory. Restore exactly what
        # it would have prepended for the user's original configuration.
        sys.path[0] = os.path.dirname(filename)
    record = json.loads(path.read_text())
    collector = None
    try:
        import coverage

        root = Path(record["source_root"])
        compiled = Path(record["compiled_root"])
        collector = coverage.Coverage(
            data_file=str(path.parent / ".coverage"),
            config_file=False,
            branch=True,
            include=[str(root / "*"), str(compiled / "*")],
            omit=[str(Path(__file__).resolve())],
            context=record["test_uid"] + "|" + record["parent_invocation_id"],
        )
        compatibility = {
            "gem5_compatibility_id": record["build"].get(
                "gem5_compatibility_id", "unknown"
            ),
            "python_version": sys.version.split()[0],
            "coverage_version": coverage.__version__,
            "instrumentation": "coverage.py branch contexts",
            "scope": "config",
        }
        record["build"] = dict(compatibility)
        if compatibility["gem5_compatibility_id"] != "unknown":
            record["build"]["compatibility_id"] = _identity(compatibility)
        collector.start()
    except Exception as error:
        record["collection"] = "error"
        record["error"] = str(error)
        collector = None
        print(f"Python coverage preparation failed: {error}", file=sys.stderr)
    try:
        # Match m5.main's file execution semantics, including its explicit
        # UTF-8 decode and __m5_main__ name (runpy uses different globals).
        with open(filename, "rb") as stream:
            code = compile(stream.read().decode("utf-8"), filename, "exec")
        scope = {"__file__": filename, "__name__": "__m5_main__"}
        exec(code, scope)
        record["outcome"] = "passed"
    except SystemExit as error:
        record["outcome"] = "passed" if error.code in (None, 0) else "failed"
        raise
    except KeyboardInterrupt:
        record["outcome"] = "interrupted"
        raise
    except BaseException:
        record["outcome"] = "failed"
        raise
    finally:
        if collector is not None:
            try:
                _finish(collector, record, path)
            except Exception as error:
                record["collection"] = "error"
                record["error"] = str(error)
                print(
                    f"Python coverage collection failed: {error}",
                    file=sys.stderr,
                )
        try:
            _write(path, record)
        except Exception as error:
            print(
                f"Cannot save Python coverage result: {error}", file=sys.stderr
            )


if __name__ in ("__main__", "__m5_main__"):
    _main()
