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

"""Build the offline coverage landing page and retained source browser."""

import hashlib
import html
import io
import json
import subprocess
import tarfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import (
    Path,
    PurePosixPath,
)

MAX_SOURCE_BYTES = 2 * 1024 * 1024
MAX_TOTAL_BYTES = 128 * 1024 * 1024


def safe_source(value):
    path = PurePosixPath(value)
    return (
        bool(value)
        and not path.is_absolute()
        and ".." not in path.parts
        and "\\" not in value
        and not any(ord(c) < 32 for c in value)
    )


def native_lines(output, language="native"):
    """Union executable lines from retained TestLib LCOV and native XML."""
    files = defaultdict(dict)
    for path in sorted(output.glob("*.info")):
        if path.name.startswith("python-") != (language == "python"):
            continue
        current = None
        for row in path.read_text().splitlines():
            if row.startswith("SF:"):
                current = row[3:]
            elif current and row.startswith("DA:"):
                line, count = row[3:].split(",")[:2]
                files[current][int(line)] = max(
                    files[current].get(int(line), 0), int(count)
                )
    aggregates = (
        sorted(output.glob("aggregate-*.xml")) if language == "native" else []
    )
    for path in aggregates:
        tree = ET.parse(path)
        for item in tree.iter("class"):
            name = item.get("filename", "")
            for line in item.findall("lines/line"):
                number, count = int(line.get("number")), int(line.get("hits"))
                files[name][number] = max(files[name].get(number, 0), count)
    return {path: lines for path, lines in files.items() if safe_source(path)}


def source_texts(source, requested, revision, checkout=None):
    """Read regular source members without extracting or following links."""
    found, unavailable, total = {}, {}, 0
    if checkout:
        checkout = Path(checkout).resolve()
        tree = subprocess.run(
            ["git", "ls-tree", "-r", "-l", "-z", revision, "--"],
            cwd=checkout,
            capture_output=True,
            check=True,
        ).stdout
        selected = []
        for row in tree.split(b"\0"):
            if not row:
                continue
            metadata, raw_name = row.split(b"\t", 1)
            mode, kind, identity, size = metadata.split()
            name = raw_name.decode("utf-8", errors="replace")
            if name not in requested or mode not in (b"100644", b"100755"):
                continue
            size = int(size)
            if size <= MAX_SOURCE_BYTES and total + size <= MAX_TOTAL_BYTES:
                selected.append((name, identity))
                total += size
        if selected:
            result = subprocess.run(
                ["git", "cat-file", "--batch"],
                cwd=checkout,
                input=b"\n".join(identity for _, identity in selected) + b"\n",
                capture_output=True,
                check=True,
            )
            stream = io.BytesIO(result.stdout)
            for name, _ in selected:
                identity, kind, size = stream.readline().split()
                found[name] = stream.read(int(size)).decode(
                    "utf-8", errors="replace"
                )
                stream.read(1)
    conflicting = set()
    archives = list(Path(source).rglob("raw-gcov.tar.gz"))
    archives.extend(Path(source).rglob("raw-profiles.tar.gz"))
    for archive_path in sorted(archives):
        try:
            with tarfile.open(archive_path, "r:gz") as archive:
                for member in archive:
                    name = member.name.removeprefix("./")
                    if archive_path.name == "raw-profiles.tar.gz":
                        parts = PurePosixPath(name).parts
                        if (
                            len(parts) < 6
                            or parts[:2] != ("coverage", "baselines")
                            or parts[3] != "sources"
                        ):
                            continue
                        name = str(PurePosixPath(*parts[4:]))
                    if (
                        name not in requested
                        or name in conflicting
                        or not member.isfile()
                        or not safe_source(name)
                    ):
                        continue
                    if (
                        member.size > MAX_SOURCE_BYTES
                        or total + member.size > MAX_TOTAL_BYTES
                    ):
                        unavailable[name] = (
                            "Source exceeds retained browser size limit"
                        )
                        continue
                    stream = archive.extractfile(member)
                    value = stream.read().decode("utf-8", errors="replace")
                    if name in found:
                        if found[name] != value:
                            del found[name]
                            conflicting.add(name)
                            unavailable[name] = (
                                "Different builds retained conflicting source contents"
                            )
                        continue
                    found[name] = value
                    total += member.size
        except (tarfile.TarError, OSError) as error:
            unavailable[str(archive_path.name)] = (
                f"Cannot read source archive: {error}"
            )
    for name in requested - found.keys():
        unavailable.setdefault(
            name, "Source was not retained at this revision"
        )
    return found, unavailable


STYLE = """<style>body{font:16px system-ui;margin:2rem auto;max-width:1100px;padding:0 1rem;color:#243746}a{color:#006b76}table{border-collapse:collapse;width:100%}td,th{padding:.5rem;text-align:left;border-bottom:1px solid #ddd}pre{overflow:auto}.hit{background:#e5f5e7}.miss{background:#ffe9e9}.number{display:inline-block;width:5em;color:#666}details{margin:1rem 0}</style>"""


def page(title, body):
    return (
        "<!doctype html><html lang='en'><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        f"<title>{html.escape(title)}</title>{STYLE}<body>{body}</body></html>"
    )


def build(source, output, revision, checkout=None):
    source, output = Path(source), Path(output)
    summary = json.loads((output / "summary.json").read_text())
    if summary["revision"] != revision:
        raise ValueError("Landing page revision mismatch")
    files = native_lines(output)
    python_files = native_lines(output, "python")
    requested = set(files) | set(python_files)
    for row in summary["suites"]:
        definition = row["test_uid"].split(":")[1]
        if not definition.startswith("tests/"):
            definition = "tests/" + definition
        if safe_source(definition):
            requested.add(definition)
    texts, unavailable = source_texts(source, requested, revision, checkout)
    source_map = {}
    for name, text in sorted(texts.items()):
        filename = hashlib.sha256(name.encode()).hexdigest() + ".html"
        lines = files.get(name, python_files.get(name, {}))
        rows = []
        for number, value in enumerate(text.splitlines(), 1):
            css = (
                "hit"
                if lines.get(number, 0) > 0
                else "miss" if number in lines else ""
            )
            rows.append(
                f'<span id="L{number}" class="{css}"><a class="number" href="#L{number}">{number}</a>{html.escape(value)}</span>'
            )
        target = output / "sources/pages" / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            page(
                name,
                f'<p><a href="../../index.html">Coverage report</a></p><h1>{html.escape(name)}</h1><p>Revision {html.escape(revision)}</p><pre>'
                + "\n".join(rows)
                + "</pre>",
            )
        )
        source_map[name] = {"page": "pages/" + filename}
    (output / "sources").mkdir(exist_ok=True)
    (output / "sources/map.json").write_text(
        json.dumps(source_map, sort_keys=True)
    )
    file_rows = []
    covered = executable = 0
    for name, lines in sorted(files.items()):
        hit = sum(count > 0 for count in lines.values())
        covered += hit
        executable += len(lines)
        label = html.escape(name)
        if name in source_map:
            label = f'<a href="sources/{source_map[name]["page"]}">{label}</a>'
        else:
            label += " (source unavailable)"
        file_rows.append(
            f"<tr><td>{label}</td><td>{hit}</td><td>{len(lines)}</td></tr>"
        )
    counts = summary["counts"]
    status = "Complete" if summary["complete"] else "Incomplete"
    body = f"<h1>gem5 coverage</h1><p><strong>{status} collection</strong></p><p>Revision <code>{html.escape(revision)}</code><br>Generated {html.escape(summary.get('generated_at', 'unknown'))}</p>"
    body += '<p><a href="index/index.html">Browse tests and line coverage</a> · <a href="summary.json">Accounting data</a> · <a href="uploads.json">Upload reports</a></p>'
    body += f"<p>{html.escape(summary['scope'])}</p>"
    body += (
        "<p>"
        + "; ".join(
            f"{html.escape(key.replace('_', ' '))}: {value}"
            for key, value in counts.items()
        )
        + "</p>"
    )
    body += (
        "<h2>Collection issues</h2><ul>"
        + "".join(
            f"<li>{html.escape(error)}</li>" for error in summary["errors"]
        )
        + "</ul>"
    )
    body += "<h2>Native aggregate groups</h2><p>These measure a complete group, not individual GTest or integration cases.</p><ul>"
    for row in summary.get("aggregates", []):
        body += (
            f"<li>{html.escape(row['group'])}: "
            + html.escape(
                ", ".join(
                    f"{key}={value}" for key, value in row["outcomes"].items()
                )
            )
            + f"; counters: {row['counters']['files']}; report retained: {row['report_present']}</li>"
        )
    body += "</ul><h2>Native combined line coverage</h2>"
    body += (
        f"<p>{covered:,} / {executable:,} executable lines hit. Union of TestLib and aggregate groups; counts use the maximum observed hit count, not an execution total.</p><table><tr><th>Source</th><th>Hit</th><th>Executable</th></tr>"
        + "".join(file_rows)
        + "</table>"
    )
    python_covered = sum(
        count > 0
        for lines in python_files.values()
        for count in lines.values()
    )
    python_executable = sum(len(lines) for lines in python_files.values())
    body += f"<h2>Python line coverage</h2><p>{python_covered:,} / {python_executable:,} executable Python lines hit after gem5 initialization. This scope is separate from native coverage.</p><table><tr><th>Source</th><th>Hit</th><th>Executable</th></tr>"
    for name, lines in sorted(python_files.items()):
        label = html.escape(name)
        if name in source_map:
            label = f'<a href="sources/{source_map[name]["page"]}">{label}</a>'
        else:
            label += " (source unavailable)"
        body += f"<tr><td>{label}</td><td>{sum(count > 0 for count in lines.values())}</td><td>{len(lines)}</td></tr>"
    body += "</table>"
    body += "<h2>Suite execution and exclusions</h2><p>Suite outcomes include verifier results; a successful gem5 process can still belong to a failed suite. The test index retains individual invocation outcomes.</p><ul>"
    for row in summary["suites"]:
        if (
            row["exclusion"]
            or row["outcome"] != "completed"
            or row["missing_profiles"]
        ):
            body += (
                f"<li>{html.escape(row['test_uid'])}: {html.escape(row['outcome'])}; missing profiles: {row['missing_profiles']}"
                + (
                    f"; excluded: {html.escape(row['exclusion'])}"
                    if row["exclusion"]
                    else ""
                )
                + "</li>"
            )
    body += (
        "</ul><details><summary>Source availability</summary><ul>"
        + "".join(
            f"<li>{html.escape(name)}: {html.escape(reason)}</li>"
            for name, reason in sorted(unavailable.items())
        )
        + "</ul></details>"
    )
    body += "<p>Report retry uses retained extracted profiles. Extraction retry additionally needs raw counters, matching notes and the original gcov toolchain; it does not rerun tests.</p>"
    (output / "index.html").write_text(page("gem5 coverage", body))
    return source_map
