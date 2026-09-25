#!/usr/bin/env python3
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

"""Build and query an offline, bidirectional TestLib coverage index."""

import argparse
import hashlib
import json
import os
import sys
from pathlib import (
    Path,
)
from urllib.parse import (
    quote,
    urlsplit,
)

try:
    from .schema import (
        COLLECTIONS,
        OUTCOMES,
        SparseProfile,
        normalize_record,
        read_index_records,
        read_records,
        source_path,
    )
except ImportError:  # Direct command-line execution.
    from schema import (
        COLLECTIONS,
        OUTCOMES,
        SparseProfile,
        normalize_record,
        read_index_records,
        read_records,
        source_path,
    )

REPOSITORY_URL = "https://github.com/gem5/gem5"


def attach_source_links(index, source_map_path, output_directory):
    """Resolve retained source pages inside their artifact before linking."""
    source_map_path = Path(source_map_path).resolve()
    root = source_map_path.parent
    mapping = json.loads(source_map_path.read_text(encoding="utf-8"))
    if not isinstance(mapping, dict):
        raise ValueError("Source map must contain a path-to-page mapping")
    links = {}
    for path, entry in mapping.items():
        source_path(path)
        if not isinstance(entry, dict):
            raise ValueError("Source map entries must describe a page")
        page = root / source_path(entry.get("page"))
        resolved = page.resolve()
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise ValueError("Source page is missing or escapes its artifact")
        if resolved.suffix != ".html":
            raise ValueError("Retained source pages must be HTML")
        relative = Path(
            os.path.relpath(resolved, Path(output_directory).resolve())
        )
        links[path] = "./" + quote(relative.as_posix(), safe="/")
    index["source_links"] = links
    return index


def build_index(records, repository_url=REPOSITORY_URL):
    """Stream profiles into a union with interned invocation memberships.

    A complete profile from a failed test still measures executed code. A
    missing/error profile contributes no denominator or zero-hit lines.
    """
    parsed = urlsplit(repository_url)
    if (
        parsed.scheme not in ("http", "https")
        or not parsed.netloc
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError("repository_url must be an HTTP(S) repository URL")
    invocations = {}
    fingerprints = {}
    inventories = {"native": {}, "python": {}}
    branch_inventories = {"native": {}, "python": {}}
    files = inventories["native"]
    revision = None
    collection = dict.fromkeys(COLLECTIONS, 0)
    outcomes = dict.fromkeys(OUTCOMES, 0)
    seeded = set()
    for raw in records:
        baseline = raw.baseline if isinstance(raw, SparseProfile) else None
        record = raw.record if baseline is not None else normalize_record(raw)
        identifier = record["invocation_id"]
        fingerprint = hashlib.sha256(
            json.dumps(
                record, sort_keys=True, separators=(",", ":"), allow_nan=False
            ).encode()
        ).hexdigest()
        if identifier in fingerprints:
            if fingerprints[identifier] != fingerprint:
                raise ValueError(
                    f"Conflicting records for invocation_id {identifier}"
                )
            continue
        if revision is not None and record["revision"] != revision:
            raise ValueError(
                "Cannot combine records from different source revisions"
            )
        revision = record["revision"]
        fingerprints[identifier] = fingerprint
        ordinal = len(invocations)
        bit = 1 << ordinal
        # Original files remain in the raw artifacts. Do not retain a full
        # zero-hit baseline per invocation in memory or in the browser index.
        metadata = {
            key: value for key, value in record.items() if key != "files"
        }
        metadata["record_sha256"] = fingerprint
        metadata["profile_summary"] = {
            "measured_lines": sum(
                len(item["lines"]) for item in (baseline or record)["files"]
            ),
            "covered_lines": sum(
                count > 0
                for item in record["files"]
                for count in item["lines"].values()
            ),
        }
        invocations[identifier] = metadata
        collection[record["collection"]] += 1
        outcomes[record["outcome"]] += 1
        if record["collection"] != "complete":
            continue
        language = record["language"]
        if baseline is not None and record["baseline_id"] not in seeded:
            seeded.add(record["baseline_id"])
            for entry in baseline["files"]:
                lines = inventories[language].setdefault(entry["path"], {})
                for number in entry["lines"]:
                    lines.setdefault(number, [0, 0])
                branches = branch_inventories[language].setdefault(
                    entry["path"], {}
                )
                for branch in entry.get("branches", []):
                    branches.setdefault(
                        branch["id"],
                        {
                            "metadata": {
                                key: value
                                for key, value in branch.items()
                                if key != "count"
                            },
                            "count": 0,
                            "membership": 0,
                        },
                    )
        for entry in record["files"]:
            lines = inventories[language].setdefault(entry["path"], {})
            for number, count in entry["lines"].items():
                line = lines.setdefault(number, [0, 0])
                line[0] += count
                if count > 0:
                    line[1] |= bit
            branches = branch_inventories[language].setdefault(
                entry["path"], {}
            )
            for branch in entry.get("branches", []):
                metadata = {
                    key: value
                    for key, value in branch.items()
                    if key != "count"
                }
                existing = branches.setdefault(
                    branch["id"],
                    {"metadata": metadata, "count": 0, "membership": 0},
                )
                if existing["metadata"] != metadata:
                    raise ValueError(
                        "Conflicting metadata for one branch identity"
                    )
                existing["count"] += branch["count"]
                if branch["count"] > 0:
                    existing["membership"] |= bit
    if not invocations:
        raise ValueError("No coverage.json invocation records found")
    ordered = [invocations[key] for key in sorted(invocations)]
    # Input order does not control public ordinals or membership IDs.
    ordinals = {item["invocation_id"]: i for i, item in enumerate(ordered)}
    remap = [ordinals[identifier] for identifier in invocations]
    masks = {
        line[1]
        for inventory in inventories.values()
        for lines in inventory.values()
        for line in lines.values()
    }
    masks.update(
        branch["membership"]
        for inventory in branch_inventories.values()
        for branches in inventory.values()
        for branch in branches.values()
    )
    memberships = {}
    for mask in masks:
        remaining = mask
        members = []
        while remaining:
            bit = remaining & -remaining
            members.append(remap[bit.bit_length() - 1])
            remaining ^= bit
        memberships[mask] = sorted(members)
    mask_order = sorted(masks, key=memberships.__getitem__)
    membership_ids = {mask: i for i, mask in enumerate(mask_order)}
    for inventory in inventories.values():
        for lines in inventory.values():
            for line in lines.values():
                line[1] = membership_ids[line[1]]
    for inventory in branch_inventories.values():
        for branches in inventory.values():
            for branch in branches.values():
                branch["membership"] = membership_ids[branch["membership"]]
    tests = {}
    for ordinal, record in enumerate(ordered):
        tests.setdefault(record["test_uid"], []).append(ordinal)
    files = {
        path: dict(sorted(lines.items(), key=lambda item: int(item[0])))
        for path, lines in sorted(files.items())
    }
    return {
        "schema_version": 1,
        "format": "gem5-coverage-index",
        "revision": revision,
        "repository_url": repository_url.rstrip("/"),
        "invocations": ordered,
        "tests": dict(sorted(tests.items())),
        "files": files,
        "python_files": inventories["python"],
        "branches": branch_inventories,
        "memberships": [memberships[mask] for mask in mask_order],
        "summary": {
            "invocations": len(ordered),
            "tests": len(tests),
            "collection": collection,
            "outcomes": outcomes,
            "languages": {
                language: {
                    "measured_lines": sum(
                        len(lines) for lines in inventory.values()
                    ),
                    "covered_lines": sum(
                        line[0] > 0
                        for lines in inventory.values()
                        for line in lines.values()
                    ),
                    "measured_branches": sum(
                        len(branches)
                        for branches in branch_inventories[language].values()
                    ),
                    "covered_branches": sum(
                        branch["count"] > 0
                        for branches in branch_inventories[language].values()
                        for branch in branches.values()
                    ),
                }
                for language, inventory in inventories.items()
            },
            "measured_lines": sum(len(lines) for lines in files.values()),
            "covered_lines": sum(
                line[0] > 0
                for lines in files.values()
                for line in lines.values()
            ),
        },
    }


def tests_for_line(index, path, line, language="native"):
    """Find only tests with positive hits; distinguish unknown from zero."""
    source_path(path)
    if type(line) is not int or line < 1:
        raise ValueError("A source line must be a positive integer")
    if language not in ("native", "python"):
        raise ValueError("Unknown coverage language")
    inventory = (
        index["files"]
        if language == "native"
        else index.get("python_files", {})
    )
    measured = inventory.get(path, {}).get(str(line))
    ordinals = index["memberships"][measured[1]] if measured else []
    tests = {}
    for ordinal in ordinals:
        record = index["invocations"][ordinal]
        tests.setdefault(record["test_uid"], []).append(
            record["invocation_id"]
        )
    return {
        "revision": index["revision"],
        "path": path,
        "line": line,
        "language": language,
        "measured": measured is not None,
        "count": measured[0] if measured else None,
        "tests": [
            {"test_uid": uid, "invocation_ids": ids}
            for uid, ids in sorted(tests.items())
        ],
        "collection": index["summary"]["collection"],
    }


def tests_for_branch(index, path, identity, language="native"):
    source_path(path)
    if language not in ("native", "python"):
        raise ValueError("Unknown coverage language")
    branch = (
        index.get("branches", {}).get(language, {}).get(path, {}).get(identity)
    )
    tests = {}
    if branch:
        for ordinal in index["memberships"][branch["membership"]]:
            record = index["invocations"][ordinal]
            tests.setdefault(record["test_uid"], []).append(
                record["invocation_id"]
            )
    return {
        "revision": index["revision"],
        "path": path,
        "branch_id": identity,
        "language": language,
        "measured": branch is not None,
        "count": branch["count"] if branch else None,
        "metadata": branch["metadata"] if branch else None,
        "tests": [
            {"test_uid": uid, "invocation_ids": ids}
            for uid, ids in sorted(tests.items())
        ],
    }


def lines_for_test(index, test_uid):
    """Return a test's executed lines, combining but retaining its attempts."""
    if test_uid not in index["tests"]:
        raise ValueError(f"Unknown test UID: {test_uid}")
    ordinals = set(index["tests"][test_uid])
    identifiers = [
        index["invocations"][i]["invocation_id"] for i in sorted(ordinals)
    ]
    collection = dict.fromkeys(COLLECTIONS, 0)
    for ordinal in ordinals:
        record = index["invocations"][ordinal]
        collection[record["collection"]] += 1
    matching = {}
    for membership, members in enumerate(index["memberships"]):
        overlap = [
            index["invocations"][i]["invocation_id"]
            for i in members
            if i in ordinals
        ]
        if overlap:
            matching[membership] = overlap
    lines = [
        {
            "path": path,
            "line": int(number),
            "language": language,
            "invocation_ids": matching[line[1]],
        }
        for language, inventory in (
            ("native", index["files"]),
            ("python", index.get("python_files", {})),
        )
        for path, measured in sorted(inventory.items())
        for number, line in sorted(
            measured.items(), key=lambda item: int(item[0])
        )
        if line[1] in matching
    ]
    return {
        "revision": index["revision"],
        "test_uid": test_uid,
        "invocation_ids": identifiers,
        "collection": collection,
        "coverage_known": collection["complete"] > 0,
        "lines": lines,
    }


def render_html(index):
    # JSON is inert data, but an HTML parser still recognizes </script>.
    # Escape all HTML delimiters before embedding; render values as text.
    data = json.dumps(
        index, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    )
    for char, escaped in (
        ("<", "\\u003c"),
        (">", "\\u003e"),
        ("&", "\\u0026"),
    ):
        data = data.replace(char, escaped)
    return HTML.replace("__COVERAGE_DATA__", data)


HTML = r"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>gem5 test coverage</title>
<style>
  :root { color-scheme: light; font-family: system-ui, sans-serif;
    color: #15323b; background: #f3f7f8; }
  body { max-width: 1200px; margin: 0 auto; padding: 2rem; }
  h1 { margin-bottom: .4rem; } h2 { margin-top: 0; }
  p { line-height: 1.5; } a { color: #006b80; }
  .muted { color: #4b626b; } #summary { font-weight: 600; }
  main { display: grid; grid-template-columns: 1fr 1fr; gap: 1.4rem; }
  section { min-width: 0; background: white; border: 1px solid #d9e5e8;
    border-radius: 10px; padding: 1.4rem; }
  label { display: block; margin: .8rem 0 .4rem; }
  input, select, button { box-sizing: border-box; font: inherit;
    border: 1px solid #94aeb7; border-radius: 5px; padding: .5rem; }
  input, select { width: 100%; background: white; color: inherit; }
  button { background: #006b80; color: white; cursor: pointer;
    margin-top: .7rem; } button:focus-visible, a:focus-visible {
    outline: 3px solid #eead35; outline-offset: 3px; }
  ul { padding-left: 1.3rem; } li { margin: .5rem 0; overflow-wrap: anywhere; }
  .scroll { overflow: auto; max-height: 30rem; }
  table { width: 100%; border-collapse: collapse; font-size: .9rem; }
  td, th { padding: .6rem .3rem; text-align: left;
    border-bottom: 1px solid #d9e5e8; overflow-wrap: anywhere; }
  code, pre { font-size: .85rem; overflow-wrap: anywhere; }
  pre { white-space: pre-wrap; } .note { border-left: 4px solid #bf8300;
    padding-left: .8rem; } footer { margin-top: 1.5rem; }
  @media(max-width: 760px) { main { grid-template-columns: 1fr; }
    body { padding: 1rem; } }
</style>
<header>
  <h1>gem5 test coverage</h1>
  <p class="muted" id="revision"></p>
  <p id="summary"></p>
  <p id="collection" class="note"></p>
  <label for="language">Coverage language</label>
  <select id="language"><option value="native">Native C/C++</option>
    <option value="python">Python</option></select>
</header>
<main>
  <section aria-labelledby="test-title">
    <h2 id="test-title">Test → source lines</h2>
    <label for="test-filter">Filter tests</label>
    <input id="test-filter" type="search" placeholder="Suite name or path">
    <label for="test-select">TestLib suite</label>
    <select id="test-select"></select>
    <p id="test-definition"></p>
    <p id="test-status" aria-live="polite"></p>
    <details><summary>Invocations and build details</summary>
      <ul id="invocations"></ul>
    </details>
    <label for="path-filter">Filter covered paths</label>
    <input id="path-filter" type="search" placeholder="src/cpu/">
    <p id="line-count" class="muted"></p>
    <div class="scroll"><table>
      <thead><tr><th>Source line</th><th>Invocations</th></tr></thead>
      <tbody id="covered-lines"></tbody>
    </table></div>
    <button id="more-lines" type="button" hidden>Show more lines</button>
  </section>
  <section aria-labelledby="line-title">
    <h2 id="line-title">Source line → tests</h2>
    <form id="lookup">
      <label for="location">Repository path:line</label>
      <input id="location" placeholder="src/cpu/base.cc:100" required>
      <button type="submit">Find covering tests</button>
    </form>
    <p id="line-status" aria-live="polite">Enter a source location.</p>
    <ul id="covering-tests"></ul>
  </section>
</main>
<footer class="muted">
  <p>Each record describes a TestLib gem5 invocation, identified by its suite.
  This does not identify individual cases within GTest or Python test runs.
  A complete profile from a failed test still contributes observed coverage.
  Counts sum invocations and retries; they are not a performance measurement.
  Source links open the recorded revision. This page works offline;
  following source or result links requires access to their host.</p>
</footer>
<script id="coverage-data" type="application/json">__COVERAGE_DATA__</script>
<script>
  "use strict";
  const data = JSON.parse(
    document.getElementById("coverage-data").textContent);
  const allTests = Object.keys(data.tests);
  const el = id => document.getElementById(id);
  const text = (tag, value) => {
    const node = document.createElement(tag); node.textContent = value;
    return node;
  };
  const safeLink = (label, url) => {
    try {
      const parsed = new URL(url);
      if (!["http:", "https:"].includes(parsed.protocol)) {
        return text("span", label);
      }
      const link = text("a", label); link.href = parsed.href;
      link.target = "_blank"; link.rel = "noopener noreferrer"; return link;
    } catch { return text("span", label); }
  };
  const sourceUrl = (path, line) => data.repository_url + "/blob/" +
    encodeURIComponent(data.revision) + "/" +
    path.split("/").map(encodeURIComponent).join("/") +
    (line ? "#L" + line : "");
  const sourceLink = (label, path, line) => {
    const local = (data.source_links || {})[path];
    if (typeof local === "string" && local.startsWith("./") &&
        !/[\\?#\x00-\x20]/.test(local) && !local.includes(":") &&
        !/%(?:2f|5c|00)/i.test(local)) {
      const link = text("a", label);
      link.href = local + (line ? "#L" + line : "");
      return link;
    }
    return safeLink(label, sourceUrl(path, line));
  };
  const summary = data.summary;
  el("revision").textContent = "Source revision: " + data.revision;
  el("summary").textContent = summary.tests + " suites · " +
    summary.invocations + " invocations · " + summary.covered_lines +
    " covered / " + summary.measured_lines + " measured native lines";
  const language = () => el("language").value;
  const inventory = () => language() === "native" ? data.files : (data.python_files || {});
  if (summary.languages) {
    el("summary").textContent = Object.entries(summary.languages).map(([name, s]) =>
      name + ": " + s.covered_lines + "/" + s.measured_lines + " lines, " +
      s.covered_branches + "/" + s.measured_branches + " branches").join(" · ");
  }
  const statuses = c => c.complete + " complete, " + c.missing +
    " missing, " + c.error + " extraction errors";
  el("collection").textContent = "Profiles: " + statuses(summary.collection) +
    ". Missing/error profiles do not count as zero coverage. " +
    "The denominator contains only lines reported by complete profiles, " +
    "not all gem5 code.";
  let selectedLines = [], visibleCount = 200;

  function drawLines() {
    const filter = el("path-filter").value.toLowerCase();
    const lines = selectedLines.filter(item =>
      item.path.toLowerCase().includes(filter));
    const body = el("covered-lines"); body.replaceChildren();
    for (const item of lines.slice(0, visibleCount)) {
      const row = document.createElement("tr");
      const location = item.path + ":" + item.line;
      const cell = document.createElement("td");
      cell.append(sourceLink(location, item.path, item.line));
      row.append(cell, text("td", item.ids.length));
      body.append(row);
    }
    el("line-count").textContent = lines.length + " covered lines; showing " +
      Math.min(lines.length, visibleCount) + ".";
    el("more-lines").hidden = visibleCount >= lines.length;
  }

  function selectTest() {
    const uid = el("test-select").value;
    selectedLines = []; visibleCount = 200;
    el("invocations").replaceChildren();
    el("test-definition").replaceChildren();
    if (!uid) { el("test-status").textContent = "No matching tests.";
      drawLines(); return; }
    const definition = uid.split(":")[1];
    if (definition && !definition.split("/").includes("..")) {
      const path = definition.startsWith("tests/") ?
        definition : "tests/" + definition;
      el("test-definition").append(sourceLink("Open test definition", path));
    }
    const counts = {complete: 0, missing: 0, error: 0};
    const ordinals = new Set(data.tests[uid].filter(ordinal =>
      (data.invocations[ordinal].language || "native") === language()));
    for (const ordinal of ordinals) {
      const item = data.invocations[ordinal]; counts[item.collection]++;
      const li = text("li", item.invocation_id + " — " +
        item.outcome + " / " + item.collection);
      const detail = document.createElement("details");
      detail.append(text("summary", "Build details"),
        text("pre", JSON.stringify(item.build, null, 2)));
      li.append(detail);
      if (item.results_url) {
        li.append(safeLink("Open results", item.results_url));
      }
      el("invocations").append(li);
    }
    const matching = new Map();
    data.memberships.forEach((members, membership) => {
      const overlap = members.filter(ordinal => ordinals.has(ordinal));
      if (overlap.length) matching.set(membership, overlap);
    });
    for (const [path, measured] of Object.entries(inventory())) {
      for (const [number, line] of Object.entries(measured)) {
        if (!matching.has(line[1])) continue;
        selectedLines.push({path, line: Number(number),
          ids: matching.get(line[1])});
      }
    }
    el("test-status").textContent = "Profiles: " + statuses(counts) +
      (counts.complete ? ". Coverage shown from complete profiles." :
        ". Coverage is unknown; no complete profile was collected.");
    selectedLines.sort((a, b) =>
      a.path < b.path ? -1 : a.path > b.path ? 1 : a.line - b.line);
    drawLines();
  }

  function filterTests(preferred) {
    const filter = el("test-filter").value.toLowerCase();
    const select = el("test-select"); select.replaceChildren();
    const matches = allTests.filter(uid => uid.toLowerCase().includes(filter));
    for (const uid of matches) {
      const option = text("option", uid); option.value = uid;
      select.append(option);
    }
    if (preferred && matches.includes(preferred)) {
      select.value = preferred;
    }
    selectTest();
  }

  el("lookup").addEventListener("submit", event => {
    event.preventDefault(); el("covering-tests").replaceChildren();
    const match = /^(.*):([1-9][0-9]*)$/.exec(el("location").value.trim());
    if (!match) {
      el("line-status").textContent = "Use a repository path:line, " +
        "for example src/cpu/base.cc:100."; return;
    }
    const [, path, number] = match;
    const selected = inventory();
    const file = Object.hasOwn(selected, path) ? selected[path] : null;
    const line = file && Object.hasOwn(file, number) ? file[number] : null;
    if (!line) { el("line-status").textContent =
      "No complete profile measured this line; coverage is unknown."; return; }
    const members = data.memberships[line[1]];
    const branchSet = (data.branches || {})[language()] || {};
    for (const branch of Object.values(branchSet[path] || {})) {
      if (branch.metadata.line !== Number(number)) continue;
      const suites = [...new Set(data.memberships[branch.membership].map(i =>
        data.invocations[i].test_uid))];
      el("covering-tests").append(text("li", "Branch " + branch.metadata.id +
        ": " + branch.count + " hits; " +
        (suites.length ? suites.join(", ") : "no covering suite")));
    }
    el("line-status").replaceChildren(sourceLink(path + ":" + number, path, number), document.createTextNode(" — " + line[0] +
      " hits across " + members.length + " invocations."));
    if (!members.length) {
      el("covering-tests").append(text("li",
        "Measured, but no complete profile covered this line.")); return;
    }
    const groups = new Map();
    for (const ordinal of members) {
      const item = data.invocations[ordinal];
      const uid = item.test_uid;
      if (!groups.has(uid)) groups.set(uid, []);
      groups.get(uid).push(item.invocation_id);
    }
    for (const uid of [...groups.keys()].sort()) {
      const li = document.createElement("li");
      const button = text("button", uid);
      button.type = "button";
      button.addEventListener("click", () => {
        el("test-filter").value = ""; filterTests(uid);
        el("test-select").focus();
      });
      li.append(button,
        text("p", "Invocations: " + groups.get(uid).join(", ")));
      el("covering-tests").append(li);
    }
  });
  el("language").addEventListener("change", () => {
    selectTest(); el("covering-tests").replaceChildren();
    el("line-status").textContent = "Enter a source location.";
  });
  el("test-filter").addEventListener("input", () =>
    filterTests(el("test-select").value));
  el("test-select").addEventListener("change", selectTest);
  el("path-filter").addEventListener("input", () => {
    visibleCount = 200; drawLines();
  });
  el("more-lines").addEventListener("click", () => {
    visibleCount += 200; drawLines();
  });
  filterTests();
</script>
</html>
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    build = commands.add_parser(
        "build", help="Build index.json and index.html"
    )
    build.add_argument(
        "input", type=Path, help="Search recursively for coverage.json"
    )
    build.add_argument("--output", required=True, type=Path)
    build.add_argument("--repository-url", default=REPOSITORY_URL)
    build.add_argument(
        "--source-map", type=Path, help="Retained sources/map.json"
    )
    reverse = commands.add_parser(
        "tests-for-line", help="Find tests covering a line"
    )
    reverse.add_argument("index", type=Path, help="The generated index.json")
    reverse.add_argument("location", help="Repository-relative path:line")
    reverse.add_argument(
        "--language", choices=("native", "python"), default="native"
    )
    branch = commands.add_parser(
        "tests-for-branch", help="Find tests covering a branch"
    )
    branch.add_argument("index", type=Path)
    branch.add_argument("path")
    branch.add_argument("identity")
    branch.add_argument(
        "--language", choices=("native", "python"), default="native"
    )
    forward = commands.add_parser(
        "lines-for-test", help="Find lines covered by a suite"
    )
    forward.add_argument("index", type=Path, help="The generated index.json")
    forward.add_argument("test_uid")
    args = parser.parse_args(argv)
    try:
        if args.command == "build":
            result = build_index(
                read_index_records(args.input), args.repository_url
            )
            if args.source_map:
                attach_source_links(result, args.source_map, args.output)
            args.output.mkdir(parents=True, exist_ok=True)
            (args.output / "index.json").write_text(
                json.dumps(
                    result,
                    ensure_ascii=True,
                    sort_keys=True,
                    separators=(",", ":"),
                )
                + "\n",
                encoding="utf-8",
            )
            (args.output / "index.html").write_text(
                render_html(result), encoding="utf-8"
            )
            print(json.dumps(result["summary"], sort_keys=True))
        else:
            stored = json.loads(args.index.read_text(encoding="utf-8"))
            if (
                not isinstance(stored, dict)
                or stored.get("schema_version") != 1
                or stored.get("format") != "gem5-coverage-index"
            ):
                raise ValueError("Unsupported coverage index schema_version")
            result = stored
            if args.command == "tests-for-line":
                path, separator, number = args.location.rpartition(":")
                if not separator or not number.isdecimal():
                    raise ValueError(
                        "Expected a repository-relative path:line"
                    )
                answer = tests_for_line(
                    result, path, int(number), args.language
                )
            elif args.command == "tests-for-branch":
                answer = tests_for_branch(
                    result, args.path, args.identity, args.language
                )
            else:
                answer = lines_for_test(result, args.test_uid)
            print(json.dumps(answer, ensure_ascii=True, indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        print(f"coverage index: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
