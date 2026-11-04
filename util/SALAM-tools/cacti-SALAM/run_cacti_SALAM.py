#!/usr/bin/env python3

# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import import_cacti
import yaml

_configurator = Path(__file__).resolve().parents[1] / "SALAM-Configurator"
if str(_configurator) not in sys.path:
    sys.path.insert(0, str(_configurator))
from config_parser import load_clusters  # noqa: E402
from power_model import CharacterizationError  # noqa: E402
from workload_config import WorkloadConfigError  # noqa: E402

SCRIPT_DIR = Path(__file__).parent.resolve()
CACTI_DEFAULT_FILE = SCRIPT_DIR / "cacti_salam.cfg"


def collect_mem_objects(config_path, working_dir=None):
    """Collect SPMs from the simulator's include-resolved workload model.

    Ports retains the normalized workload field's meaning. Connections are
    recorded separately: this discovery helper does not infer a characterized
    physical organization from the simulator's connected endpoints.
    """
    config_path = Path(os.path.abspath(config_path))
    working_dir = (
        config_path.parent
        if working_dir is None
        else Path(os.path.abspath(working_dir))
    )
    clusters = load_clusters(str(working_dir), str(config_path))
    memories = []
    for cluster in clusters:
        for accelerator in cluster.accs:
            for variable in accelerator.variables:
                if variable.type != "SPM":
                    continue
                memories.append(
                    {
                        "Identity": (
                            f"{cluster.name}/{accelerator.name}/"
                            f"{variable.name}"
                        ),
                        "Name": variable.name,
                        "Size": variable.size,
                        "Ports": variable.ports,
                        "Source": cluster.config_path,
                        "Address": variable.address,
                        "Characterization": variable.characterization,
                        "Connections": tuple(
                            (connection.conName, connection.numPorts)
                            for connection in variable.connections
                        ),
                    }
                )
    return memories


def read_geometry(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise CharacterizationError(f"duplicate geometry key {key}")
            result[key] = value
        return result

    return json.loads(Path(path).read_text(), object_pairs_hook=unique)


def prepare_input(memory, geometry, template):
    """Validate an explicit physical organization against resolved endpoints.

    The shared cluster builder connects one generic host port and every
    private connection. Matching that endpoint count does not infer banks,
    word width, arbitration, or multiplexing for a physical implementation.
    """
    identity = f"{memory['Identity']} ({memory['Source']})"
    fields = {
        "capacity_bytes": "-size (bytes)",
        "word_bytes": "-block size (bytes)",
        "banks": "-UCA bank count",
        "rw_ports": "-read-write port",
        "read_ports": "-exclusive read port",
        "write_ports": "-exclusive write port",
    }
    if not isinstance(geometry, dict) or set(geometry) != {
        *fields,
        "mask_model",
    }:
        raise CharacterizationError(
            f"{identity}: explicit importer geometry is required: "
            f"{sorted([*fields, 'mask_model'])}"
        )
    for key in fields:
        value = geometry[key]
        if type(value) is not int or value < 0:
            raise CharacterizationError(f"{identity}: invalid {key}")
        if key not in {"read_ports", "write_ports"} and not value:
            raise CharacterizationError(f"{identity}: {key} must be positive")
    if geometry["capacity_bytes"] != memory["Size"]:
        raise CharacterizationError(
            f"{identity}: capacity must equal resolved Size={memory['Size']}; "
            "padding/substitution is not same-device characterization"
        )
    endpoints = 1 + sum(count for _, count in memory["Connections"])
    if (
        geometry["read_ports"] != 0
        or geometry["write_ports"] != 0
        or geometry["rw_ports"] != endpoints
    ):
        raise CharacterizationError(
            f"{identity}: physical organization must explicitly provide "
            f"{endpoints} RW ports (all private connections plus one generic "
            "host port), with no dedicated read/write ports; an arbiter or "
            "multiplexed alternative is not modeled"
        )
    if geometry["mask_model"] not in ("read_modify_write", "unsupported"):
        raise CharacterizationError(
            f"{identity}: CACTI does not establish byte-enable support"
        )
    replacements = {
        prefix: str(geometry[key]) for key, prefix in fields.items()
    }
    replacements["-output/input bus width"] = str(8 * geometry["word_bytes"])
    seen = set()
    lines = []
    for line in template.splitlines():
        prefix = next(
            (
                key
                for key in replacements
                if line.strip().startswith(key + " ")
            ),
            None,
        )
        if prefix is not None:
            if prefix in seen:
                raise CharacterizationError(
                    f"{identity}: duplicate template field {prefix}"
                )
            seen.add(prefix)
            line = f"{prefix} {replacements[prefix]}"
        lines.append(line)
    if seen != set(replacements):
        raise CharacterizationError(
            f"{identity}: template lacks {sorted(set(replacements) - seen)}"
        )
    text = "\n".join(lines) + "\n"
    import_cacti.read_config(text)
    return text


def produce(
    memory,
    geometry,
    template,
    source_dir,
    binary,
    format_id,
    output_dir,
    timeout=300,
):
    """Run one producer in a new directory and import exactly one result."""
    text = prepare_input(memory, geometry, Path(template).read_text())
    source_dir = Path(source_dir).resolve()
    binary = Path(binary).resolve()
    selected = import_cacti.FORMATS[format_id]
    if any(
        import_cacti.digest(source_dir / name) != digest
        for name, digest in selected["hashes"].items()
    ):
        raise CharacterizationError("CACTI source identity mismatch")
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise CharacterizationError(
            f"CACTI binary is not executable: {binary}"
        )
    if not math.isfinite(timeout) or timeout <= 0:
        raise CharacterizationError("timeout must be positive and finite")
    runtime_files = {}
    if format_id == "hp-cacti-7.0":
        # HP CACTI opens technology tables relative to its working directory.
        # Retain the exact tables alongside the raw producer input/output.
        runtime_files = {
            path.relative_to(source_dir): path
            for path in sorted((source_dir / "tech_params").glob("*.dat"))
        }
        if not runtime_files:
            raise CharacterizationError(
                "HP CACTI source-dir must include tech_params/*.dat"
            )
    if (source_dir / "contention.dat").is_file():
        runtime_files[Path("contention.dat")] = source_dir / "contention.dat"
    output_dir = Path(output_dir).absolute()
    # Never consume stale CSVs or overwrite evidence from an earlier run.
    output_dir.mkdir(parents=True, exist_ok=False)

    def save(name, value):
        (output_dir / name).write_text(
            json.dumps(value, indent=2, allow_nan=False) + "\n"
        )

    config = output_dir / "input.cfg"
    config.write_text(text)
    runtime_hashes = {}
    for relative, original in runtime_files.items():
        target = output_dir / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
        runtime_hashes[str(relative)] = import_cacti.digest(target)
    save("geometry.json", geometry)
    command = [str(binary), "-infile", str(config)]
    save("command.json", command)
    source = Path(memory["Source"])
    (output_dir / "declaring-workload.yaml").write_bytes(source.read_bytes())
    evidence = {
        "memory": memory,
        "declaring_workload_sha256": import_cacti.digest(source),
        "template_path": str(Path(template).resolve()),
        "template_sha256": import_cacti.digest(template),
        "format": format_id,
        "source_dir": str(source_dir),
        "source_hashes": selected["hashes"],
        "binary_sha256": import_cacti.digest(binary),
        "input_sha256": import_cacti.digest(config),
        "runtime_input_sha256": runtime_hashes,
        "command": command,
        "cwd": str(output_dir),
        "environment": {
            key: os.environ.get(key)
            for key in ("LANG", "LC_ALL", "OMP_NUM_THREADS")
        },
        "timeout_s": timeout,
        "status": "running",
    }
    save("execution.json", evidence)
    try:
        try:
            process = subprocess.run(
                command,
                cwd=output_dir,
                capture_output=True,
                text=True,
                check=False,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            for name, data in (
                ("stdout.log", exc.stdout),
                ("stderr.log", exc.stderr),
            ):
                (output_dir / name).write_bytes(
                    data.encode() if isinstance(data, str) else data or b""
                )
            raise CharacterizationError("CACTI producer timed out") from exc
        (output_dir / "stdout.log").write_text(process.stdout)
        (output_dir / "stderr.log").write_text(process.stderr)
        evidence["exit_code"] = process.returncode
        if process.returncode != 0 or "ERROR:" in process.stdout:
            raise CharacterizationError(
                f"CACTI producer failed (exit {process.returncode}); see logs"
            )
        csv_path = output_dir / (
            "out.csv" if format_id == "gem5-cacti-6.5" else "input.cfg.out"
        )
        if evidence["input_sha256"] != import_cacti.digest(config):
            raise CharacterizationError("CACTI input changed during run")
        if any(
            import_cacti.digest(output_dir / relative) != digest
            for relative, digest in runtime_hashes.items()
        ):
            raise CharacterizationError(
                "CACTI runtime data changed during run"
            )
        document = import_cacti.import_record(
            csv_path,
            config,
            source_dir,
            geometry,
            memory["Identity"],
            command,
            binary,
            format_id,
        )
        if evidence["binary_sha256"] != import_cacti.digest(binary):
            raise CharacterizationError("CACTI binary changed during run")
        if evidence["declaring_workload_sha256"] != import_cacti.digest(
            source
        ):
            raise CharacterizationError(
                "declaring workload changed during run"
            )
        save("characterization.json", document)
        evidence.update(
            status="imported", output_sha256=import_cacti.digest(csv_path)
        )
    except (CharacterizationError, OSError) as exc:
        evidence.update(status="rejected", error=str(exc))
        raise CharacterizationError(
            f"{memory['Identity']} ({memory['Source']}): {exc}; "
            f"evidence retained in {output_dir}"
        ) from exc
    finally:
        save("execution.json", evidence)
    return document


def main():
    parser = argparse.ArgumentParser(
        description="Produce an exact CACTI characterization for one SALAM SPM.",
        epilog="The old --bench-list sweep did not establish physical "
        "organizations. Use --list to discover include-resolved identities, "
        "then run each memory with explicit importer geometry and a new "
        "--output-dir. No M5_PATH/ACC_BENCH_PATH variables are required.",
    )
    parser.add_argument("--config", type=Path)
    parser.add_argument("--bench-list", help=argparse.SUPPRESS)
    parser.add_argument("--delay", help=argparse.SUPPRESS)
    parser.add_argument(
        "--workload-root",
        type=Path,
        help="SysPath/IrPath root, as in the simulator",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print resolved SPM identities/topology as JSON",
    )
    parser.add_argument("--memory", help="Exact identity printed by --list")
    parser.add_argument(
        "--geometry",
        type=Path,
        help="Explicit importer geometry JSON for this memory",
    )
    parser.add_argument("--template", type=Path, default=CACTI_DEFAULT_FILE)
    parser.add_argument("--source-dir", type=Path)
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--format", choices=import_cacti.FORMATS)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="New evidence directory; must not already exist",
    )
    parser.add_argument("--timeout", type=float, default=300)
    args = parser.parse_args()
    if args.bench_list is not None or args.delay is not None:
        parser.error(
            "the --bench-list sweep is replaced by explicit per-memory "
            "production: use --config PATH --list, then select --memory "
            "with --geometry and a fresh --output-dir; see --help"
        )
    if args.config is None:
        parser.error("--config is required; add --list to discover memories")
    try:
        memories = collect_mem_objects(args.config, args.workload_root)
        if args.list:
            print(json.dumps(memories, indent=2))
            return
        for name in (
            "memory",
            "geometry",
            "source_dir",
            "binary",
            "format",
            "output_dir",
        ):
            if getattr(args, name) is None:
                parser.error(
                    f"--{name.replace('_', '-')} is required without --list"
                )
        selected = [m for m in memories if m["Identity"] == args.memory]
        if len(selected) != 1:
            raise CharacterizationError(
                f"{args.config}: --memory {args.memory!r} must select exactly "
                "one resolved SPM; use --list"
            )
        produce(
            selected[0],
            read_geometry(args.geometry),
            args.template,
            args.source_dir,
            args.binary,
            args.format,
            args.output_dir,
            args.timeout,
        )
    except (
        CharacterizationError,
        WorkloadConfigError,
        yaml.YAMLError,
        OSError,
        ValueError,
    ) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
