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

import argparse
import csv
import hashlib
import io
import json
import math
import re
import sys
from fractions import Fraction
from pathlib import Path

_CONFIGURATOR = Path(__file__).resolve().parents[1] / "SALAM-Configurator"
if str(_CONFIGURATOR) not in sys.path:
    sys.path.insert(0, str(_CONFIGURATOR))
from power_model import (
    CharacterizationError,
    normalize,
)

# io.cc prints per-access nJ, summed gate/non-gate leakage per bank in mW,
# and whole-memory mm2. uca.cc and cacti_interface.cc define those scopes.
# parameter.cc/const.h define the effective 2D SRAM subarray capacity.
FORMATS = {
    "gem5-cacti-6.5": {
        "revision": "0deef376d96bfe0a3a2496714ac22471d9ee818a",
        "url": "https://github.com/gem5/gem5/tree/0deef376d96bfe0a3a2496714ac22471d9ee818a/ext/mcpat/cacti",
        "hashes": {
            "io.cc": "60e8072514526e8133dee5409936ffb3b37a77fcce1dde77e0123f009bc3ed56",
            "uca.cc": "197bc35665a883a6e2d7b4afa9f04d4ef1f8e33626c4fe1598b97f69621e9248",
            "cacti_interface.cc": "3ecb77e00745ea7e5e24b891f10af942e2cc9adf79adc630956b527101f51c4f",
            "parameter.cc": "57d8753aaffd92b59a6aa1320321809dcfbd4c64c5d141f22bd5bde29ce8e365",
            "const.h": "060bacd7b94ea4451a9d0475f93fcc1d25b319a64ce85d2527ab100feee582e1",
        },
    },
    "hp-cacti-7.0": {
        "revision": "1ffd8dfb10303d306ecd8d215320aea07651e878",
        "url": "https://github.com/HewlettPackard/cacti/tree/1ffd8dfb10303d306ecd8d215320aea07651e878",
        "hashes": {
            "io.cc": "78584c149e83d1da9a33a55392c2233fea3144696050f1ddc3b2b65bab7831be",
            "uca.cc": "94960f04863c478af9849a5e82a094fa46557dfa2eff7c061f50997ff616dc1b",
            "cacti_interface.cc": "5f2de8c7b743ea2ab43202b1dc0db65d9c84ef4b1fc2ac8dc59f5b9304aa03b4",
            "parameter.cc": "c5a889134dc231f262fe8d9fe61fd7e856bf67445ca5c0db3d0e9fa7072c2ab0",
            "const.h": "8cb43a197cd73685a2a8d70804728a4879246772860ab48b7d52c723e64152ad",
        },
    },
}
HEADERS = (
    "Tech node (nm)",
    "Capacity (bytes)",
    "Number of banks",
    "Associativity",
    "Output width (bits)",
    "Access time (ns)",
    "Random cycle time (ns)",
    "Dynamic search energy (nJ)",
    "Dynamic read energy (nJ)",
    "Dynamic write energy (nJ)",
    "Standby leakage per bank(mW)",
    "Area (mm2)",
    "Ndwl",
    "Ndbl",
    "Nspd",
    "Ndcm",
    "Ndsam_level_1",
    "Ndsam_level_2",
    "Data arrary area efficiency %",
    "Ntwl",
    "Ntbl",
    "Ntspd",
    "Ntcm",
    "Ntsam_level_1",
    "Ntsam_level_2",
    "Tag arrary area efficiency %",
)
_FIELDS = {
    "capacity_bytes": "-size (bytes)",
    "word_bytes": "-block size (bytes)",
    "banks": "-UCA bank count",
    "rw_ports": "-read-write port",
    "read_ports": "-exclusive read port",
    "write_ports": "-exclusive write port",
    "single_ended_ports": "-single ended read ports",
    "output_bits": "-output/input bus width",
    "temperature_k": "-operating temperature (K)",
    "technology_um": "-technology (u)",
    "associativity": "-associativity",
    "cache_type": "-cache type",
    "cache_model": "-Cache model (NUCA, UCA)",
    "ecc": "-Add ECC",
    "cell_type": "-Data array cell type -",
    "peripheral_type": "-Data array peripheral type -",
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value, name, positive=False):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CharacterizationError(f"{name}: invalid numeric value") from exc
    if isinstance(value, bool) or not math.isfinite(result) or result < 0:
        raise CharacterizationError(f"{name}: invalid numeric value")
    if positive and result == 0:
        raise CharacterizationError(f"{name}: must be positive")
    return result


def integer(value, name, positive=False):
    if not re.fullmatch(r"[0-9]+", str(value)):
        raise CharacterizationError(f"{name}: expected integer")
    result = int(value)
    if positive and result == 0:
        raise CharacterizationError(f"{name}: must be positive")
    return result


def read_config(text):
    values = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].split("//", 1)[0].strip()
        if (
            "Power Gating" in line and "Performance Loss" not in line
        ) or line.startswith("-Bitline floating"):
            if '"false"' not in line:
                raise CharacterizationError("power gating is unsupported")
        for name, prefix in _FIELDS.items():
            if not line.startswith(prefix + " "):
                continue
            if name in values:
                raise CharacterizationError(f"duplicate config field {name}")
            value = line[len(prefix) :].strip()
            if name == "ecc":
                value = value.removeprefix("-").strip()
            values[name] = value.strip('"')
    if set(values) != set(_FIELDS):
        raise CharacterizationError(
            f"missing config fields {sorted(set(_FIELDS) - set(values))}"
        )
    for name in set(_FIELDS) - {
        "cache_type",
        "cache_model",
        "cell_type",
        "peripheral_type",
        "technology_um",
        "ecc",
    }:
        values[name] = integer(values[name], name)
    values["technology_um"] = finite(
        values["technology_um"], "technology", positive=True
    )
    if (
        values["cache_type"] != "ram"
        or values["cache_model"] != "UCA"
        or values["associativity"] != 1
    ):
        raise CharacterizationError(
            "only non-associative UCA SRAM is supported"
        )
    if values["ecc"] != "false":
        raise CharacterizationError("ECC storage is not mapped")
    for field in ("cell_type", "peripheral_type"):
        if values[field] not in {"itrs-hp", "itrs-lstp", "itrs-lop"}:
            raise CharacterizationError("unsupported SRAM transistor type")
    if not values["temperature_k"]:
        raise CharacterizationError("temperature must be positive Kelvin")
    if values["single_ended_ports"]:
        raise CharacterizationError("single-ended read ports are unsupported")
    if values["output_bits"] != 8 * values["word_bytes"]:
        raise CharacterizationError("block and access widths must match")
    return values


def read_csv(text):
    rows = []
    for row in csv.reader(io.StringIO(text)):
        row = [item.strip() for item in row]
        if row and not row[-1]:
            row.pop()
        if row:
            rows.append(row)
    if len(rows) != 2:
        raise CharacterizationError("expected one CSV header and one result")
    if tuple(rows[0]) != HEADERS or len(rows[1]) != len(HEADERS):
        raise CharacterizationError("unsupported CACTI CSV columns")
    result = dict(zip(HEADERS, rows[1]))
    if result["Dynamic search energy (nJ)"] != "N/A":
        raise CharacterizationError(
            "CAM/search characterization is unsupported"
        )
    for name in HEADERS[19:]:
        if result[name] != "N/A":
            raise CharacterizationError(
                "tag-array characterization is unsupported"
            )
    return result


def effective_organization(row, config):
    # Both pinned versions use ceil() for 2D data-array rows and columns,
    # with NUMBER_STACKED_DIE_LAYERS=1. CSV Capacity merely echoes input.
    # Exact rational arithmetic detects padding without a fitted tolerance.
    ndwl = integer(row["Ndwl"], "Ndwl", positive=True)
    ndbl = integer(row["Ndbl"], "Ndbl", positive=True)
    finite(row["Nspd"], "Nspd", positive=True)
    nspd = Fraction(row["Nspd"])
    for value in (ndwl, ndbl, nspd.numerator, nspd.denominator):
        if value & (value - 1):
            raise CharacterizationError("unsupported subarray organization")
    rows = math.ceil(
        Fraction(
            config["capacity_bytes"],
            config["banks"] * config["word_bytes"] * ndbl,
        )
        / nspd
    )
    columns = math.ceil(Fraction(8 * config["word_bytes"], ndwl) * nspd)
    bits = rows * columns * ndwl * ndbl * config["banks"]
    if bits != 8 * config["capacity_bytes"]:
        raise CharacterizationError(
            f"effective physical capacity changed: requested "
            f"{8 * config['capacity_bytes']} bits, subarrays contain {bits}"
        )
    return {
        "rows_per_subarray": rows,
        "columns_per_subarray": columns,
        "subarrays_per_bank": ndwl * ndbl,
        "capacity_bits": bits,
        "Ndwl": ndwl,
        "Ndbl": ndbl,
        "Nspd": str(nspd),
    }


def import_record(
    csv_path,
    config_path,
    source_dir,
    expected,
    identity,
    command,
    binary,
    format_id,
):
    if format_id not in FORMATS:
        raise CharacterizationError("unsupported CACTI source version")
    selected = FORMATS[format_id]
    hashes = {
        name: digest(Path(source_dir) / name) for name in selected["hashes"]
    }
    if hashes != selected["hashes"]:
        raise CharacterizationError("CACTI source identity mismatch")
    if not isinstance(identity, str) or not identity.strip():
        raise CharacterizationError("nonempty implementation id required")
    if (
        not isinstance(command, list)
        or not command
        or any(not isinstance(x, str) or not x for x in command)
    ):
        raise CharacterizationError("exact producer command list required")
    config = read_config(Path(config_path).read_text())
    row = read_csv(Path(csv_path).read_text())
    geometry_keys = {
        "capacity_bytes",
        "word_bytes",
        "banks",
        "rw_ports",
        "read_ports",
        "write_ports",
        "mask_model",
    }
    if not isinstance(expected, dict) or set(expected) != geometry_keys:
        raise CharacterizationError("expected geometry fields do not match")
    if expected["mask_model"] not in {"read_modify_write", "unsupported"}:
        raise CharacterizationError("CACTI does not establish byte enables")
    for name in geometry_keys - {"mask_model"}:
        if type(expected[name]) is not int or expected[name] != config[name]:
            raise CharacterizationError(f"requested geometry mismatch: {name}")
    for name, header in {
        "capacity_bytes": "Capacity (bytes)",
        "banks": "Number of banks",
        "output_bits": "Output width (bits)",
        "associativity": "Associativity",
    }.items():
        if integer(row[header], header, positive=True) != config[name]:
            raise CharacterizationError(f"effective geometry mismatch: {name}")
    technology = finite(row["Tech node (nm)"], "technology", positive=True)
    if technology != config["technology_um"] * 1000:
        raise CharacterizationError("effective technology differs from input")
    organization = effective_organization(row, config)
    cycle = finite(row["Random cycle time (ns)"], "cycle time", positive=True)
    access = finite(row["Access time (ns)"], "access time", positive=True)
    banks = expected["banks"]
    leakage = finite(row["Standby leakage per bank(mW)"], "leakage")
    geometry = dict(expected)
    geometry.update(
        storage_physical="sram",
        storage_scope="whole_device",
        minimum_access_interval_s=cycle * 1e-9,
        minimum_access_latency_s=access * 1e-9,
    )
    source = {
        "url": selected["url"],
        "revision": selected["revision"],
        "path": str(csv_path),
        "row": "CSV data row 1",
        "sha256": digest(csv_path),
        "library": f"CACTI {format_id}; {config['cell_type']} cell, "
        f"{config['peripheral_type']} peripheral",
        "technology_nm": technology,
        "temperature_c": config["temperature_k"] - 273.15,
        "voltage_v": None,
        "datatype": f"{8 * expected['word_bytes']}-bit SRAM access",
        "implementation": "UCA SRAM; one memory with the stated banks/ports",
        "activity": "CACTI analytical per-access read/write energy",
        "conversion": "nJ to J x1e-9; mW per bank to W x1e-3*banks; "
        "whole-memory mm2 to um2 x1e6",
        "notes": json.dumps(
            {
                "format": format_id,
                "source_hashes": hashes,
                "source_file_hashes": {
                    str(path.relative_to(source_dir)): digest(path)
                    for path in sorted(Path(source_dir).rglob("*"))
                    if path.is_file() and path.suffix in {".cc", ".h", ".dat"}
                },
                "input_path": str(config_path),
                "input_sha256": digest(config_path),
                "binary_sha256": digest(binary),
                "command": command,
                "requested_geometry": expected,
                "effective_geometry": geometry,
                "effective_subarray_organization": organization,
                "raw_fields": row,
                "access_time_s": access * 1e-9,
                "leakage_scope": "per-bank nongate plus gate, scaled once",
                "area_scope": "whole memory, not scaled by banks",
                "ecc": False,
                "voltage_assumption": "CACTI technology defaults; not independently supplied",
                "partial_write_assumption": expected["mask_model"],
                "validation": "analytical CACTI estimate; not synthesis validation",
            },
            sort_keys=True,
        ),
    }
    record = {
        "key": "memory",
        "id": identity,
        "source": source,
        "area": {
            "value": finite(row["Area (mm2)"], "area"),
            "unit": "mm2",
            "status": "derived",
        },
        "leakage": {
            "value": leakage * banks,
            "unit": "mW",
            "status": "derived",
        },
        "read": {
            "value": finite(row["Dynamic read energy (nJ)"], "read energy"),
            "unit": "nJ",
            "status": "derived",
        },
        "write": {
            "value": finite(row["Dynamic write energy (nJ)"], "write energy"),
            "unit": "nJ",
            "status": "derived",
        },
    }
    document = {"version": 1, "records": [record], "geometry": geometry}
    normalize(document, str(csv_path))
    return document


def main():
    parser = argparse.ArgumentParser(
        description="Import an offline CACTI SRAM result."
    )
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--source-dir", required=True, type=Path)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--format", required=True, choices=FORMATS)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument(
        "--command",
        required=True,
        type=Path,
        help="JSON array containing the producer command",
    )
    parser.add_argument("--id", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        document = import_record(
            args.csv,
            args.input,
            args.source_dir,
            json.loads(args.geometry.read_text()),
            args.id,
            json.loads(args.command.read_text()),
            args.binary,
            args.format,
        )
        with args.output.open("x") as stream:
            json.dump(document, stream, indent=2)
            stream.write("\n")
    except (CharacterizationError, OSError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
