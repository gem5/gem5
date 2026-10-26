# Power, performance, and area

SALAM estimates functional-unit datapath and explicitly modeled storage.
It reports known quantities and coverage separately. A missing coefficient
is unavailable, not zero. No characterization is selected by the default
hardware profile, so a default run normally reports incomplete PPA while
continuing to execute the workload.

Controllers, DMA engines, interconnects, clock trees, caches, streams, and
external memory are outside these totals. CPU or DMA traffic can still
consume energy in a characterized SPM or RegisterBank. These estimates are
not a synthesized chip area or board power measurement.

## Quantities and intervals

| Quantity | Unit and calculation |
|---|---|
| Area | Square micrometres; each physical instance counted once. |
| Dynamic energy | Joules; admitted operations and serviced transfers times their event coefficients. |
| Leakage energy | Watts per physical instance times seconds in the declared interval. |
| Total energy | Known dynamic plus known leakage energy. |
| Average power | Known total joules divided by elapsed seconds. |

The accelerator invocation summary runs from accepted launch through
completion and survives a statistics reset during that invocation. Device
statistics cover the current statistics window, including timed preload
and result retrieval during that window. Dumping statistics does not add
energy or advance simulation. At zero duration, average power is unavailable;
finite activity energy and area remain reportable.

Compute passes and elapsed invocation ticks are different quantities.
Consecutive compute edges include both endpoints;
N scheduler passes need not mean N elapsed clock periods. PPA observation
does not introduce events, compensating delays, or changes to the clocks,
FU admission, memory service, or existing counters.

## Characterization records

The shared loader is `gem5.salam.power_model`; its source is
`util/SALAM-tools/SALAM-Configurator/power_model.py`. Files ending in `.json`
use JSON parsing, including exponent-only numeric notation such as `1e-8`.
Other record files use the safe YAML loader. Both reject duplicate mapping
keys, and the same normalization checks follow parsing. Parsing stays in
Python; simulation neither parses YAML in C++ nor launches CACTI.

A version-1 document contains `records`, plus `geometry` for a memory.
Each record has a unique `key`, a physical implementation `id`, and
`source`. The required source fields are `revision`, `path`, `row`,
`sha256`, `library`, `datatype`, and `implementation`. Preserve the original
conditions, missing evidence, and conversions in the optional provenance
fields. Synthetic test records are not production characterization.

| Key | Meaning |
|---|---|
| `fu:<alias>` | One physical FU implementation; `signatures` lists supported operand/result types. |
| `op:<alias>:<opcode>:<signature>` | Explicit operation support and optional event energy on the same implementation ID. |
| `register:i32`, `register:f64`, `register:p32`, etc. | One materialized scalar storage slot of that exact LLVM type. |
| `register:bit` | Explicit homogeneous per-bit storage characterization, used only when no exact slot record exists. |
| `memory` | One physical memory; geometry and physical scope must be explicit. |

A signature such as `i32,i32->i32` names the actual operand and result types.
Supported scalar types are `i1`, `i8`, `i16`, `i32`, `i64`, `f32`, and `f64`,
plus pointers with the IR `DataLayout` width up to 64 bits. Void is permitted
only where the operation requires it. Other widths are not extrapolated.
The loader rejects malformed LLVM operation/type combinations, duplicate
bindings, unsupported fields, negative or nonfinite values, booleans used
as numeric coefficients, and unknown units. Pointer/integer conversion uses
`ptrtoint` or `inttoptr`, not `bitcast`. A remapped operation retains its
actual opcode; remapping an `fsub` does not characterize it as sine.

Profile ownership uses SALAM instruction names, while `op:` keys use LLVM
operation names. The generator maps `and_inst`, `or_inst`, `xor_inst`, and
`gep` ownership entries to `and`, `or`, `xor`, and `getelementptr` when
checking characterization bindings. For example, a unit owning `and_inst`
can use `op:bitwise_operations:and:i32,i32->i32`. Its ownership entry remains
`and_inst`; a coefficient key containing `and_inst` is invalid. This mapping
does not accept arbitrary aliases, grant ownership to another FU, or relax
the operation/type and physical-signature checks.

An available `area`, `leakage`, `event`, `read`, or `write` quantity contains
`value`, `unit`, and `status`. Available status is `characterized`, `derived`,
or `proxy`. Derived quantities require a documented conversion; proxies
require an explicit reason. Omit a missing quantity or use
`{status: unavailable}` without a numeric value or unit. A genuine zero
uses a numeric zero with an available status.

Physical `fu:` records supply area and leakage; `op:` records bind actual
operations and may supply their event energy. An operation record must share
its physical FU's implementation ID and name a signature declared by that
FU. Every operation in the resolved reachable graph needs that explicit
binding before the FU can claim compatible static coverage. A datatype
signature alone is insufficient: an i32 multiply record does not establish
an implementation for i32 division. An explicit operation row with
unavailable event energy can retain known area/leakage while leaving dynamic
energy incomplete. Shared hardware area is still counted once.

Accepted area units are `um2` and `mm2`; power units are `W`, `mW`, and
`uW`; energy units are `J`, `nJ`, and `pJ`. Conversion occurs once on input.
No width, technology, voltage, frequency, or arbitrary instruction-latency
scaling is inferred for FUs.

Optional physical FU `constraints` contain all of:

```yaml
constraints:
    minimum_latency_s: 2.0e-9
    minimum_issue_interval_s: 1.0e-9
    pipelined: true
```

These example numbers only illustrate the fields. Use values supported by
your implementation's characterization. SALAM compares them with actual
instruction `CycleCounts`, the compute clock, and the effective FU issue
interval. Positive configured II denotes pipelined admission; zero II
retains a unit until completion. Every physical instance also admits at
most once per compute edge, so the effective interval is at least one
compute period even for zero-latency operations. This admission limit does
not extend their completion latency or occupancy. Pipeline organization
must match the explicit physical declaration, and the corresponding
latency and issue interval must meet its minima. Legacy FU `cycles` or
`stages` metadata does not substitute for the execution settings.

Unknown constraints are reported as unavailable timing compatibility,
separately from coefficient coverage. A known violation makes the estimate
incomplete without changing simulation. A source row named "10 ns" is not
proof of timing compatibility. Passing these declared lower bounds is not
independent synthesis validation, and a slower clock alone does not prove
a different pipeline organization compatible.

## Selecting records

Use the existing hardware profile mechanism. In a copied profile's FU YAML,
add an explicit reference under the existing `functional_unit.power_model`:

```yaml
characterization: relative/path/to/coefficients.yml
```

The path is relative to that FU YAML, not the shell's working directory.
FU and operation records must match the declaring alias and its implemented
operations. Referenced inputs are build dependencies. Generated profile data
embeds normalized records, so the simulator does not fall back to source
YAML at runtime. Keep `--with-salam` and the existing optional
`--salam-hw-config=/path/to/salam-hw-config.yml` selection.

For a workload SPM or RegisterBank variable, add:

```yaml
Characterization: coefficients/storage.yml
```

This path is relative to the declaring workload YAML. It selects an
offline record for the existing memory and does not change topology,
profile, latency, bandwidth, or clocks. Both launchers use the same loader.

## Fixed hardware and observed activity

FU area/leakage uses `HWInterface`'s inventory resolved on first invocation.
A never-launched accelerator has unavailable inventory rather than a proven
zero-area datapath. A positive-limit idle unit still exists; an unused
auto-sized unit with zero instances does not. Multiple operations sharing
one unit do not allocate multiple copies of its area. Incompatible widths
remain incomplete rather than selecting the largest convenient coefficient.
Unbound compute operations are identified explicitly; control bookkeeping
and memory operations are not invented as physical FUs.

Internal registers use a static SSA-storage approximation. SALAM counts
distinct materialized scalar backing slots in reachable functions, with
actual LLVM types and `DataLayout` pointer widths. Aliases sharing storage
are counted once. Constants, labels, void results, arrays, and bookkeeping
are excluded. Loops and repeated invocations do not allocate new hardware.
The count reflects the existing modeled function capacity; it does not
invent recursion or reentrant capacity. This is not an RTL register
allocator, and muxes, routing, and clock distribution are not included.

FU event energy is charged once after successful admission. Register reads
observe operand capture; writes observe result publication. Dependency
polls, tracing, and debug getters add no transfers. Device hooks observe
actual serviced timed accesses, including CPU and DMA requestors. Rejected
requests, response retransmission, and functional initialization/inspection
add no duplicate activity. RegisterBank's host-side deferred-buffer copy is
not a whole-bank physical write. Exposing a memory backdoor leaves activity
coverage incomplete because accesses through it can bypass these hooks.

## Physical memory assumptions

Memory geometry declares `capacity_bytes`, `word_bytes`, `banks`,
`read_ports`, `write_ports`, `rw_ports`, `mask_model`,
`minimum_access_latency_s`, `minimum_access_interval_s`, `storage_physical`,
and `storage_scope`. Cell-only records may explicitly declare both timing
minima `null`; the loader derives `memory_timing_available=false` and inert
numeric fields while retaining the original nulls in provenance. The
availability flag is normalized output, not an input geometry key. Omission
or a mixture of null and numeric minima is rejected. Whole-device records
require finite nonnegative minima. Configured capacity and bound ports must
match the record. Count each connected vector-port endpoint separately; a
vector with two peers is two modeled ports. SALAM does not silently resize
the memory or infer an organization. A selected physical word width is an
explicit characterization assumption, not a width inferred from one
observed request.

SRAM energy is per physical word touched, including partial, unaligned,
and split accesses. `byte_enable` needs explicit supporting evidence.
`read_modify_write` adds the required reads for partial writes;
`unsupported` leaves such accesses incomplete. All-disabled writes consume
no write event. Unknown word width and unsupported partial-write mappings
leave word counts unavailable; known mapped accesses retain their subtotals.
The model does not invent a one-word event for an unmapped transfer.

For SPMs, the minimum configured access latency must meet the characterized
access time. The bandwidth-derived service interval must also accommodate
physical cycle time. A partial write to a multi-byte word with
`read_modify_write` can require a read and a write; its conservative check
uses two physical cycles for the shortest possible serviced transfer.
These checks neither draw another latency sample nor slow the simulated
device. They are necessary timing bounds, not a characterization of the
unmodeled controller or proof that every request pattern is feasible.

RegisterBank uses explicit `flip_flop` storage and per-bit read/write
coefficients. SRAM records are not substitutes. Flip-flop read-modify-write
is unsupported. `storage_scope: storage_cells` retains known cell totals
but leaves selection, decoder, and port logic incomplete. Only an explicitly
characterized `whole_device` record can claim that complete modeled scope.
Clock distribution remains excluded in either case. Area and leakage refer
to the entire declared scope of that memory. Multiplying sourced per-bit
flip-flop area/leakage by capacity produces a `storage_cells` proxy, not a
whole-device characterization or read/write energy.

RegisterBank's private reads are immediate and its service model does not
impose a physical access interval. A record requiring a positive read
latency or interval therefore remains incompatible with that execution
model. Unavailable cell-only timing remains explicitly unavailable; it is
not replaced by a claimed zero-latency implementation.

The importer `util/SALAM-tools/cacti-SALAM/import_cacti.py` accepts pinned
CACTI 6.5 and HP CACTI 7.0 formats. It verifies the pinned `io.cc`, `uca.cc`,
`cacti_interface.cc`, `parameter.cc`, and `const.h` hashes and retains the
other source hashes. Supply the raw CSV, original input, producer command,
binary, source directory, and requested geometry JSON. These commands assume
the repository root and a Python interpreter with PyYAML available.

The importer accepts exactly these keys in its `geometry.json` input:

```json
{
    "capacity_bytes": 4096,
    "word_bytes": 8,
    "banks": 1,
    "read_ports": 0,
    "write_ports": 0,
    "rw_ports": 2,
    "mask_model": "read_modify_write"
}
```

This is an illustrative organization, not a default for every workload.
The importer derives the additional normalized fields, including
`storage_physical: sram`, `storage_scope: whole_device`, and both timing
minima, from the selected raw output. Do not add those fields to this input
JSON. `command.json` must contain the actual producer command as a JSON
array, for example `["./cacti", "-infile", "memory.cfg"]` only if that is
what produced the retained output. Keep its working directory and execution
environment with the raw evidence.

For already existing input files, and a new output filename:

```bash
python3 -B util/SALAM-tools/cacti-SALAM/import_cacti.py \
    --format hp-cacti-7.0 --csv memory.csv --input memory.cfg \
    --source-dir /path/to/pinned/cacti --binary /path/to/pinned/cacti/cacti \
    --geometry geometry.json --command command.json \
    --id my-memory --output memory-characterization.json
```

The importer does not run or build CACTI and refuses to overwrite an
existing output. Supported inputs are non-associative UCA SRAM with the
selected ITRS cell/periphery types, no ECC or power gating, and one full
block per access with matching output width. Tag/search arrays, single-ended
ports, and unproven byte-enable support are rejected. The importer accepts
only `read_modify_write` or `unsupported` for partial-write handling; it does
not infer byte enables from CACTI's whole-word energy.

It converts nJ/access to J/access, bank
leakage (including the selected version's gate-leakage semantics) from mW
to whole-memory W, and whole-memory mm² to µm². It preserves access latency
and random cycle time separately, requested/effective geometry, raw hashes,
and source identity. The CSV capacity field alone is insufficient: it
echoes the requested size. The importer also checks the selected subarray
organization for physical padding. Unsupported mappings, changed capacity,
and unknown output formats are rejected. The older offline helper's summary
CSV is not automatically equivalent to this raw format. A successful import
checks the selected model's provenance and consistency; it does not establish
that CACTI's analytical technology assumptions match fabricated hardware.

## Reports and strict checks

Native `SALAMPowerModel` statistics report known area, dynamic/leakage/total
energy, elapsed seconds, average power availability, independent coverage
flags, approximation status, timing compatibility, and transfer counts.
Detailed console reports are opt-in: set `report_details=True` on each
`SALAMPowerModel` whose invocation and statistics-window details are
needed. These reports identify scope, physical inventory, missing
coefficients/events, and SSA storage slots. Native statistics remain
available with the default `report_details=False`. The option changes
output only; it does not affect observation or strict checks.

`memoryWordCountsAvailable` distinguishes fully mapped word
counts from known partial subtotals. Unknown FU timing and known timing
violations are reported separately; a known violation takes precedence in
the combined compatibility status. Use actual object paths from
`config.json`. Missing-event diagnostics follow the same invocation or
statistics-window lifetime as the corresponding counters.

`SALAMPowerModel.strict=True` rejects incomplete modeled coefficient or
component coverage whether or not detailed output is requested. Unknown FU
timing remains a separate unavailable validation claim. Cell-only memory
always lacks selection, decoder, and
port logic, and can also lack timing evidence. Explicit timing
incompatibility fails strict coverage. Complete coefficient coverage alone
is not independent physical validation.

A memory's area and leakage coverage describe coefficients for its declared
geometry. A known memory timing violation makes dynamic coverage incomplete
and fails strict mode, while retaining those static known subtotals. The
explicit detailed report identifies the violated latency or service-rate
constraint in its `PPA missing` lines; retain that output when those
diagnostics are needed alongside an aggregate report. The report's
`fu_timing_status` is specific to functional units, and
`memory_timing_available` says whether memory timing evidence was supplied,
not whether the configured service rate is compatible. FU implementation
mismatches conservatively invalidate both static and dynamic coverage.

Handing out a direct memory backdoor makes dynamic coverage incomplete
because subsequent direct accesses bypass packet observation. No event
count is invented for that unobserved activity. Unsupported serviced atomic
accesses and unknown physical word widths are recorded as missing events;
known word-count subtotals remain separate from full count availability.

Setting `enabled=False` disables observation and detailed PPA output
while preserving execution. Apply it to every actual PPA child when
disabling the model for an entire system. Disabled observation remains
distinct from enabled observation with missing characterization.

Aggregate one statistics window by physical model identity:

```bash
python3 -B util/SALAM-tools/ppa_report.py \
    --stats m5out/stats.txt --config m5out/config.json \
    --window -1 --output ppa.json
```

Add `--strict` to require complete modeled component coverage. The report
sums each physical device once and rejects inconsistent observation
intervals. Shared memories are not duplicated per accelerator or assigned
arbitrary leakage shares. Keep invocation datapath estimates separate from
full-offload device-window totals when making comparisons.
