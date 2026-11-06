# Bounded gem5 coverage pilot

A local Linux pilot on 25 September 2026 exercised the collector against a
real instrumented `NULL/gem5.opt` binary. This checks collection and artifact
handling; it does **not** establish capacity or correctness for the full
scheduled campaign, its eight build targets, integration jobs, or Codecov's
hosted ingestion.

## Environment and provenance

The environment used GCC/gcov `13.3.0-6ubuntu2~24.04.1`, Python 3.12.3 and
coverage.py 7.10.7 on Linux aarch64. The local image was
`gem5-devcontainer-review:single`, with image configuration digest
`sha256:082941dad0260262e40888fbc9ef040c29ec5d29b17ab7f55d105216fd0f49ff`.
This identifies a local image, not a downloadable registry manifest.

gem5 was built from a snapshot of this PR, initialized as a temporary Git
repository at `b40efcf3dd1fa36e5bba435a541f7bd6722ef533`. That is a synthetic
local revision, not an upstream gem5 commit. The tiny configuration scripts
were added only for this experiment. Collector and reporting improvements
were copied into the environment as the pilot exposed problems; the final
consumer checks used the retained original records as well as a newer compact
record. The binary was not rebuilt for these Python helper changes.

The host architecture differs from the intended production x86 runners.
Although this particular NULL build ran successfully, this is not a general
claim of ARM-host gcov support. The validation container limited memory and
combined memory/swap to 7 GiB; the compilation ran separately.

## Workload and checks

The build commands were:

```sh
scons defconfig build/NULL build_opts/NULL --ignore-style
scons setconfig build/NULL USE_TEST_OBJECTS=y --ignore-style
scons --ignore-style --gcov build/NULL/gem5.opt -j4 CXX=g++ CC=gcc
```

The configuration instantiated an empty `Root(full_system=False)` and
simulated either three or seven ticks. It asserted `__m5_main__` and the
original argument list, including an argument containing a space. A third
mode deliberately raised an exception after simulation.

Two successful runs executed serially, the same pair ran concurrently, and
one run deliberately failed. All five produced complete native and Python
records; the failure remained marked failed. Python line and branch results
matched between serial and concurrent execution. Native differences were
limited to two wall-clock arithmetic lines in `src/base/time.hh`; identical
native hit sets are not expected for time-dependent code.

A sixth run used gem5's `-m` module entry point and the compact collector.
It preserved arguments and `__m5_main__`, and retained the same native build
identity. Separate tests against gem5's actual option declarations covered
attached/grouped module options, option values, failure propagation, and
fork ownership. All 22 collector/module regression tests passed with GNU GCC.

The reporting experiment supplied an explicit discovery manifest and JUnit
records derived from these known outcomes. Those records were synthetic;
this was a direct collector pilot, not an end-to-end TestLib harness run.
Queries checked both directions for selected Python lines and branches,
native measured-zero versus unknown locations, and combination of original
and compact records without doubling the executable denominator.

Native extraction recovery regenerated all five original native profiles
without rerunning gem5. All ten normalized native/Python records matched the
originals exactly, including the intentional failure.

## Observed costs

These are individual local measurements, not averages or production sizing
estimates. MiB/GiB use powers of two. Extraction and packaging times include
coverage work, separate from the simulated process time.

| Operation | Observed time | Size or peak memory |
| --- | --- | --- |
| Original cold native baseline | 66.1 s | 662.1 MiB JSON |
| Warm baseline preparation | 0.068 s | Shared within the collector |
| Serial simulation | 0.73–0.84 s | Tiny NULL configuration |
| Serial extraction | 46.4–49.3 s | Native and Python records |
| Concurrent extraction | 69.4–70.9 s per invocation | Two concurrent runs |
| Compact module baseline | 62.0 s | 344.6 MiB JSON; 48% smaller |
| Compact module simulation / extraction | 0.86 s / 44.3 s | Same native build identity |
| Shared build packaging | 21.6 s | 892.0 MiB compressed; 3,468 files |
| One compact invocation's split packaging | 147.6 s | 112.2 MiB report inputs; 188.1 MiB raw inputs |
| Compressed grouped report, original five invocations | 40.5 s | 2.80 GiB peak RSS; 96.9 MiB native LCOV |
| Retained source browser, original five invocations | 5.17 s | 75.7 MiB peak RSS; 2,544 pages |
| Bounded index, original plus compact invocations | 50.9 s | 4.23 GiB peak RSS; 172.4 MiB total artifact |
| Recovery, five original native invocations | 585.3 s | 5.18 GiB peak RSS |

The final mixed index held 12 native/Python records across four synthetic
suites. It retained 114,360 measured native lines and 2,561,729 compiled
branches. The initial HTML was 14.8 MiB and the core JSON was 15.6 MiB; 2,349
compressed sidecars retained branch data for on-demand queries.

An earlier implementation was killed by the 7 GiB memory limit while
indexing only the five original invocations. The resulting changes share
baseline data, omit repeated branch descriptions, process native build
graphs separately, and retain compressed branch and LCOV data. An automated
multi-build fixture additionally checks that each graph is released before
the next is loaded; the real pilot contains only one native build graph.

## Repeating and extending the check

Use a fresh checkout and a matching GCC/gcov pair. After the build above,
install the pinned tracer in the Python environment used by gem5. A minimal
configuration is:

```python
import sys
import m5
from m5.objects import Root

assert __name__ == "__m5_main__"
root = Root(full_system=False)
m5.instantiate()
m5.simulate(3 if sys.argv[1] == "a" else 7)
if sys.argv[1] == "fail":
    raise RuntimeError("intentional coverage pilot failure")
```

Load `ext/testlib/coverage.py` with `importlib.util.spec_from_file_location`.
Construct one `CoverageBuild` with the checkout, `build/NULL`, binary,
result directory, and `python_coverage=True`. For each execution, create
`build.invocation(suite_uid, log)`, enter its context to obtain the process
environment, wrap the original command with `invocation.python_command`,
and run the process with `check=True`. Catch the intentional process failure
outside the context so the collector retains its failed outcome. Reuse the
same build object for the concurrent pair. See the collector regression
tests for executable examples of these interfaces.

Use `report.py package`, `report.py summarize`, `report.py landing`, and
`index.py build` to exercise retained artifacts. For report accounting,
provide an expected-suite manifest and actual harness results, or explicitly
label manually supplied outcomes as synthetic. Follow [RECOVERY.md](RECOVERY.md)
for extraction retries; compare normalized records, not regenerated filenames
or compressed archive bytes.

Before enabling the scheduled campaign, measure the largest production build
graph, aggregate report sizes across all directory/length groups, disk use,
runner throughput, and artifact upload/download time. Per-invocation gcov
extraction remains expensive even for a subsecond simulation. The complete
campaign and an actual Codecov upload still need a controlled production run.
