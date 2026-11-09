# AMX foundation instruction regressions

The three quick TestLib suites run the same instruction probes on Atomic,
Timing, and O3. They use the X86 opt build fixture and the x86_64 host tag.
No guest compiler, AMX host CPU, downloaded binary, or root access is needed.

From `tests/`, list and run the suites (omit `--skip-build` to build gem5):

```sh
./main.py list gem5/x86_amx --suites -q
./main.py run gem5/x86_amx --skip-build
```

`ci-tests.yaml` discovers quick-test directories and their build fixtures
through TestLib, so these suites are included without a workflow change.
For a standalone run from the repository root:

```sh
python3 tests/gem5/x86_amx/run.py --gem5 build/X86/gem5.opt --outdir /tmp/amx-regression
```

`--cpus atomic` selects one CPU model. The runner generates minimal x86 ELF
guests in Python. Successful cases compare all 64 bytes of a separate
sentinel-filled store destination and its guard bytes before printing the
success marker. They check distinct configurations before/after LDTILECFG,
all-zero configuration after TILERELEASE (or palette-zero LDTILECFG), and
TILEZERO resetting a nonzero `start_row` for each tile destination. TILEZERO's
tile-data contents are covered by the state-helper GTest: this foundation
patch does not yet implement TILESTORED for guest-side data inspection.

Negative cases check the specific exception, not just a nonzero exit status.
Coverage includes actual legacy prefixes versus VEX.pp, reserved VEX fields,
unused register extensions, DS/SS addressing, and noncanonical addresses.
Expected fault cases deliberately terminate gem5; core dumps are disabled.
Detailed results and logs are retained by TestLib's temporary-output fixture.

Reference: Intel SDM Vol. 2, AMX-E1/E2/E5/E6 exception tables and the
LDTILECFG, STTILECFG, TILEZERO, and TILERELEASE instruction entries:
https://cdrdv2-public.intel.com/835757/325383-sdm-vol-2abcd.pdf

This is opt-in SE coverage, not full-system AMX/XSAVE/XFD validation.
