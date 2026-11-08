# AMX foundation instruction regressions

Run from the repository root after building `build/X86/gem5.opt`:

```sh
python3 tests/gem5/x86_amx/run.py --gem5 build/X86/gem5.opt --outdir /tmp/amx-regression
```

This standalone Linux host runner builds minimal x86 ELF guests in Python;
no guest compiler, AMX host CPU, downloaded binary, or root access is needed.
It runs Atomic, Timing, and O3 by default (`--cpus atomic` selects one).
Expected fault cases deliberately terminate gem5; core dumps are disabled.
The runner checks the exception class, not just a nonzero exit status.

Coverage: the four configuration/zero/release instructions, actual legacy
prefixes versus VEX.pp, reserved VEX fields, unused register extensions,
and noncanonical memory operands with DS and SS defaults. These cases
complement the state/configuration tests in `src/arch/x86/regs/tile.test.cc`.
Instruction outcomes were also checked on an AMX-enabled Xeon 6 host.

Reference: Intel SDM Vol. 2, AMX-E1/E2/E5/E6 exception tables and the
LDTILECFG, STTILECFG, TILEZERO, and TILERELEASE instruction entries:
https://cdrdv2-public.intel.com/835757/325383-sdm-vol-2abcd.pdf

This is an opt-in SE test, not full-system AMX/XSAVE/XFD validation.
