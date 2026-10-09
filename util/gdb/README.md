# gem5 Utility Scripts

## Multiple-Guest GDB Comparison

`gdb_multiple_guest.py` adds GDB commands for driving two gem5
remote-GDB guests from one GDB session. It assumes inferior 1 and inferior 2
are connected to separate gem5 processes.

### Requirements

This tool requires a GDB build that supports the guest ISA.
We recommend `gdb-multiarch`, but other cross-isa debuggers
would also work

In gdb-multiarch You can check support with:

```gdb
set architecture aarch64
show architecture
```

### Getting-started

Start two gem5 runs with different remote-GDB ports. The configuration
script should have an option to set

```python
workload.wait_for_remote_gdb = True
```

```sh
build/ALL/gem5.opt --remote-gdb-port=7000 \
    <gem5-config-script>.py [...] ./guest_prog

build/ALL/gem5.opt --remote-gdb-port=7100 \
    <gem5-config-script>.py [...] ./guest_prog
```

Connect both targets from one GDB session:

```gdb
file ./guest_prog
target remote :7000
add-inferior -no-connection -exec ./prog

inferior 2
target remote :7100

inferior 1
source util/gdb/gdb_multiple_guest.py

info inferiors
info threads
```

`info inferiors` should show both inferior 1 and inferior 2 connected to
remote targets. `info threads` should show at least one thread for each
inferior before running `dstep`.

### Available commands:

```gdb
dstatus
```

Reports whether inferior 1 and inferior 2 have remote target connections and
live threads.

```gdb
dstep [COUNT]
```

Steps inferior 1 and inferior 2 by one guest instruction each. If `COUNT` is
provided, repeats the lockstep operation that many times.

If this reports that the program is not being run, confirm both remote targets
are still connected with `info inferiors`. The helper selects the first live
thread in each inferior before stepping, so that error normally means GDB does
not have a live thread for one of the two remote targets.

```gdb
dstepregs [MAX_STEPS]
```

Steps inferior 1 and inferior 2 one guest instruction at a time and compares
GDB's general register group after every lockstep step. It stops at the first
difference and prints each differing register. The register list comes from
`selected_frame().architecture().registers("general")`. If GDB does not expose
that group, the command tries `integer` and `general-purpose`, then falls back
to all registers reported by `Architecture.registers()`. The command prints
the register names once before it starts stepping.

If `MAX_STEPS` is omitted, the command runs until a register difference is
found or one of the targets stops responding. Supplying `MAX_STEPS` is useful
as a progress guard while debugging.

Examples:

```gdb
dstepregs
dstepregs 1000
```

```gdb
dcmpmem ADDR SIZE
```

Reads `SIZE` bytes at `ADDR` from both inferiors and reports the first byte
difference. `ADDR` and `SIZE` are GDB expressions.

```gdb
dprintmem GDB_X_COMMAND
```

Runs the same GDB memory examine command against both inferiors.

Example:

```gdb
dstep 10
dstepregs 1000
dcmpmem 0x80000000 0x100
dprintmem /16gx 0x80000000
```
