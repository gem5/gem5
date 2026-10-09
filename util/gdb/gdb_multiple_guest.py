#!/usr/bin/env python3
#
# Copyright (c) 2026 Arm Limited
# All rights reserved
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

"""
GDB commands for comparing two gem5 remote-GDB guests.

Source this file from GDB after connecting inferior 1 and inferior 2 to
separate gem5 remote-GDB instances:

    (gdb) source util/gdb/gdb_multiple_guest.py
"""

GDB_REG_GROUPS = ("general", "integer", "general-purpose")


def _inferior(num: int):
    for inferior in gdb.inferiors():
        if inferior.num == num:
            return inferior
    raise gdb.GdbError(f"Inferior {num} does not exist.")


def _connection_status(inferior) -> str:
    if inferior.connection is None:
        return "no target connection"

    threads = inferior.threads()
    if not threads:
        return "connected, no live threads"

    return f"connected, {len(threads)} live thread(s)"


def _select_inferior(num: int):
    inferior = _inferior(num)
    if inferior.connection is None:
        raise gdb.GdbError(
            f"Inferior {num} has no target connection. "
            "Connect it with 'target remote HOST:PORT' first."
        )

    gdb.execute(f"inferior {num}", to_string=True)

    threads = inferior.threads()
    if not threads:
        raise gdb.GdbError(
            f"Inferior {num} has no live threads. Check 'info inferiors' "
            "and 'info threads' to confirm the remote target is attached."
        )

    threads[0].switch()
    return inferior


def _parse_int(value: str) -> int:
    return int(gdb.parse_and_eval(value))


def _read_memory(inferior_num: int, addr: int, size: int) -> bytes:
    """
    Read size bytes of memory from the addr in the inferior's guest address
    space.
    """
    inferior = _inferior(inferior_num)
    return bytes(inferior.read_memory(addr, size))


def _gdb_registers() -> tuple:
    arch = gdb.selected_frame().architecture()

    for group in GDB_REG_GROUPS:
        try:
            regs = tuple(reg.name for reg in arch.registers(group))
        except gdb.error:
            continue

        if regs:
            return regs

    return tuple(reg.name for reg in arch.registers())


def _read_registers(inferior_num: int, regs) -> dict:
    _select_inferior(inferior_num)
    frame = gdb.selected_frame()
    return {reg: int(frame.read_register(reg)) for reg in regs}


def _step_once() -> None:
    """
    Execute a single step GDB command on both inferiors
    """
    _select_inferior(1)
    gdb.execute("stepi")
    _select_inferior(2)
    gdb.execute("stepi")


def _print_register_diff(first: dict, second: dict) -> bool:
    different = False
    for reg, first_value in first.items():
        second_value = second[reg]
        if first_value != second_value:
            different = True
            print(
                f"{reg}: inferior 1={first_value:#x}, "
                f"inferior 2={second_value:#x}"
            )

    return different


class DualStatus(gdb.Command):
    """
    Print connection and thread status for the two gem5 inferiors.

    Usage:
        dstatus
    """

    def __init__(self):
        super().__init__("dstatus", gdb.COMMAND_STATUS)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if argv:
            raise gdb.GdbError("Usage: dstatus")

        for num in (1, 2):
            inferior = _inferior(num)
            print(f"inferior {num}: {_connection_status(inferior)}")


class DualStep(gdb.Command):
    """
    Step two gem5 remote-GDB inferiors in lockstep.

    Usage:
        dstep [COUNT]
    """

    def __init__(self):
        super().__init__("dstep", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if len(argv) > 1:
            raise gdb.GdbError("Usage: dstep [COUNT]")

        count = 1 if not argv else _parse_int(argv[0])
        if count < 1:
            raise gdb.GdbError("COUNT must be positive.")

        for _ in range(count):
            _step_once()

        _select_inferior(1)


class DualStepRegs(gdb.Command):
    """
    Step both inferiors until guest GPRs differ.

    Usage:
        dstepregs [MAX_STEPS]

    If MAX_STEPS is omitted, this runs until a GPR difference is found or one
    of the targets stops responding.
    """

    def __init__(self):
        super().__init__("dstepregs", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if len(argv) > 1:
            raise gdb.GdbError("Usage: dstepregs [MAX_STEPS]")

        max_steps = None if not argv else _parse_int(argv[0])
        if max_steps is not None and max_steps < 1:
            raise gdb.GdbError("MAX_STEPS must be positive.")

        _select_inferior(1)
        regs = _gdb_registers()
        print("checking registers: " + ", ".join(regs))
        steps = 0
        while max_steps is None or steps < max_steps:
            _step_once()
            steps += 1

            first = _read_registers(1, regs)
            second = _read_registers(2, regs)
            if _print_register_diff(first, second):
                print(f"register difference after {steps} step(s)")
                _select_inferior(1)
                return

        print(f"no register difference after {steps} step(s)")
        _select_inferior(1)


class DualCompareMemory(gdb.Command):
    """
    Compare memory from two gem5 remote-GDB inferiors.

    Usage:
        dcmpmem ADDR SIZE
    """

    def __init__(self):
        super().__init__("dcmpmem", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        argv = gdb.string_to_argv(arg)
        if len(argv) != 2:
            raise gdb.GdbError("Usage: dcmpmem ADDR SIZE")

        addr = _parse_int(argv[0])
        size = _parse_int(argv[1])
        if size < 1:
            raise gdb.GdbError("SIZE must be positive.")

        first = _read_memory(1, addr, size)
        second = _read_memory(2, addr, size)

        if first == second:
            print(f"match: {size} bytes at {addr:#x}")
            return

        for offset, (first_byte, second_byte) in enumerate(zip(first, second)):
            if first_byte != second_byte:
                print(
                    f"diff at {addr + offset:#x}: "
                    f"inferior 1={first_byte:#04x}, "
                    f"inferior 2={second_byte:#04x}"
                )
                return

        common_size = min(len(first), len(second))
        print(f"memory sizes differ after {common_size} bytes")


class DualPrintMemory(gdb.Command):
    """
    Print memory from both gem5 remote-GDB inferiors.

    Usage:
        dprintmem GDB_X_COMMAND
    """

    def __init__(self):
        super().__init__("dprintmem", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        if not arg.strip():
            raise gdb.GdbError("Usage: dprintmem GDB_X_COMMAND")

        _select_inferior(1)
        print("inferior 1:")
        gdb.execute(f"x{arg}")

        _select_inferior(2)
        print("inferior 2:")
        gdb.execute(f"x{arg}")

        _select_inferior(1)


DualStatus()
DualStep()
DualStepRegs()
DualCompareMemory()
DualPrintMemory()
