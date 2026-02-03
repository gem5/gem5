# Copyright (c) 2024 The Regents of The University of California
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


"""
This script can be used within gdb to limit breaking at code locations
to specific Named objects, `Named` is the name of class in gem5
which class `SimObject` inherits from. To use this script, you need to source
it within gdb. You can do this by running the following command within gdb:
    source gem5/util/gdb_funcs.py
This is assuming that you've run your gdb from the parent directory of gem5.

Example usage:

gdb --args gem5/build/ARM/gem5.opt gem5/configs/example/gem5_library/arm-ubuntu-run.py

(gdb) source gem5/util/gdb_funcs.py
(gdb) python test = TargetNamedBreakpoint("gem5::ArmISA::MMU::translateTiming")
(gdb) python test.add_match_exact("board.processor.cores0.core.mmu")
(gdb) run

Global frequency set at 1000000000000 ticks per second
warn: No dot file generated. Please install pydot to generate the dot file and pdf.
src/mem/dram_interface.cc:690: warn: DRAM device capacity (16384 Mbytes) does not match the address range assigned (1024 Mbytes)
src/mem/dram_interface.cc:690: warn: DRAM device capacity (16384 Mbytes) does not match the address range assigned (1024 Mbytes)
src/sim/kernel_workload.cc:46: info: kernel located at: /home/msamani/.cache/gem5/arm64-linux-kernel-5.4.49
src/base/loader/symtab.cc:95: warn: Cannot insert a new symbol table due to name collisions. Adding prefix to each symbol's name can resolve this issue.
src/base/statistics.hh:279: warn: One of the stats is a legacy stat. Legacy stat is a stat that does not belong to any statistics::Group. Legacy stat is deprecated.
board.vncserver: Listening for connections on port 5900
board.terminal: Listening for connections on port 3456
board.realview.uart1.device: Listening for connections on port 3457
board.realview.uart2.device: Listening for connections on port 3458
board.realview.uart3.device: Listening for connections on port 3459
board.remote_gdb: Listening for connections on port 7000
src/arch/arm/fs_workload.cc:121: info: Using bootloader at address 0x10
src/arch/arm/fs_workload.cc:139: info: Using kernel entry physical address at 0x80080000
src/arch/arm/linux/fs_workload.cc:111: info: Loading DTB file: m5out/device.dtb at address 0x88000000
src/dev/arm/energy_ctrl.cc:252: warn: Existing EnergyCtrl, but no enabled DVFSHandler found.
src/sim/simulate.cc:199: info: Entering event queue @ 0.  Starting simulation...
Caught exception: value has been optimized out at location gem5::ArmISA::MMU::translateTiming. Continuing without stopping.

Breakpoint 1.3, gem5::ArmISA::MMU::translateTiming (this=0x55555eec9200,
    req=std::shared_ptr<gem5::Request> (use count 1, weak count 0) = {...}, tc=0x55555eec9780,
    translation=0x55555facf4b0, mode=gem5::BaseMMU::Execute) at src/arch/arm/mmu.hh:251
251         translateTiming(const RequestPtr &req, ThreadContext *tc,
(gdb) print _name
$1 = "board.processor.cores0.core.mmu"
"""

import re


# NOTE: no need to import anything since gdb has its own Python environment.
# Additionally, this script only works within gdb.
class TargetNamedBreakpoint(gdb.Breakpoint):
    def __init__(
        self, location: str, expose_gdb_exceptions: bool = False
    ) -> None:
        """
        This class allows the user to create Breakpoint objects that only stop
        when their code location is reached for certain Named objects.
        The list of qualified Named objects could be expressed either as exact
        matches or regex matches. Exact matches could be added to the breakpoint
        through `add_match_exact` method and regex matches could be added
        `add_match_regex` method.

        :param location: The location where the breakpoint should be set. E.g.
            "gem5::ruby::Cache_Controller::AllocateTBE_SeqRequest"
        :param expose_gdb_exceptions: To determine whether gdb needs to stop upon
        hitting an object of this class, this class overrides the `stop` method.
        This breakpoint will search for _name within the context of the breakpoint.
        It should succeed assuming the breakpoint is set within the scope of a
        Named class, which SimObject and its children are.
        If the breakpoint fails at extracting _name from the context,
        it will raise exceptions.
        This class treats gdb related exceptions separately from other exceptions.
        The developer suspects all the gdb exceptions to be related to the
        extraction of _name from the context.
        If this parameter is set to True, the class will raise gdb exceptions so
        that the user can course correct.
        If this parameter is set to False, the class will print the error message
        and continue without stopping.
        """
        super().__init__(location)
        self._expose_gdb_exceptions = expose_gdb_exceptions
        self._exact_matches = set()
        self._regex_matches = set()

    def add_match_exact(self, exact_match: str) -> None:
        """
        Add an exact match to the list of potential names.

        :param exact_match: The name to exact_match against upon hitting the breakpoint.
        E.g. "system.cache_hierarchy.core_clusters0.dcache" if you want to stop
        only for dcache for core_clusters0.
        """
        self._exact_matches.add(exact_match)

    def add_match_regex(self, regex_match: str) -> None:
        """
        Add a regex match to the list of potential names.

        :param regex_match: The pattern to regex_match against upon hitting the breakpoint.
        E.g. "system.cache_hierarchy.core_clusters[0-9]+.dcache" if you want to stop
        at all dcache objects for all core_clusters.
        """
        self._regex_matches.add(re.compile(regex_match))

    def _matches_exact(self, name: str) -> None:
        """
        Check if name matches any of the exact matches.
        """
        return name in self._exact_matches

    def _matches_regex(self, name: str) -> None:
        """
        Check if name matches any of the regex patterns
        """
        return any(regex.match(name) for regex in self._regex_matches)

    def _get_name_from_context(self) -> None:
        """
        This function extracts the _name from the context of the breakpoint.
        It also prepares the extracted value to match what the user would
        expect to see in python without any further manipulation.
        """
        name = gdb.parse_and_eval("_name")
        return str(name).strip('"')

    def stop(self):
        try:
            name = self._get_name_from_context()
        except gdb.error as gdb_exception:
            if self._expose_gdb_exceptions:
                raise gdb_exception
            print(
                f"Caught exception: {gdb_exception} at location "
                f"{self.location}. Continuing without stopping."
            )
            return False
        except Exception as exception:
            raise exception

        return self._matches_exact(name) or self._matches_regex(name)
