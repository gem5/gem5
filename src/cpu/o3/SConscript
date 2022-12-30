# -*- mode:python -*-

# Copyright (c) 2006 The Regents of The University of Michigan
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

import sys

Import('*')

if not env['CONF']['USE_NULL_ISA']:
    SimObject('FUPool.py', sim_objects=['FUPool'])
    SimObject('FuncUnitConfig.py', sim_objects=[])
    SimObject('BaseO3CPU.py', sim_objects=['BaseO3CPU'], enums=[
        'SMTFetchPolicy', 'SMTQueuePolicy', 'CommitPolicy'])

    Source('commit.cc')
    Source('cpu.cc')
    Source('decode.cc')
    Source('dyn_inst.cc')
    Source('fetch.cc')
    Source('free_list.cc')
    Source('fu_pool.cc')
    Source('iew.cc')
    Source('inst_queue.cc')
    Source('lsq.cc')
    Source('lsq_unit.cc')
    Source('mem_dep_unit.cc')
    Source('regfile.cc')
    Source('rename.cc')
    Source('rename_map.cc')
    Source('rob.cc')
    Source('scoreboard.cc')
    Source('store_set.cc')
    Source('thread_context.cc')
    Source('thread_state.cc')

    DebugFlag('CommitRate')
    DebugFlag('IEW')
    DebugFlag('IQ')
    DebugFlag('LSQ')
    DebugFlag('LSQUnit')
    DebugFlag('MemDepUnit')
    DebugFlag('O3CPU')
    DebugFlag('ROB')
    DebugFlag('Rename')
    DebugFlag('Scoreboard')
    DebugFlag('StoreSet')
    DebugFlag('Writeback')

    CompoundFlag('O3CPUAll', [ 'Fetch', 'Decode', 'Rename', 'IEW', 'Commit',
        'IQ', 'ROB', 'FreeList', 'LSQ', 'LSQUnit', 'StoreSet', 'MemDepUnit',
        'DynInst', 'O3CPU', 'Activity', 'Scoreboard', 'Writeback' ])

    SimObject('BaseO3Checker.py', sim_objects=['BaseO3Checker'])
    Source('checker.cc')

    # For backwards compatibility
    SimObject('O3CPU.py', sim_objects=[])
    SimObject('O3Checker.py', sim_objects=[])
