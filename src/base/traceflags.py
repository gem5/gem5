#!/usr/bin/env python

# Copyright (c) 2004-2005 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert
#          Steve Reinhardt

__all__ = [ 'allFlags', 'baseFlags', 'compoundFlagsMap', 'compoundFlags' ]

#
# The list of trace flags that can be used to condition DPRINTFs etc.
# To define a new flag, simply add it to this list.
#
baseFlags = [
    'Activity',
    'AlphaConsole',
    'Annotate',
    'BADADDR',
    'BE',
    'BPredRAS',
    'Bus',
    'BusAddrRanges',
    'BusBridge',
    'Cache',
    'CachePort',
    'Chains',
    'Checker',
    'Clock',
    'Commit',
    'CommitRate',
    'Config',
    'Console',
    'ConsolePoll',
    'ConsoleVerbose',
    'Context',
    'Cycle',
    'DMA',
    'DMAReadVerbose',
    'DMAWriteVerbose',
    'DebugPrintf',
    'Decode',
    'DiskImage',
    'DiskImageRead',
    'DiskImageWrite',
    'DynInst',
    'Ethernet',
    'EthernetCksum',
    'EthernetDMA',
    'EthernetData',
    'EthernetDesc',
    'EthernetEEPROM',
    'EthernetIntr',
    'EthernetPIO',
    'EthernetSM',
    'Event',
    'ExecEnable',
    'ExecCPSeq',
    'ExecEffAddr',
    'ExecFetchSeq',
    'ExecIntRegs',
    'ExecIntel',
    'ExecLegion',
    'ExecOpClass',
    'ExecRegDelta',
    'ExecResult',
    'ExecSpeculative',
    'ExecSymbol',
    'ExecThread',
    'ExecTicks',
    'FE',
    'Fault',
    'Fetch',
    'Flow',
    'FreeList',
    'FullCPU',
    'FunctionalAccess',
    'GDBAcc',
    'GDBExtra',
    'GDBMisc',
    'GDBRead',
    'GDBRecv',
    'GDBSend',
    'GDBWrite',
    'HWPrefetch',
    'IBE',
    'IEW',
    'IIC',
    'IICMore',
    'IPI',
    'IPR',
    'IQ',
    'ISP',
    'IdeCtrl',
    'IdeDisk',
    'Interrupt',
    'LLSC',
    'LSQ',
    'LSQUnit',
    'Loader',
    'MC146818',
    'MMU',
    'MSHR',
    'Mbox',
    'MemDepUnit',
    'MemoryAccess',
    'O3CPU',
    'OzoneCPU',
    'OzoneLSQ',
    'PCEvent',
    'PCIA',
    'PCIDEV',
    'PciConfigAll',
    'Pipeline',
    'Printf',
    'Quiesce',
    'ROB',
    'Regs',
    'Rename',
    'RenameMap',
    'SQL',
    'Sampler',
    'Scoreboard',
    'ScsiCtrl',
    'ScsiDisk',
    'ScsiNone',
    'Serialize',
    'SimpleCPU',
    'SimpleDisk',
    'SimpleDiskData',
    'Sparc',
    'Split',
    'Stack',
    'StatEvents',
    'Stats',
    'StoreSet',
    'Syscall',
    'SyscallVerbose',
    'TCPIP',
    'TLB',
    'Thread',
    'Timer',
    'Tsunami',
    'Uart',
    'VtoPhys',
    'WriteBarrier',
    'Writeback',
    ]

#
# "Compound" flags correspond to a set of base flags.  These exist
# solely for convenience in setting them via the command line: if a
# compound flag is specified, all of the corresponding base flags are
# set.  Compound flags cannot be used directly in DPRINTFs etc.
# To define a new compound flag, add a new entry to this hash
# following the existing examples.
#
compoundFlagMap = {
    'All'            : baseFlags,
    'DiskImageAll'   : [ 'DiskImage', 'DiskImageRead', 'DiskImageWrite' ],
    'EthernetAll'    : [ 'Ethernet', 'EthernetPIO', 'EthernetDMA',
                         'EthernetData' , 'EthernetDesc', 'EthernetIntr',
                         'EthernetSM', 'EthernetCksum' ],
    'EthernetNoData' : [ 'Ethernet', 'EthernetPIO', 'EthernetDesc',
                         'EthernetIntr', 'EthernetSM', 'EthernetCksum' ],
    'Exec'           : [ 'ExecEnable', 'ExecTicks', 'ExecOpClass',
                         'ExecThread', 'ExecEffAddr', 'ExecResult',
                         'ExecSymbol' ],
    'GDBAll'         : [ 'GDBMisc', 'GDBAcc', 'GDBRead', 'GDBWrite', 'GDBSend',
                         'GDBRecv', 'GDBExtra' ],
    'IdeAll'         : [ 'IdeCtrl', 'IdeDisk' ],
    'O3CPUAll'       : [ 'Fetch', 'Decode', 'Rename', 'IEW', 'Commit', 'IQ',
                         'ROB', 'FreeList', 'RenameMap', 'LSQ', 'LSQUnit',
                         'StoreSet', 'MemDepUnit', 'DynInst', 'FullCPU',
                         'O3CPU', 'Activity','Scoreboard','Writeback' ],
    'OzoneCPUAll'    : [ 'BE', 'FE', 'IBE', 'OzoneLSQ', 'OzoneCPU' ],
    'ScsiAll'        : [ 'ScsiDisk', 'ScsiCtrl', 'ScsiNone' ]
}

# extract just the compound flag names into a list
compoundFlags = []
compoundFlags.extend(compoundFlagMap.keys())
compoundFlags.sort()

allFlags = frozenset(baseFlags + compoundFlags)

#############################################################
#
# Everything below this point generates the appropriate C++
# declarations and definitions for the trace flags.  If you are simply
# adding or modifying flag definitions, you should not have to change
# anything below.
#
def gen_hh(filename):
    #
    # First generate the header file.  This defines the Flag enum
    # and some extern declarations for the .cc file.
    #
    try:
        hhfile = file(filename, 'w')
    except IOError, e:
        sys.exit("can't open %s: %s" % (hhfilename, e))

    # file header boilerplate
    print >>hhfile, '''
    /*
     * DO NOT EDIT THIS FILE!
     *
     * Automatically generated from traceflags.py
     */

    #ifndef __BASE_TRACE_FLAGS_HH__
    #define __BASE_TRACE_FLAGS_HH__

    namespace Trace {

    enum Flags {
    ''',

    # Generate the enum.  Base flags come first, then compound flags.
    idx = 0
    for flag in baseFlags:
        print >>hhfile, '    %s = %d,' % (flag, idx)
        idx += 1

    numBaseFlags = idx
    print >>hhfile, '    NumFlags = %d,' % idx

    # put a comment in here to separate base from compound flags
    print >>hhfile, '''
        // The remaining enum values are *not* valid indices for Trace::flags.
        // They are "compound" flags, which correspond to sets of base
        // flags, and are used only by TraceParamContext::setFlags().
    ''',

    for flag in compoundFlags:
        print >>hhfile, '    %s = %d,' % (flag, idx)
        idx += 1

    numCompoundFlags = idx - numBaseFlags
    print >>hhfile, '    NumCompoundFlags = %d' % numCompoundFlags

    # trailer boilerplate
    print >>hhfile, '''\
    }; // enum Flags

    // Array of strings for SimpleEnumParam
    extern const char *flagStrings[];
    extern const int numFlagStrings;

    // Array of arraay pointers: for each compound flag, gives the list of
    // base flags to set.  Inidividual flag arrays are terminated by -1.
    extern const Flags *compoundFlags[];

    /* namespace Trace */ }

    #endif // __BASE_TRACE_FLAGS_HH__
    ''',

    hhfile.close()

def gen_cc(filename):
    '''Print out .cc file with array definitions.'''

    try:
        ccfile = file(filename, 'w')
    except OSError, e:
        sys.exit("can't open %s: %s" % (ccfilename, e))

    # file header
    print >>ccfile, '''
/*
 * DO NOT EDIT THIS FILE!
 *
 * Automatically generated from traceflags.pl.
 */

#include "base/traceflags.hh"

using namespace Trace;

const char *Trace::flagStrings[] =
{
''',

    # The string array is used by SimpleEnumParam to map the strings
    # provided by the user to enum values.
    for flag in baseFlags:
        print >>ccfile, '    "%s",' % flag

    for flag in compoundFlags:
        print >>ccfile, '    "%s",' % flag

    print >>ccfile, '};\n'

    numFlagStrings = len(baseFlags) + len(compoundFlags);

    print >>ccfile, 'const int Trace::numFlagStrings = %d;' % numFlagStrings
    print >>ccfile

    #
    # Now define the individual compound flag arrays.  There is an array
    # for each compound flag listing the component base flags.
    #

    for flag in compoundFlags:
        flags = compoundFlagMap[flag]
        flags.append('(Flags)-1')
        print >>ccfile, 'static const Flags %sMap[] =' % flag
        print >>ccfile, '{ %s };' % (', '.join(flags))
        print >>ccfile

    #
    # Finally the compoundFlags[] array maps the compound flags
    # to their individual arrays/
    #
    print >>ccfile, 'const Flags *Trace::compoundFlags[] ='
    print >>ccfile, '{'

    for flag in compoundFlags:
        print >>ccfile, '    %sMap,' % flag

    # file trailer
    print >>ccfile, '};'

    ccfile.close()

if __name__ == '__main__':
    # This file generates the header and source files for the flags
    # that control the tracing facility.

    import sys

    if len(sys.argv) != 2:
        print "%s: Need argument (basename of cc/hh files)" % sys.argv[0]
        sys.exit(1)

    gen_hh(sys.argv[1] + '.hh')
    gen_cc(sys.argv[1] + '.cc')
