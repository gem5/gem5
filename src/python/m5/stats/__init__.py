# Copyright (c) 2007 The Regents of The University of Michigan
# Copyright (c) 2010 The Hewlett-Packard Development Company
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

import m5

from m5 import internal
from m5.internal.stats import schedStatEvent as schedEvent
from m5.objects import Root
from m5.util import attrdict, fatal

outputList = []
def initText(filename, desc=True):
    output = internal.stats.initText(filename, desc)
    outputList.append(output)

def initSimStats():
    internal.stats.initSimStats()
    internal.stats.registerPythonStatsHandlers()

names = []
stats_dict = {}
stats_list = []
raw_stats_list = []
def enable():
    '''Enable the statistics package.  Before the statistics package is
    enabled, all statistics must be created and initialized and once
    the package is enabled, no more statistics can be created.'''
    __dynamic_cast = []
    for k, v in internal.stats.__dict__.iteritems():
        if k.startswith('dynamic_'):
            __dynamic_cast.append(v)

    for stat in internal.stats.statsList():
        for cast in __dynamic_cast:
            val = cast(stat)
            if val is not None:
                stats_list.append(val)
                raw_stats_list.append(val)
                break
        else:
            fatal("unknown stat type %s", stat)

    for stat in stats_list:
        if not stat.check() or not stat.baseCheck():
            fatal("statistic '%s' (%d) was not properly initialized " \
                  "by a regStats() function\n", stat.name, stat.id)

        if not (stat.flags & flags.display):
            stat.name = "__Stat%06d" % stat.id

    def less(stat1, stat2):
        v1 = stat1.name.split('.')
        v2 = stat2.name.split('.')
        return v1 < v2

    stats_list.sort(less)
    for stat in stats_list:
        stats_dict[stat.name] = stat
        stat.enable()

    internal.stats.enable();

def prepare():
    '''Prepare all stats for data access.  This must be done before
    dumping and serialization.'''

    for stat in stats_list:
        stat.prepare()

lastDump = 0
def dump():
    '''Dump all statistics data to the registered outputs'''

    curTick = m5.curTick()

    global lastDump
    assert lastDump <= curTick
    if lastDump == curTick:
        return
    lastDump = curTick

    internal.stats.processDumpQueue()

    prepare()

    for output in outputList:
        if output.valid():
            output.begin()
            for stat in stats_list:
                output.visit(stat)
            output.end()

def reset():
    '''Reset all statistics to the base state'''

    # call reset stats on all SimObjects
    root = Root.getInstance()
    if root:
        for obj in root.descendants(): obj.resetStats()

    # call any other registered stats reset callbacks
    for stat in stats_list:
        stat.reset()

    internal.stats.processResetQueue()

flags = attrdict({
    'none'    : 0x0000,
    'init'    : 0x0001,
    'display' : 0x0002,
    'total'   : 0x0010,
    'pdf'     : 0x0020,
    'cdf'     : 0x0040,
    'dist'    : 0x0080,
    'nozero'  : 0x0100,
    'nonan'   : 0x0200,
})
