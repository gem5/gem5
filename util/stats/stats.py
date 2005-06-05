#!/usr/bin/env python

# Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#Permission is granted to use, copy, create derivative works and
#redistribute this software and such derivative works for any purpose,
#so long as the copyright notice above, this grant of permission, and
#the disclaimer below appear in all copies made; and so long as the
#name of The University of Michigan is not used in any advertising or
#publicity pertaining to the use or distribution of this software
#without specific, written prior authorization.
#
#THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
#UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND WITHOUT
#WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER EXPRESS OR
#IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF
#MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE REGENTS OF
#THE UNIVERSITY OF MICHIGAN SHALL NOT BE LIABLE FOR ANY DAMAGES,
#INCLUDING DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
#DAMAGES, WITH RESPECT TO ANY CLAIM ARISING OUT OF OR IN CONNECTION
#WITH THE USE OF THE SOFTWARE, EVEN IF IT HAS BEEN OR IS HEREAFTER
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

from __future__ import division
import re, sys, math


def usage():
    print '''\
Usage: %s [-E] [-F] [-d <db> ] [-g <get> ] [-h <host>] [-p]
       [-s <system>] [-r <runs> ] [-T <samples>] [-u <username>]
       <command> [command args]

       commands    extra parameters   description
       ----------- ------------------ ---------------------------------------
       bins        [regex]            List bins (only matching regex)
       formula     <formula>          Evaluated formula specified
       formulas    [regex]            List formulas (only matching regex)
       runs        none               List all runs in database
       samples     none               List samples present in database
       stability   <pairnum> <stats>  Calculated statistical info about stats
       stat        <regex>            Show stat data (only matching regex)
       stats       [regex]            List all stats (only matching regex)

       database    <command>          Where command is drop, init, or clean

''' % sys.argv[0]
    sys.exit(1)

def getopts(list, flags):
    import getopt
    try:
        opts, args = getopt.getopt(list, flags)
    except getopt.GetoptError:
        usage()

    return opts, args

def printval(name, value, invert = False):
    if invert and value != 0.0:
        value = 1 / value

    if value == (1e300*1e300):
        return

    if printval.mode == 'G':
        print '%s:    %g' % (name, value)
    elif printval.mode != 'F' and value > 1e6:
        print '%s:    %0.5e' % (name, value)
    else:
        print '%s:    %f' % (name, value)

printval.mode = 'G'

def unique(list):
    set = {}
    map(set.__setitem__, list, [])
    return set.keys()

#benchmarks = [ 'm', 's', 'snt', 'nb1', 'w1', 'w2', 'w3', 'w4', 'nm', 'ns', 'nw1', 'nw2', 'nw3' ]

def graphdata(runs, options, tag, label, value):
    import info

    bench_system = {
        'm' : 'client',
        's' : 'client',
        'snt' : 'client',
        'nb1' : 'server',
        'nb2' : 'server',
        'nt1' : 'server',
        'nt2' : 'server',
        'w1' : 'server',
        'w2' : 'server',
        'w3' : 'server',
        'w4' : 'server',
        'w1s' : 'server',
        'w2s' : 'server',
        'w3s' : 'server',
        'ns' : 'natbox',
        'nm' : 'natbox',
        'nw1' : 'natbox',
        'nw2' : 'natbox',
        'nw3' : 'natbox'
        }

    system_configs = {
        's1' : 'Uni 4GHz',
        'm1' : 'Uni 6GHz',
        'f1' : 'Uni 8GHz',
        'q1' : 'Uni 10GHz',
        's2' : 'Dual 4GHz',
        'm2' : 'Dual 6GHz',
        's4' : 'Quad 4GHz',
        'm4' : 'Quad 6GHz' }

    configs = ['ste', 'hte', 'htd', 'ocm', 'occ', 'ocp' ]
    benchmarks = [ 'm', 'snt', 'w2', 'nm', 'nw2' ]
    caches = [ '0', '2', '4' ]

    names = []
    for bench in benchmarks:
        if bench_system[bench] != options.system:
            continue

        for cache in caches:
            names.append([bench, cache])

    for bench,cache in names:
        base = '%s.%s' % (bench, cache)
        fname = 'data/uni.%s.%s.dat' % (tag, base)
        f = open(fname, 'w')
        print >>f, '#set TITLE = '
        print >>f, '#set ylbl = %s' % label
        #print >>f, '#set sublabels = %s' % ' '.join(configs)
        print >>f, '#set sublabels = ste hte htd ocm occ ocs'

        for speed in ('s1', 'm1', 'f1', 'q1'):
            label = system_configs[speed]
            print >>f, '"%s"' % label,
            for conf in configs:
                name = '%s.%s.%s.%s' % (conf, bench, cache, speed)
                run = info.source.allRunNames[name]
                info.display_run = run.run;
                val = float(value)
                if val == 1e300*1e300:
                    print >>f, 0.0,
                else:
                    print >>f, "%f" % val,
            print >>f
        f.close()

    configs = ['ste', 'hte', 'htd', 'ocm', 'occ', 'ocp' ]
    benchmarks = [ 'w2']
    caches = [ '0', '2', '4' ]

    names = []
    for bench in benchmarks:
        if bench_system[bench] != options.system:
            continue

        for cache in caches:
            names.append([bench, cache])

    for bench,cache in names:
        base = '%s.%s' % (bench, cache)
        fname = 'data/mp.%s.%s.dat' % (tag, base)
        f = open(fname, 'w')
        print >>f, '#set TITLE = '
        print >>f, '#set ylbl = %s' % label
        #print >>f, '#set sublabels = %s' % ' '.join(configs)
        print >>f, '#set sublabels = ste hte htd ocm occ ocs'

        for speed in ('s2', 'm2', 's4', 'm4'):
            label = system_configs[speed]
            print >>f, '"%s"' % label,
            for conf in configs:
                name = '%s.%s.%s.%s' % (conf, bench, cache, speed)
                run = info.source.allRunNames[name]
                info.display_run = run.run;
                val = float(value)
                if val == 1e300*1e300:
                    print >>f, 0.0,
                else:
                    print >>f, "%f" % val,
            print >>f
        f.close()

def printdata(runs, value, invert = False):
    import info
    for run in runs:
        info.display_run = run.run;
        val = float(value)
        printval(run.name, val)

class CommandException(Exception):
    pass

def commands(options, command, args):
    if command == 'database':
        if len(args) == 0: raise CommandException

        import dbinit
        mydb = dbinit.MyDB(options)

        if args[0] == 'drop':
            if len(args) > 2: raise CommandException
            mydb.admin()
            mydb.drop()
            if len(args) == 2 and args[1] == 'init':
                mydb.create()
                mydb.connect()
                mydb.populate()
            mydb.close()
            return

        if args[0] == 'init':
            if len(args) > 1: raise CommandException
            mydb.admin()
            mydb.create()
            mydb.connect()
            mydb.populate()
            mydb.close()
            return

        if args[0] == 'clean':
            if len(args) > 1: raise CommandException
            mydb.connect()
            mydb.clean()
            return

        raise CommandException

    import db, info
    info.source = db.Database()
    info.source.host = options.host
    info.source.db = options.db
    info.source.passwd = options.passwd
    info.source.user = options.user
    info.source.connect()
    info.source.update_dict(globals())

    if type(options.get) is str:
        info.source.get = options.get

    if options.runs is None:
        runs = info.source.allRuns
    else:
        rx = re.compile(options.runs)
        runs = []
        for run in info.source.allRuns:
            if rx.match(run.name):
                runs.append(run)

    info.display_run = runs[0].run

    if command == 'runs':
        user = None
        opts, args = getopts(args, '-u')
        if len(args):
            raise CommandException
        for o,a in opts:
            if o == '-u':
                user = a
        info.source.listRuns(user)
        return

    if command == 'stability':
        if len(args) < 2:
            raise CommandException

        try:
            merge = int(args[0])
        except ValueError:
            usage()
        stats = info.source.getStat(args[1])
        info.source.get = "sum"


        #loop through all the stats selected
        for stat in stats:

            print "%s:" % stat.name
            print "%-20s %12s %12s %4s %5s %5s %5s %10s" % \
                  ("run name", "average", "stdev", ">10%", ">1SDV", ">2SDV", "SAMP", "CV")
            print "%-20s %12s %12s %4s %5s %5s %5s %10s" % \
                  ("--------------------", "------------",
                   "------------", "----", "-----", "-----", "-----", "----------")
            #loop through all the selected runs
            for run in runs:
                info.display_run = run.run;
                runTicks = info.source.retTicks([ run ])
                #throw away the first one, it's 0
                runTicks.pop(0)
                info.globalTicks = runTicks
                avg = 0
                stdev = 0
                numoutsideavg  = 0
                numoutside1std = 0
                numoutside2std = 0
                pairRunTicks = []
                if float(stat) == 1e300*1e300:
                    continue
                for t in range(0, len(runTicks)-(merge-1), merge):
                    tempPair = []
                    for p in range(0,merge):
                        tempPair.append(runTicks[t+p])
                    pairRunTicks.append(tempPair)
                #loop through all the various ticks for each run
                for tick in pairRunTicks:
                    info.globalTicks = tick
                    avg += float(stat)
                avg /= len(pairRunTicks)
                for tick in pairRunTicks:
                    info.globalTicks = tick
                    val = float(stat)
                    stdev += pow((val-avg),2)
                stdev = math.sqrt(stdev / len(pairRunTicks))
                for tick in pairRunTicks:
                    info.globalTicks = tick
                    val = float(stat)
                    if (val < (avg * .9)) or (val > (avg * 1.1)):
                        numoutsideavg += 1
                    if (val < (avg - stdev)) or (val > (avg + stdev)):
                        numoutside1std += 1
                    if (val < (avg - (2*stdev))) or (val > (avg + (2*stdev))):
                        numoutside2std += 1
                if avg > 1000:
                    print "%-20s %12s %12s %4s %5s %5s %5s %10s" % \
                          (run.name, "%.1f" % avg, "%.1f" % stdev,
                           "%d" % numoutsideavg, "%d" % numoutside1std,
                           "%d" % numoutside2std, "%d" % len(pairRunTicks),
                           "%.3f" % (stdev/avg*100))
                elif avg > 100:
                    print "%-20s %12s %12s %4s %5s %5s %5s %10s" % \
                          (run.name, "%.1f" % avg, "%.1f" % stdev,
                           "%d" % numoutsideavg, "%d" % numoutside1std,
                           "%d" % numoutside2std, "%d" % len(pairRunTicks),
                           "%.5f" % (stdev/avg*100))
                else:
                    print "%-20s %12s %12s %4s %5s %5s %5s %10s" % \
                          (run.name, "%.5f" % avg, "%.5f" % stdev,
                           "%d" % numoutsideavg, "%d" % numoutside1std,
                           "%d" % numoutside2std, "%d" % len(pairRunTicks),
                           "%.7f" % (stdev/avg*100))
        return

    if command == 'stats':
        if len(args) == 0:
            info.source.listStats()
        elif len(args) == 1:
            info.source.listStats(args[0])
        else:
            raise CommandException

        return

    if command == 'stat':
        if len(args) != 1:
            raise CommandException

        stats = info.source.getStat(args[0])
        for stat in stats:
            if options.graph:
                graphdata(runs, options, stat.name, stat.name, stat)
            else:
                if options.ticks:
                   print 'only displaying sample %s' % options.ticks
                   info.globalTicks = [ int(x) for x in options.ticks.split() ]

                if options.binned:
                    print 'kernel ticks'
                    stat.bins = 'kernel'
                    printdata(runs, stat)

                    print 'idle ticks'
                    stat.bins = 'idle'
                    printdata(runs, stat)

                    print 'user ticks'
                    stat.bins = 'user'
                    printdata(runs, stat)

                    print 'interrupt ticks'
                    stat.bins = 'interrupt'
                    printdata(runs, stat)

                    print 'total ticks'

                stat.bins = None
                print stat.name
                printdata(runs, stat)
        return

    if command == 'formula':
        if len(args) != 1:
            raise CommandException

        stats = eval(args[0])
        for stat in stats:
            if options.graph:
                graphdata(runs, options, stat.name, stat.name, stat)
            else:
                if options.binned:
                    print 'kernel ticks'
                    stat.bins = 'kernel'
                    printdata(runs, stat)

                    print 'idle ticks'
                    stat.bins = 'idle'
                    printdata(runs, stat)

                    print 'user ticks'
                    stat.bins = 'user'
                    printdata(runs, stat)

                    print 'interrupt ticks'
                    stat.bins = 'interrupt'
                    printdata(runs, stat)

                    print 'total ticks'

                stat.bins = None
                print args[0]
                printdata(runs, stat)
        return

    if command == 'bins':
        if len(args) == 0:
            info.source.listBins()
        elif len(args) == 1:
            info.source.listBins(args[0])
        else:
            raise CommandException

        return

    if command == 'formulas':
        if len(args) == 0:
            info.source.listFormulas()
        elif len(args) == 1:
            info.source.listFormulas(args[0])
        else:
            raise CommandException

        return

    if command == 'samples':
        if len(args):
            raise CommandException

        info.source.listTicks(runs)
        return

    if len(args):
        raise CommandException

    system = info.source.__dict__[options.system]

    if command == 'usertime':
        import copy
        kernel = copy.copy(system.full0.numCycles)
        kernel.bins = 'kernel'

        user = copy.copy(system.full0.numCycles)
        user.bins = 'user'

        if options.graph:
            graphdata(runs, options, 'usertime', 'User Fraction',
                      user / system.full0.numCycles)
        else:
            printdata(runs, user / system.full0.numCycles)
        return

    if command == 'ticks':
        if options.binned:
            print 'kernel ticks'
            system.full0.numCycles.bins = 'kernel'
            printdata(runs, system.full0.numCycles)

            print 'idle ticks'
            system.full0.numCycles.bins = 'idle'
            printdata(runs, system.full0.numCycles)

            print 'user ticks'
            system.full0.numCycles.bins = 'user'
            printdata(runs, system.full0.numCycles)

            print 'total ticks'

        system.full0.numCycles.bins = None
        printdata(runs, system.full0.numCycles)
        return

    if command == 'packets':
        packets = system.tsunami.etherdev0.rxPackets
        if options.graph:
            graphdata(runs, options, 'packets', 'Packets', packets)
        else:
            printdata(runs, packets)
        return

    if command == 'ppt' or command == 'tpp':
        ppt = system.tsunami.etherdev0.rxPackets / sim_ticks
        printdata(runs, ppt, command == 'tpp')
        return

    if command == 'pps':
        pps = system.tsunami.etherdev0.rxPackets / sim_seconds
        if options.graph:
            graphdata(runs, options, 'pps', 'Packets/s', pps)
        else:
            printdata(runs, pps)
        return

    if command == 'bpt' or command == 'tpb':
        bytes = system.tsunami.etherdev0.rxBytes + system.tsunami.etherdev0.txBytes
        bpt = bytes / sim_ticks * 8
        if options.graph:
            graphdata(runs, options, 'bpt', 'bps / Hz', bpt)
        else:
            printdata(runs, bpt, command == 'tpb')
        return

    if command == 'bptb' or command == 'tpbb':
        bytes = system.tsunami.etherdev0.rxBytes + system.tsunami.etherdev0.txBytes

        print 'kernel stats'
        bytes.bins = 'kernel'
        printdata(runs, bytes / ticks)

        print 'idle stats'
        bytes.bins = 'idle'
        printdata(runs, bytes / ticks)

        print 'user stats'
        bytes.bins = 'user'
        printdata(runs, bytes / ticks)

        return

    if command == 'bytes':
        stat = system.tsunami.etherdev0.rxBytes + system.tsunami.etherdev0.txBytes

        if options.binned:
            print '%s kernel stats' % stat.name
            stat.bins = 'kernel'
            printdata(runs, stat)

            print '%s idle stats' % stat.name
            stat.bins = 'idle'
            printdata(runs, stat)

            print '%s user stats' % stat.name
            stat.bins = 'user'
            printdata(runs, stat)

            print '%s total stats' % stat.name
            stat.bins = None

        printdata(runs, stat)
        return

    if command == 'rxbps':
        gbps = system.tsunami.etherdev0.rxBandwidth / 1e9
        if options.graph:
            graphdata(runs, options, 'rxbps', 'Bandwidth (Gbps)',  gbps)
        else:
            printdata(runs, gbps)
        return

    if command == 'txbps':
        gbps = system.tsunami.etherdev0.txBandwidth / 1e9
        if options.graph:
            graphdata(runs, options, 'txbps', 'Bandwidth (Gbps)',  gbps)
        else:
            printdata(runs, gbps)
        return

    if command == 'bps':
        rxbps = system.tsunami.etherdev0.rxBandwidth
        txbps = system.tsunami.etherdev0.txBandwidth
        gbps = (rxbps + txbps) / 1e9
        if options.graph:
            graphdata(runs, options, 'bps', 'Bandwidth (Gbps)',  gbps)
        else:
            printdata(runs, gbps)
        return

    if command == 'misses':
        stat = system.l2.overall_mshr_misses
        if options.binned:
            print '%s kernel stats' % stat.name
            stat.bins = 'kernel'
            printdata(runs, stat)

            print '%s idle stats' % stat.name
            stat.bins = 'idle'
            printdata(runs, stat)

            print '%s user stats' % stat.name
            stat.bins = 'user'
            printdata(runs, stat)

            print '%s total stats' % stat.name

        stat.bins = None
        if options.graph:
            graphdata(runs, options, 'misses', 'Overall MSHR Misses', stat)
        else:
            printdata(runs, stat)
        return

    if command == 'mpkb':
        misses = system.l2.overall_mshr_misses
        rxbytes = system.tsunami.etherdev0.rxBytes
        txbytes = system.tsunami.etherdev0.txBytes

        if options.binned:
            print 'mpkb kernel stats'
            misses.bins = 'kernel'
            mpkb = misses / ((rxbytes + txbytes) / 1024)
            printdata(runs, mpkb)

            print 'mpkb idle stats'
            misses.bins = 'idle'
            mpkb = misses / ((rxbytes + txbytes) / 1024)
            printdata(runs, mpkb)

            print 'mpkb user stats'
            misses.bins = 'user'
            mpkb = misses / ((rxbytes + txbytes) / 1024)
            printdata(runs, mpkb)

            print 'mpkb total stats'

        mpkb = misses / ((rxbytes + txbytes) / 1024)
        misses.bins = None
        if options.graph:
            graphdata(runs, options, 'mpkb', 'Misses / KB',  mpkb)
        else:
            printdata(runs, mpkb)
        return

    if command == 'ipkb':
        interrupts = system.full0.kern.faults[4]
        rxbytes = system.tsunami.etherdev0.rxBytes
        txbytes = system.tsunami.etherdev0.txBytes

        if options.binned:
            print 'ipkb kernel stats'
            interrupts.bins = 'kernel'
            ipkb = interrupts / ((rxbytes + txbytes) / 1024)
            printdata(runs, ipkb)

            print 'ipkb idle stats'
            interrupts.bins = 'idle'
            ipkb = interrupts / ((rxbytes + txbytes) / 1024)
            printdata(runs, ipkb)

            print 'ipkb user stats'
            interrupts.bins = 'user'
            ipkb = interrupts / ((rxbytes + txbytes) / 1024)
            printdata(runs, ipkb)

            print 'ipkb total stats'

        ipkb = interrupts / ((rxbytes + txbytes) / 1024)
        interrupts.bins = None
        if options.graph:
            graphdata(runs, options, 'ipkb', 'Interrupts / KB',  ipkb)
        else:
            printdata(runs, ipkb)
        return

    if command == 'execute':
        printdata(runs, system.full0.ISSUE__count)
        return

    if command == 'commit':
        printdata(runs, system.full0.COM__count)
        return

    if command == 'fetch':
        printdata(runs, system.full0.FETCH__count)
        return

    if command == 'bpp':
        ed = system.tsunami.etherdev0
        bpp = (ed.rxBytes + ed.txBytes) / (ed.rxPackets + ed.txPackets)
        if options.graph:
            graphdata(runs, options, 'bpp', 'Bytes / Packet',  bpp)
        else:
            printdata(runs, bpp)
        return

    if command == 'rxbpp':
        bpp = system.tsunami.etherdev0.rxBytes / system.tsunami.etherdev0.rxPackets
        if options.graph:
            graphdata(runs, options, 'rxbpp', 'Receive Bytes / Packet',  bpp)
        else:
            printdata(runs, bpp)
        return

    if command == 'txbpp':
        bpp = system.tsunami.etherdev0.txBytes / system.tsunami.etherdev0.txPackets
        if options.graph:
            graphdata(runs, options, 'txbpp', 'Transmit Bytes / Packet',  bpp)
        else:
            printdata(runs, bpp)
        return

    if command == 'rtp':
        rtp = system.tsunami.etherdev0.rxPackets / system.tsunami.etherdev0.txPackets
        if options.graph:
            graphdata(runs, options, 'rtp', 'rxPackets / txPackets',  rtp)
        else:
            printdata(runs, rtp)
        return

    if command == 'rtb':
        rtb = system.tsunami.etherdev0.rxBytes / system.tsunami.etherdev0.txBytes
        if options.graph:
            graphdata(runs, options, 'rtb', 'rxBytes / txBytes',  rtb)
        else:
            printdata(runs, rtb)
        return

    raise CommandException


class Options: pass

if __name__ == '__main__':
    import getpass

    options = Options()
    options.host = 'zizzer.pool'
    options.db = None
    options.passwd = ''
    options.user = getpass.getuser()
    options.runs = None
    options.system = 'client'
    options.get = None
    options.binned = False
    options.graph = False
    options.ticks = False

    opts, args = getopts(sys.argv[1:], '-6BEFGd:g:h:pr:s:u:T:')
    for o,a in opts:
        if o == '-B':
            options.binned = True
        if o == '-E':
            printval.mode = 'E'
        if o == '-F':
            printval.mode = 'F'
        if o == '-G':
            options.graph = True;
        if o == '-d':
            options.db = a
        if o == '-g':
            options.get = a
        if o == '-h':
            options.host = a
        if o == '-p':
            options.passwd = getpass.getpass()
        if o == '-r':
            options.runs = a
        if o == '-u':
            options.user = a
        if o == '-s':
            options.system = a
        if o == '-T':
            options.ticks = a

    if len(args) == 0:
        usage()

    command = args[0]
    args = args[1:]

    try:
        commands(options, command, args)
    except CommandException:
        usage()
