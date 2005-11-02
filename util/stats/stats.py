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

from __future__ import division
import re, sys, math

def usage():
    print '''\
Usage: %s [-E] [-F] [ -G <get> ] [-d <db> ] [-g <graphdir> ] [-h <host>] [-p]
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
    #info.source.update_dict(globals())

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

    if command == 'stats':
        if len(args) == 0:
            info.source.listStats()
        elif len(args) == 1:
            info.source.listStats(args[0])
        else:
            raise CommandException

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

    if command == 'stability':
        if len(args) < 2:
            raise CommandException

        try:
            merge = int(args[0])
        except ValueError:
            usage()
        stats = info.source.getStat(args[1])
        info.source.get = "sum"

        def disp(*args):
            print "%-35s %12s %12s %4s %5s %5s %5s %10s" % args

        # temporary variable containing a bunch of dashes
        d = '-' * 100

        #loop through all the stats selected
        for stat in stats:
            print "%s:" % stat.name
            disp("run name", "average", "stdev", ">10%", ">1SDV", ">2SDV",
                 "SAMP", "CV")
            disp(d[:35], d[:12], d[:12], d[:4], d[:5], d[:5], d[:5], d[:10])

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
                    disp(run.name, "%.1f" % avg, "%.1f" % stdev,
                         "%d" % numoutsideavg, "%d" % numoutside1std,
                         "%d" % numoutside2std, "%d" % len(pairRunTicks),
                         "%.3f" % (stdev/avg*100))
                elif avg > 100:
                    disp(run.name, "%.1f" % avg, "%.1f" % stdev,
                         "%d" % numoutsideavg, "%d" % numoutside1std,
                         "%d" % numoutside2std, "%d" % len(pairRunTicks),
                         "%.5f" % (stdev/avg*100))
                else:
                    disp(run.name, "%.5f" % avg, "%.5f" % stdev,
                         "%d" % numoutsideavg, "%d" % numoutside1std,
                         "%d" % numoutside2std, "%d" % len(pairRunTicks),
                         "%.7f" % (stdev/avg*100))
        return

    if command == 'all':
        if len(args):
            raise CommandException

        all = [ 'bps', 'rxbps', 'txbps', 'bpt',
                'misses', 'mpkb',
                'ipkb',
                'pps', 'bpp', 'txbpp', 'rxbpp',
                'rtp', 'rtb' ]
        for command in all:
            commands(options, command, args)

    if options.ticks:
        if not options.graph:
            print 'only displaying sample %s' % options.ticks
        info.globalTicks = [ int(x) for x in options.ticks.split() ]

    from output import StatOutput

    def display():
        if options.graph:
            output.graph(options.graphdir)
        else:
            output.display(options.binned, options.printmode)


    if command == 'stat' or command == 'formula':
        if len(args) != 1:
            raise CommandException

        if command == 'stat':
            stats = info.source.getStat(args[0])
        if command == 'formula':
            stats = eval(args[0])

        for stat in stats:
            output = StatOutput(stat.name, options.jobfile)
            output.stat = stat
            output.label = stat.name
            display()

        return

    if len(args):
        raise CommandException

    system = info.source.__dict__[options.system]

    from proxy import ProxyGroup
    sim_ticks = info.source['sim_ticks']
    sim_seconds = info.source['sim_seconds']
    proxy = ProxyGroup(system = info.source[options.system])
    system = proxy.system

    etherdev = system.tsunami.etherdev0
    bytes = etherdev.rxBytes + etherdev.txBytes
    kbytes = bytes / 1024
    packets = etherdev.rxPackets + etherdev.txPackets
    bps = etherdev.rxBandwidth + etherdev.txBandwidth

    output = StatOutput(command, options.jobfile)

    if command == 'usertime':
        import copy
        user = copy.copy(system.full0.numCycles)
        user.bins = 'user'

        output.stat = user / system.full0.numCycles
        output.label = 'User Fraction'

        display()
        return

    if command == 'ticks':
        output.stat = system.full0.numCycles
        output.binstats = [ system.full0.numCycles ]

        display()
        return

    if command == 'bytes':
        output.stat = bytes
        display()
        return

    if command == 'packets':
        output.stat = packets
        display()
        return

    if command == 'ppt' or command == 'tpp':
        output.stat = packets / sim_ticks
        output.invert = command == 'tpp'
        display()
        return

    if command == 'pps':
        output.stat = packets / sim_seconds
        output.label = 'Packets/s'
        display()
        return

    if command == 'bpt' or command == 'tpb':
        output.stat = bytes / sim_ticks * 8
        output.label = 'bps / Hz'
        output.invert = command == 'tpb'
        display()
        return

    if command == 'rxbps':
        output.stat = etherdev.rxBandwidth / 1e9
        output.label = 'Bandwidth (Gbps)'
        display()
        return

    if command == 'txbps':
        output.stat = etherdev.txBandwidth / 1e9
        output.label = 'Bandwidth (Gbps)'
        display()
        return

    if command == 'bps':
        output.stat = bps / 1e9
        output.label = 'Bandwidth (Gbps)'
        display()
        return

    if command == 'bpp':
        output.stat = bytes / packets
        output.label = 'Bytes / Packet'
        display()
        return

    if command == 'rxbpp':
        output.stat = etherdev.rxBytes / etherdev.rxPackets
        output.label = 'Receive Bytes / Packet'
        display()
        return

    if command == 'txbpp':
        output.stat = etherdev.txBytes / etherdev.txPackets
        output.label = 'Transmit Bytes / Packet'
        display()
        return

    if command == 'rtp':
        output.stat = etherdev.rxPackets / etherdev.txPackets
        output.label = 'rxPackets / txPackets'
        display()
        return

    if command == 'rtb':
        output.stat = etherdev.rxBytes / etherdev.txBytes
        output.label = 'rxBytes / txBytes'
        display()
        return

    misses = system.l2.overall_mshr_misses

    if command == 'misses':
        output.stat = misses
        output.label = 'Overall MSHR Misses'
        display()
        return

    if command == 'mpkb':
        output.stat = misses / (bytes / 1024)
        output.binstats = [ misses ]
        output.label = 'Misses / KB'
        display()
        return

    if command == 'ipkb':
        interrupts = system.full0.kern.faults[4]
        output.stat = interrupts / kbytes
        output.binstats = [ interrupts ]
        output.label = 'Interrupts / KB'
        display()
        return

    if command == 'execute':
        output.stat = system.full0.ISSUE__count
        display()
        return

    if command == 'commit':
        output.stat = system.full0.COM__count
        display()
        return

    if command == 'fetch':
        output.stat = system.full0.FETCH__count
        display()
        return

    raise CommandException


class Options: pass

if __name__ == '__main__':
    import getpass
    from jobfile import JobFile

    options = Options()
    options.host = None
    options.db = None
    options.passwd = ''
    options.user = getpass.getuser()
    options.runs = None
    options.system = 'client'
    options.get = None
    options.binned = False
    options.graph = False
    options.ticks = False
    options.printmode = 'G'
    jobfilename = 'Test.py'
    options.jobfile = None
    options.all = False

    opts, args = getopts(sys.argv[1:], '-BEFG:Jad:g:h:j:pr:s:u:T:')
    for o,a in opts:
        if o == '-B':
            options.binned = True
        if o == '-E':
            options.printmode = 'E'
        if o == '-F':
            options.printmode = 'F'
        if o == '-G':
            options.get = a
        if o == '-a':
            options.all = True
        if o == '-d':
            options.db = a
        if o == '-g':
            options.graph = True;
            options.graphdir = a
        if o == '-h':
            options.host = a
        if o == '-J':
            jobfilename = None
        if o == '-j':
            jobfilename = a
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

    if jobfilename:
        options.jobfile = JobFile(jobfilename)
        if not options.host:
            options.host = options.jobfile.dbhost
        if not options.db:
            options.db = options.jobfile.statdb

    if not options.host:
        sys.exit('Database server must be provided from a jobfile or -h')

    if not options.db:
        sys.exit('Database name must be provided from a jobfile or -d')

    if len(args) == 0:
        usage()

    command = args[0]
    args = args[1:]

    try:
        commands(options, command, args)
    except CommandException:
        usage()
