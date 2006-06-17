# Copyright (c) 2005 The Regents of The University of Michigan
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

import sys, os, time, atexit, optparse

# import the SWIG-wrapped main C++ functions
import main
# import a few SWIG-wrapped items (those that are likely to be used
# directly by user scripts) completely into this module for
# convenience
from main import simulate, SimLoopExitEvent

# import the m5 compile options
import defines

# define this here so we can use it right away if necessary
def panic(string):
    print >>sys.stderr, 'panic:', string
    sys.exit(1)

# Prepend given directory to system module search path.  We may not
# need this anymore if we can structure our config library more like a
# Python package.
def AddToPath(path):
    # if it's a relative path and we know what directory the current
    # python script is in, make the path relative to that directory.
    if not os.path.isabs(path) and sys.path[0]:
        path = os.path.join(sys.path[0], path)
    path = os.path.realpath(path)
    # sys.path[0] should always refer to the current script's directory,
    # so place the new dir right after that.
    sys.path.insert(1, path)


# The m5 module's pointer to the parsed options object
options = None


# User should call this function after calling parse_args() to pass
# parsed standard option values back into the m5 module for
# processing.
def setStandardOptions(_options):
    # Set module global var
    global options
    options = _options
    # tell C++ about output directory
    main.setOutputDir(options.outdir)

# Callback to set trace flags.  Not necessarily the best way to do
# things in the long run (particularly if we change how these global
# options are handled).
def setTraceFlags(option, opt_str, value, parser):
    objects.Trace.flags = value

def setTraceStart(option, opt_str, value, parser):
    objects.Trace.start = value

def setTraceFile(option, opt_str, value, parser):
    objects.Trace.file = value

def usePCSymbol(option, opt_str, value, parser):
    objects.ExecutionTrace.pc_symbol = value

def printCycle(option, opt_str, value, parser):
    objects.ExecutionTrace.print_cycle = value

def printOp(option, opt_str, value, parser):
    objects.ExecutionTrace.print_opclass = value

def printThread(option, opt_str, value, parser):
    objects.ExecutionTrace.print_thread = value

def printEA(option, opt_str, value, parser):
    objects.ExecutionTrace.print_effaddr = value

def printData(option, opt_str, value, parser):
    objects.ExecutionTrace.print_data = value

def printFetchseq(option, opt_str, value, parser):
    objects.ExecutionTrace.print_fetchseq = value

def printCpseq(option, opt_str, value, parser):
    objects.ExecutionTrace.print_cpseq = value

def dumpOnExit(option, opt_str, value, parser):
    objects.Trace.dump_on_exit = value

def debugBreak(option, opt_str, value, parser):
    objects.Debug.break_cycles = value

def statsTextFile(option, opt_str, value, parser):
    objects.Statistics.text_file = value

# Extra list to help for options that are true or false
TrueOrFalse = ['True', 'False']
TorF = "True | False"

# Standard optparse options.  Need to be explicitly included by the
# user script when it calls optparse.OptionParser().
standardOptions = [
    optparse.make_option("--outdir", type="string", default="."),
    optparse.make_option("--traceflags", type="string", action="callback",
                         callback=setTraceFlags),
    optparse.make_option("--tracestart", type="int", action="callback",
                         callback=setTraceStart),
    optparse.make_option("--tracefile", type="string", action="callback",
                         callback=setTraceFile),
    optparse.make_option("--pcsymbol", type="choice", choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=usePCSymbol,
                         help="Use PC symbols in trace output"),
    optparse.make_option("--printcycle", type="choice", choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printCycle,
                         help="Print cycle numbers in trace output"),
    optparse.make_option("--printopclass", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printOp,
                         help="Print cycle numbers in trace output"),
    optparse.make_option("--printthread", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printThread,
                         help="Print thread number in trace output"),
    optparse.make_option("--printeffaddr", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printEA,
                         help="Print effective address in trace output"),
    optparse.make_option("--printdata", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printData,
                         help="Print result data in trace output"),
    optparse.make_option("--printfetchseq", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printFetchseq,
                         help="Print fetch sequence numbers in trace output"),
    optparse.make_option("--printcpseq", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=printCpseq,
                         help="Print correct path sequence numbers in trace output"),
    optparse.make_option("--dumponexit", type="choice",
                         choices=TrueOrFalse,
                         default="True", metavar=TorF,
                         action="callback", callback=dumpOnExit,
                         help="Dump trace buffer on exit"),
    optparse.make_option("--debugbreak", type="int", metavar="CYCLE",
                         action="callback", callback=debugBreak,
                         help="Cycle to create a breakpoint"),
    optparse.make_option("--statsfile", type="string", action="callback",
                         callback=statsTextFile, metavar="FILE",
                         help="Sets the output file for the statistics")
    ]

# make a SmartDict out of the build options for our local use
import smartdict
build_env = smartdict.SmartDict()
build_env.update(defines.m5_build_env)

# make a SmartDict out of the OS environment too
env = smartdict.SmartDict()
env.update(os.environ)


# Function to provide to C++ so it can look up instances based on paths
def resolveSimObject(name):
    obj = config.instanceDict[name]
    return obj.getCCObject()

# The final hook to generate .ini files.  Called from the user script
# once the config is built.
def instantiate(root):
    config.ticks_per_sec = float(root.clock.frequency)
    # ugly temporary hack to get output to config.ini
    sys.stdout = file(os.path.join(options.outdir, 'config.ini'), 'w')
    root.print_ini()
    sys.stdout.close() # close config.ini
    sys.stdout = sys.__stdout__ # restore to original
    main.loadIniFile(resolveSimObject)  # load config.ini into C++
    root.createCCObject()
    root.connectPorts()
    main.finalInit()
    noDot = True # temporary until we fix dot
    if not noDot:
       dot = pydot.Dot()
       instance.outputDot(dot)
       dot.orientation = "portrait"
       dot.size = "8.5,11"
       dot.ranksep="equally"
       dot.rank="samerank"
       dot.write("config.dot")
       dot.write_ps("config.ps")

# Export curTick to user script.
def curTick():
    return main.cvar.curTick

# register our C++ exit callback function with Python
atexit.register(main.doExitCleanup)

# This import allows user scripts to reference 'm5.objects.Foo' after
# just doing an 'import m5' (without an 'import m5.objects').  May not
# matter since most scripts will probably 'from m5.objects import *'.
import objects
