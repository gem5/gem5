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


# Callback to set trace flags.  Not necessarily the best way to do
# things in the long run (particularly if we change how these global
# options are handled).
def setTraceFlags(option, opt_str, value, parser):
    objects.Trace.flags = value

def setTraceStart(option, opt_str, value, parser):
    objects.Trace.start = value

def clearPCSymbol(option, opt_str, value, parser):
    objects.ExecutionTrace.pc_symbol = False

def clearPrintCycle(option, opt_str, value, parser):
    objects.ExecutionTrace.print_cycle = False

def statsTextFile(option, opt_str, value, parser):
    objects.Statistics.text_file = value

# Standard optparse options.  Need to be explicitly included by the
# user script when it calls optparse.OptionParser().
standardOptions = [
    optparse.make_option("--traceflags", type="string", action="callback",
                         callback=setTraceFlags),
    optparse.make_option("--tracestart", type="int", action="callback",
                         callback=setTraceStart),
    optparse.make_option("--nopcsymbol", action="callback",
                         callback=clearPCSymbol,
                         help="Turn off printing PC symbols in trace output"),
    optparse.make_option("--noprintcycle", action="callback",
                         callback=clearPrintCycle,
                         help="Turn off printing cycles in trace output"),
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

# The final hook to generate .ini files.  Called from the user script
# once the config is built.
def instantiate(root):
    config.ticks_per_sec = float(root.clock.frequency)
    # ugly temporary hack to get output to config.ini
    sys.stdout = file('config.ini', 'w')
    root.print_ini()
    sys.stdout.close() # close config.ini
    sys.stdout = sys.__stdout__ # restore to original
    main.initialize()  # load config.ini into C++ and process it
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
