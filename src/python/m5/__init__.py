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

import atexit, os, sys

# import the SWIG-wrapped main C++ functions
import cc_main
# import a few SWIG-wrapped items (those that are likely to be used
# directly by user scripts) completely into this module for
# convenience
from cc_main import simulate, SimLoopExitEvent

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

from main import options, arguments, main

# The final hook to generate .ini files.  Called from the user script
# once the config is built.
def instantiate(root):
    config.ticks_per_sec = float(root.clock.frequency)
    # ugly temporary hack to get output to config.ini
    sys.stdout = file(os.path.join(options.outdir, 'config.ini'), 'w')
    root.print_ini()
    sys.stdout.close() # close config.ini
    sys.stdout = sys.__stdout__ # restore to original
    cc_main.loadIniFile(resolveSimObject)  # load config.ini into C++
    root.createCCObject()
    root.connectPorts()
    cc_main.finalInit()
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
    return cc_main.cvar.curTick

# register our C++ exit callback function with Python
atexit.register(cc_main.doExitCleanup)

# This import allows user scripts to reference 'm5.objects.Foo' after
# just doing an 'import m5' (without an 'import m5.objects').  May not
# matter since most scripts will probably 'from m5.objects import *'.
import objects

# This loops until all objects have been fully drained.
def doDrain(root):
    all_drained = drain(root)
    while (not all_drained):
        all_drained = drain(root)

# Tries to drain all objects.  Draining might not be completed unless
# all objects return that they are drained on the first call.  This is
# because as objects drain they may cause other objects to no longer
# be drained.
def drain(root):
    all_drained = False
    drain_event = cc_main.createCountedDrain()
    unready_objects = root.startDrain(drain_event, True)
    # If we've got some objects that can't drain immediately, then simulate
    if unready_objects > 0:
        drain_event.setCount(unready_objects)
        simulate()
    else:
        all_drained = True
    cc_main.cleanupCountedDrain(drain_event)
    return all_drained

def resume(root):
    root.resume()

def checkpoint(root, dir):
    if not isinstance(root, objects.Root):
        raise TypeError, "Object is not a root object. Checkpoint must be called on a root object."
    doDrain(root)
    print "Writing checkpoint"
    cc_main.serializeAll(dir)
    resume(root)

def restoreCheckpoint(root, dir):
    print "Restoring from checkpoint"
    cc_main.unserializeAll(dir)
    resume(root)

def changeToAtomic(system):
    if not isinstance(system, objects.Root) and not isinstance(system, System):
        raise TypeError, "Object is not a root or system object.  Checkpoint must be "
        "called on a root object."
    doDrain(system)
    print "Changing memory mode to atomic"
    system.changeTiming(cc_main.SimObject.Atomic)
    resume(system)

def changeToTiming(system):
    if not isinstance(system, objects.Root) and not isinstance(system, System):
        raise TypeError, "Object is not a root or system object.  Checkpoint must be "
        "called on a root object."
    doDrain(system)
    print "Changing memory mode to timing"
    system.changeTiming(cc_main.SimObject.Timing)
    resume(system)

def switchCpus(cpuList):
    if not isinstance(cpuList, list):
        raise RuntimeError, "Must pass a list to this function"
    for i in cpuList:
        if not isinstance(i, tuple):
            raise RuntimeError, "List must have tuples of (oldCPU,newCPU)"

    [old_cpus, new_cpus] = zip(*cpuList)

    for cpu in old_cpus:
        if not isinstance(cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU", cpu
    for cpu in new_cpus:
        if not isinstance(cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU", cpu

    # Drain all of the individual CPUs
    drain_event = cc_main.createCountedDrain()
    unready_cpus = 0
    for old_cpu in old_cpus:
        unready_cpus += old_cpu.startDrain(drain_event, False)
    # If we've got some objects that can't drain immediately, then simulate
    if unready_cpus > 0:
        drain_event.setCount(unready_cpus)
        simulate()
    cc_main.cleanupCountedDrain(drain_event)
    # Now all of the CPUs are ready to be switched out
    for old_cpu in old_cpus:
        old_cpu._ccObject.switchOut()
    index = 0
    print "Switching CPUs"
    for new_cpu in new_cpus:
        new_cpu.takeOverFrom(old_cpus[index])
        new_cpu._ccObject.resume()
        index += 1
