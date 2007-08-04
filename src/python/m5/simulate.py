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

import atexit
import os
import sys

# import the SWIG-wrapped main C++ functions
import internal
from main import options
import SimObject
import ticks
import objects

# The final hook to generate .ini files.  Called from the user script
# once the config is built.
def instantiate(root):
    # we need to fix the global frequency
    ticks.fixGlobalFrequency()

    root.unproxy_all()
    # ugly temporary hack to get output to config.ini
    sys.stdout = file(os.path.join(options.outdir, 'config.ini'), 'w')
    root.print_ini()
    sys.stdout.close() # close config.ini
    sys.stdout = sys.__stdout__ # restore to original

    # Initialize the global statistics
    internal.stats.initSimStats()

    # Create the C++ sim objects and connect ports
    root.createCCObject()
    root.connectPorts()

    # Do a second pass to finish initializing the sim objects
    internal.core.initAll()

    # Do a third pass to initialize statistics
    internal.core.regAllStats()

    # Check to make sure that the stats package is properly initialized
    internal.stats.check()

    # Reset to put the stats in a consistent state.
    internal.stats.reset()

def doDot(root):
    dot = pydot.Dot()
    instance.outputDot(dot)
    dot.orientation = "portrait"
    dot.size = "8.5,11"
    dot.ranksep="equally"
    dot.rank="samerank"
    dot.write("config.dot")
    dot.write_ps("config.ps")

need_resume = []
need_startup = True
def simulate(*args, **kwargs):
    global need_resume, need_startup

    if need_startup:
        internal.core.SimStartup()
        need_startup = False

    for root in need_resume:
        resume(root)
    need_resume = []

    return internal.event.simulate(*args, **kwargs)

# Export curTick to user script.
def curTick():
    return internal.core.cvar.curTick

# Python exit handlers happen in reverse order.  We want to dump stats last.
atexit.register(internal.stats.dump)

# register our C++ exit callback function with Python
atexit.register(internal.core.doExitCleanup)

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
    drain_event = internal.event.createCountedDrain()
    unready_objects = root.startDrain(drain_event, True)
    # If we've got some objects that can't drain immediately, then simulate
    if unready_objects > 0:
        drain_event.setCount(unready_objects)
        simulate()
    else:
        all_drained = True
    internal.event.cleanupCountedDrain(drain_event)
    return all_drained

def resume(root):
    root.resume()

def checkpoint(root, dir):
    if not isinstance(root, objects.Root):
        raise TypeError, "Checkpoint must be called on a root object."
    doDrain(root)
    print "Writing checkpoint"
    internal.core.serializeAll(dir)
    resume(root)

def restoreCheckpoint(root, dir):
    print "Restoring from checkpoint"
    internal.core.unserializeAll(dir)
    need_resume.append(root)

def changeToAtomic(system):
    if not isinstance(system, (objects.Root, objects.System)):
        raise TypeError, "Parameter of type '%s'.  Must be type %s or %s." % \
              (type(system), objects.Root, objects.System)
    if system.getMemoryMode() != objects.params.atomic:
        doDrain(system)
        print "Changing memory mode to atomic"
        system.changeTiming(objects.params.atomic)

def changeToTiming(system):
    if not isinstance(system, (objects.Root, objects.System)):
        raise TypeError, "Parameter of type '%s'.  Must be type %s or %s." % \
              (type(system), objects.Root, objects.System)

    if system.getMemoryMode() != objects.params.timing:
        doDrain(system)
        print "Changing memory mode to timing"
        system.changeTiming(objects.params.timing)

def switchCpus(cpuList):
    print "switching cpus"
    if not isinstance(cpuList, list):
        raise RuntimeError, "Must pass a list to this function"
    for i in cpuList:
        if not isinstance(i, tuple):
            raise RuntimeError, "List must have tuples of (oldCPU,newCPU)"

    [old_cpus, new_cpus] = zip(*cpuList)

    for cpu in old_cpus:
        if not isinstance(cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU" % cpu
    for cpu in new_cpus:
        if not isinstance(cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU" % cpu

    # Drain all of the individual CPUs
    drain_event = internal.event.createCountedDrain()
    unready_cpus = 0
    for old_cpu in old_cpus:
        unready_cpus += old_cpu.startDrain(drain_event, False)
    # If we've got some objects that can't drain immediately, then simulate
    if unready_cpus > 0:
        drain_event.setCount(unready_cpus)
        simulate()
    internal.event.cleanupCountedDrain(drain_event)
    # Now all of the CPUs are ready to be switched out
    for old_cpu in old_cpus:
        old_cpu._ccObject.switchOut()
    index = 0
    for new_cpu in new_cpus:
        new_cpu.takeOverFrom(old_cpus[index])
        new_cpu._ccObject.resume()
        index += 1
