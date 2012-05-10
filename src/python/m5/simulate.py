# Copyright (c) 2012 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2005 The Regents of The University of Michigan
# Copyright (c) 2010 Advanced Micro Devices, Inc.
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
import core
import stats
import SimObject
import ticks
import objects
from m5.util.dot_writer import do_dot

from util import fatal
from util import attrdict

# define a MaxTick parameter
MaxTick = 2**63 - 1

# The final hook to generate .ini files.  Called from the user script
# once the config is built.
def instantiate(ckpt_dir=None):
    from m5 import options

    root = objects.Root.getInstance()

    if not root:
        fatal("Need to instantiate Root() before calling instantiate()")

    # we need to fix the global frequency
    ticks.fixGlobalFrequency()

    # Make sure SimObject-valued params are in the configuration
    # hierarchy so we catch them with future descendants() walks
    for obj in root.descendants(): obj.adoptOrphanParams()

    # Unproxy in sorted order for determinism
    for obj in root.descendants(): obj.unproxyParams()

    if options.dump_config:
        ini_file = file(os.path.join(options.outdir, options.dump_config), 'w')
        # Print ini sections in sorted order for easier diffing
        for obj in sorted(root.descendants(), key=lambda o: o.path()):
            obj.print_ini(ini_file)
        ini_file.close()

    if options.json_config:
        try:
            import json
            json_file = file(os.path.join(options.outdir, options.json_config), 'w')
            d = root.get_config_as_dict()
            json.dump(d, json_file, indent=4)
            json_file.close()
        except ImportError:
            pass

    do_dot(root, options.outdir, options.dot_config)

    # Initialize the global statistics
    stats.initSimStats()

    # Create the C++ sim objects and connect ports
    for obj in root.descendants(): obj.createCCObject()
    for obj in root.descendants(): obj.connectPorts()

    # Do a second pass to finish initializing the sim objects
    for obj in root.descendants(): obj.init()

    # Do a third pass to initialize statistics
    for obj in root.descendants(): obj.regStats()
    for obj in root.descendants(): obj.regFormulas()

    # We're done registering statistics.  Enable the stats package now.
    stats.enable()

    # Restore checkpoint (if any)
    if ckpt_dir:
        ckpt = internal.core.getCheckpoint(ckpt_dir)
        internal.core.unserializeGlobals(ckpt);
        for obj in root.descendants(): obj.loadState(ckpt)
        need_resume.append(root)
    else:
        for obj in root.descendants(): obj.initState()

    # Reset to put the stats in a consistent state.
    stats.reset()

need_resume = []
need_startup = True
def simulate(*args, **kwargs):
    global need_resume, need_startup

    if need_startup:
        root = objects.Root.getInstance()
        for obj in root.descendants(): obj.startup()
        need_startup = False

    for root in need_resume:
        resume(root)
    need_resume = []

    return internal.event.simulate(*args, **kwargs)

# Export curTick to user script.
def curTick():
    return internal.core.curTick()

# Python exit handlers happen in reverse order.  We want to dump stats last.
atexit.register(stats.dump)

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
    unready_objs = sum(obj.drain(drain_event) for obj in root.descendants())
    # If we've got some objects that can't drain immediately, then simulate
    if unready_objs > 0:
        drain_event.setCount(unready_objs)
        simulate()
    else:
        all_drained = True
    internal.event.cleanupCountedDrain(drain_event)
    return all_drained

def resume(root):
    for obj in root.descendants(): obj.resume()

def checkpoint(dir):
    root = objects.Root.getInstance()
    if not isinstance(root, objects.Root):
        raise TypeError, "Checkpoint must be called on a root object."
    doDrain(root)
    print "Writing checkpoint"
    internal.core.serializeAll(dir)
    resume(root)

def changeToAtomic(system):
    if not isinstance(system, (objects.Root, objects.System)):
        raise TypeError, "Parameter of type '%s'.  Must be type %s or %s." % \
              (type(system), objects.Root, objects.System)
    if system.getMemoryMode() != objects.params.atomic:
        doDrain(system)
        print "Changing memory mode to atomic"
        for obj in system.descendants():
            obj.changeTiming(objects.params.atomic)

def changeToTiming(system):
    if not isinstance(system, (objects.Root, objects.System)):
        raise TypeError, "Parameter of type '%s'.  Must be type %s or %s." % \
              (type(system), objects.Root, objects.System)

    if system.getMemoryMode() != objects.params.timing:
        doDrain(system)
        print "Changing memory mode to timing"
        for obj in system.descendants():
            obj.changeTiming(objects.params.timing)

def switchCpus(cpuList):
    print "switching cpus"
    if not isinstance(cpuList, list):
        raise RuntimeError, "Must pass a list to this function"
    for item in cpuList:
        if not isinstance(item, tuple) or len(item) != 2:
            raise RuntimeError, "List must have tuples of (oldCPU,newCPU)"

    for old_cpu, new_cpu in cpuList:
        if not isinstance(old_cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU" % old_cpu
        if not isinstance(new_cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU" % new_cpu

    # Now all of the CPUs are ready to be switched out
    for old_cpu, new_cpu in cpuList:
        old_cpu._ccObject.switchOut()

    for old_cpu, new_cpu in cpuList:
        new_cpu.takeOverFrom(old_cpu)

from internal.core import disableAllListeners
