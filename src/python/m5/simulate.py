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
from m5.internal.stats import updateEvents as updateStatEvents

from util import fatal
from util import attrdict

# define a MaxTick parameter, unsigned 64 bit
MaxTick = 2**64 - 1

_memory_modes = {
    "atomic" : objects.params.atomic,
    "timing" : objects.params.timing,
    "atomic_noncaching" : objects.params.atomic_noncaching,
    }

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

    # Do a fourth pass to initialize probe points
    for obj in root.descendants(): obj.regProbePoints()

    # Do a fifth pass to connect probe listeners
    for obj in root.descendants(): obj.regProbeListeners()

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

    # Check to see if any of the stat events are in the past after resuming from
    # a checkpoint, If so, this call will shift them to be at a valid time.
    updateStatEvents()

need_resume = []
need_startup = True
def simulate(*args, **kwargs):
    global need_resume, need_startup

    if need_startup:
        root = objects.Root.getInstance()
        for obj in root.descendants(): obj.startup()
        need_startup = False

        # Python exit handlers happen in reverse order.
        # We want to dump stats last.
        atexit.register(stats.dump)

        # register our C++ exit callback function with Python
        atexit.register(internal.core.doExitCleanup)

        # Reset to put the stats in a consistent state.
        stats.reset()

    for root in need_resume:
        resume(root)
    need_resume = []

    return internal.event.simulate(*args, **kwargs)

# Export curTick to user script.
def curTick():
    return internal.core.curTick()

# Drain the system in preparation of a checkpoint or memory mode
# switch.
def drain(root):
    # Try to drain all objects. Draining might not be completed unless
    # all objects return that they are drained on the first call. This
    # is because as objects drain they may cause other objects to no
    # longer be drained.
    def _drain():
        all_drained = False
        dm = internal.drain.createDrainManager()
        unready_objs = sum(obj.drain(dm) for obj in root.descendants())
        # If we've got some objects that can't drain immediately, then simulate
        if unready_objs > 0:
            dm.setCount(unready_objs)
            simulate()
        else:
            all_drained = True
        internal.drain.cleanupDrainManager(dm)
        return all_drained

    all_drained = _drain()
    while (not all_drained):
        all_drained = _drain()

def memWriteback(root):
    for obj in root.descendants():
        obj.memWriteback()

def memInvalidate(root):
    for obj in root.descendants():
        obj.memInvalidate()

def resume(root):
    for obj in root.descendants(): obj.drainResume()

def checkpoint(dir):
    root = objects.Root.getInstance()
    if not isinstance(root, objects.Root):
        raise TypeError, "Checkpoint must be called on a root object."
    drain(root)
    memWriteback(root)
    print "Writing checkpoint"
    internal.core.serializeAll(dir)
    resume(root)

def _changeMemoryMode(system, mode):
    if not isinstance(system, (objects.Root, objects.System)):
        raise TypeError, "Parameter of type '%s'.  Must be type %s or %s." % \
              (type(system), objects.Root, objects.System)
    if system.getMemoryMode() != mode:
        drain(system)
        system.setMemoryMode(mode)
    else:
        print "System already in target mode. Memory mode unchanged."

def switchCpus(system, cpuList, do_drain=True, verbose=True):
    """Switch CPUs in a system.

    By default, this method drains and resumes the system. This
    behavior can be disabled by setting the keyword argument
    'do_drain' to false, which might be desirable if multiple
    operations requiring a drained system are going to be performed in
    sequence.

    Note: This method may switch the memory mode of the system if that
    is required by the CPUs. It may also flush all caches in the
    system.

    Arguments:
      system -- Simulated system.
      cpuList -- (old_cpu, new_cpu) tuples

    Keyword Arguments:
      do_drain -- Perform a drain/resume of the system when switching.
    """

    if verbose:
        print "switching cpus"

    if not isinstance(cpuList, list):
        raise RuntimeError, "Must pass a list to this function"
    for item in cpuList:
        if not isinstance(item, tuple) or len(item) != 2:
            raise RuntimeError, "List must have tuples of (oldCPU,newCPU)"

    old_cpus = [old_cpu for old_cpu, new_cpu in cpuList]
    new_cpus = [new_cpu for old_cpu, new_cpu in cpuList]
    old_cpu_set = set(old_cpus)
    memory_mode_name = new_cpus[0].memory_mode()
    for old_cpu, new_cpu in cpuList:
        if not isinstance(old_cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU" % old_cpu
        if not isinstance(new_cpu, objects.BaseCPU):
            raise TypeError, "%s is not of type BaseCPU" % new_cpu
        if new_cpu in old_cpu_set:
            raise RuntimeError, \
                "New CPU (%s) is in the list of old CPUs." % (old_cpu,)
        if not new_cpu.switchedOut():
            raise RuntimeError, \
                "New CPU (%s) is already active." % (new_cpu,)
        if not new_cpu.support_take_over():
            raise RuntimeError, \
                "New CPU (%s) does not support CPU handover." % (old_cpu,)
        if new_cpu.memory_mode() != memory_mode_name:
            raise RuntimeError, \
                "%s and %s require different memory modes." % (new_cpu,
                                                               new_cpus[0])
        if old_cpu.switchedOut():
            raise RuntimeError, \
                "Old CPU (%s) is inactive." % (new_cpu,)
        if not old_cpu.support_take_over():
            raise RuntimeError, \
                "Old CPU (%s) does not support CPU handover." % (old_cpu,)

    try:
        memory_mode = _memory_modes[memory_mode_name]
    except KeyError:
        raise RuntimeError, "Invalid memory mode (%s)" % memory_mode_name

    if do_drain:
        drain(system)

    # Now all of the CPUs are ready to be switched out
    for old_cpu, new_cpu in cpuList:
        old_cpu.switchOut()

    # Change the memory mode if required. We check if this is needed
    # to avoid printing a warning if no switch was performed.
    if system.getMemoryMode() != memory_mode:
        # Flush the memory system if we are switching to a memory mode
        # that disables caches. This typically happens when switching to a
        # hardware virtualized CPU.
        if memory_mode == objects.params.atomic_noncaching:
            memWriteback(system)
            memInvalidate(system)

        _changeMemoryMode(system, memory_mode)

    for old_cpu, new_cpu in cpuList:
        new_cpu.takeOverFrom(old_cpu)

    if do_drain:
        resume(system)

from internal.core import disableAllListeners
