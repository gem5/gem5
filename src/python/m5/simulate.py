# Copyright (c) 2012,2019 ARM Limited
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

from __future__ import print_function

import atexit
import os
import sys

# import the wrapped C++ functions
import _m5.drain
import _m5.core
from _m5.stats import updateEvents as updateStatEvents

from . import stats
from . import SimObject
from . import ticks
from . import objects
from m5.util.dot_writer import do_dot, do_dvfs_dot
from m5.util.dot_writer_ruby import do_ruby_dot

from .util import fatal
from .util import attrdict

# define a MaxTick parameter, unsigned 64 bit
MaxTick = 2**64 - 1

_memory_modes = {
    "atomic" : objects.params.atomic,
    "timing" : objects.params.timing,
    "atomic_noncaching" : objects.params.atomic_noncaching,
    }

_drain_manager = _m5.drain.DrainManager.instance()

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
        ini_file = open(os.path.join(options.outdir, options.dump_config), 'w')
        # Print ini sections in sorted order for easier diffing
        for obj in sorted(root.descendants(), key=lambda o: o.path()):
            obj.print_ini(ini_file)
        ini_file.close()

    if options.json_config:
        try:
            import json
            json_file = open(
                os.path.join(options.outdir, options.json_config), 'w')
            d = root.get_config_as_dict()
            json.dump(d, json_file, indent=4)
            json_file.close()
        except ImportError:
            pass

    if options.dot_config:
        do_dot(root, options.outdir, options.dot_config)
        do_ruby_dot(root, options.outdir, options.dot_config)

    # Initialize the global statistics
    stats.initSimStats()

    # Create the C++ sim objects and connect ports
    for obj in root.descendants(): obj.createCCObject()
    for obj in root.descendants(): obj.connectPorts()

    # Do a second pass to finish initializing the sim objects
    for obj in root.descendants(): obj.init()

    # Do a third pass to initialize statistics
    stats._bindStatHierarchy(root)
    root.regStats()

    # Do a fourth pass to initialize probe points
    for obj in root.descendants(): obj.regProbePoints()

    # Do a fifth pass to connect probe listeners
    for obj in root.descendants(): obj.regProbeListeners()

    # We want to generate the DVFS diagram for the system. This can only be
    # done once all of the CPP objects have been created and initialised so
    # that we are able to figure out which object belongs to which domain.
    if options.dot_dvfs_config:
        do_dvfs_dot(root, options.outdir, options.dot_dvfs_config)

    # We're done registering statistics.  Enable the stats package now.
    stats.enable()

    # Restore checkpoint (if any)
    if ckpt_dir:
        _drain_manager.preCheckpointRestore()
        ckpt = _m5.core.getCheckpoint(ckpt_dir)
        _m5.core.unserializeGlobals(ckpt);
        for obj in root.descendants(): obj.loadState(ckpt)
    else:
        for obj in root.descendants(): obj.initState()

    # Check to see if any of the stat events are in the past after resuming from
    # a checkpoint, If so, this call will shift them to be at a valid time.
    updateStatEvents()

need_startup = True
def simulate(*args, **kwargs):
    global need_startup

    if need_startup:
        root = objects.Root.getInstance()
        for obj in root.descendants(): obj.startup()
        need_startup = False

        # Python exit handlers happen in reverse order.
        # We want to dump stats last.
        atexit.register(stats.dump)

        # register our C++ exit callback function with Python
        atexit.register(_m5.core.doExitCleanup)

        # Reset to put the stats in a consistent state.
        stats.reset()

    if _drain_manager.isDrained():
        _drain_manager.resume()

    # We flush stdout and stderr before and after the simulation to ensure the
    # output arrive in order.
    sys.stdout.flush()
    sys.stderr.flush()
    sim_out = _m5.event.simulate(*args, **kwargs)
    sys.stdout.flush()
    sys.stderr.flush()

    return sim_out

def drain():
    """Drain the simulator in preparation of a checkpoint or memory mode
    switch.

    This operation is a no-op if the simulator is already in the
    Drained state.

    """

    # Try to drain all objects. Draining might not be completed unless
    # all objects return that they are drained on the first call. This
    # is because as objects drain they may cause other objects to no
    # longer be drained.
    def _drain():
        # Try to drain the system. The drain is successful if all
        # objects are done without simulation. We need to simulate
        # more if not.
        if _drain_manager.tryDrain():
            return True

        # WARNING: if a valid exit event occurs while draining, it
        # will not get returned to the user script
        exit_event = _m5.event.simulate()
        while exit_event.getCause() != 'Finished drain':
            exit_event = simulate()

        return False

    # Don't try to drain a system that is already drained
    is_drained = _drain_manager.isDrained()
    while not is_drained:
        is_drained = _drain()

    assert _drain_manager.isDrained(), "Drain state inconsistent"

def memWriteback(root):
    for obj in root.descendants():
        obj.memWriteback()

def memInvalidate(root):
    for obj in root.descendants():
        obj.memInvalidate()

def checkpoint(dir):
    root = objects.Root.getInstance()
    if not isinstance(root, objects.Root):
        raise TypeError("Checkpoint must be called on a root object.")

    drain()
    memWriteback(root)
    print("Writing checkpoint")
    _m5.core.serializeAll(dir)

def _changeMemoryMode(system, mode):
    if not isinstance(system, (objects.Root, objects.System)):
        raise TypeError("Parameter of type '%s'.  Must be type %s or %s." % \
              (type(system), objects.Root, objects.System))
    if system.getMemoryMode() != mode:
        system.setMemoryMode(mode)
    else:
        print("System already in target mode. Memory mode unchanged.")

def switchCpus(system, cpuList, verbose=True):
    """Switch CPUs in a system.

    Note: This method may switch the memory mode of the system if that
    is required by the CPUs. It may also flush all caches in the
    system.

    Arguments:
      system -- Simulated system.
      cpuList -- (old_cpu, new_cpu) tuples
    """

    if verbose:
        print("switching cpus")

    if not isinstance(cpuList, list):
        raise RuntimeError("Must pass a list to this function")
    for item in cpuList:
        if not isinstance(item, tuple) or len(item) != 2:
            raise RuntimeError("List must have tuples of (oldCPU,newCPU)")

    old_cpus = [old_cpu for old_cpu, new_cpu in cpuList]
    new_cpus = [new_cpu for old_cpu, new_cpu in cpuList]
    old_cpu_set = set(old_cpus)
    memory_mode_name = new_cpus[0].memory_mode()
    for old_cpu, new_cpu in cpuList:
        if not isinstance(old_cpu, objects.BaseCPU):
            raise TypeError("%s is not of type BaseCPU" % old_cpu)
        if not isinstance(new_cpu, objects.BaseCPU):
            raise TypeError("%s is not of type BaseCPU" % new_cpu)
        if new_cpu in old_cpu_set:
            raise RuntimeError(
                "New CPU (%s) is in the list of old CPUs." % (old_cpu,))
        if not new_cpu.switchedOut():
            raise RuntimeError("New CPU (%s) is already active." % (new_cpu,))
        if not new_cpu.support_take_over():
            raise RuntimeError(
                "New CPU (%s) does not support CPU handover." % (old_cpu,))
        if new_cpu.memory_mode() != memory_mode_name:
            raise RuntimeError(
                "%s and %s require different memory modes." % (new_cpu,
                                                               new_cpus[0]))
        if old_cpu.switchedOut():
            raise RuntimeError("Old CPU (%s) is inactive." % (new_cpu,))
        if not old_cpu.support_take_over():
            raise RuntimeError(
                "Old CPU (%s) does not support CPU handover." % (old_cpu,))

    try:
        memory_mode = _memory_modes[memory_mode_name]
    except KeyError:
        raise RuntimeError("Invalid memory mode (%s)" % memory_mode_name)

    drain()

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

def notifyFork(root):
    for obj in root.descendants():
        obj.notifyFork()

fork_count = 0
def fork(simout="%(parent)s.f%(fork_seq)i"):
    """Fork the simulator.

    This function forks the simulator. After forking the simulator,
    the child process gets its output files redirected to a new output
    directory. The default name of the output directory is the same as
    the parent with the suffix ".fN" added where N is the fork
    sequence number. The name of the output directory can be
    overridden using the simout keyword argument.

    Output file formatting dictionary:
      parent -- Path to the parent process's output directory.
      fork_seq -- Fork sequence number.
      pid -- PID of the child process.

    Keyword Arguments:
      simout -- New simulation output directory.

    Return Value:
      pid of the child process or 0 if running in the child.
    """
    from m5 import options
    global fork_count

    if not _m5.core.listenersDisabled():
        raise RuntimeError("Can not fork a simulator with listeners enabled")

    drain()

    try:
        pid = os.fork()
    except OSError as e:
        raise e

    if pid == 0:
        # In child, notify objects of the fork
        root = objects.Root.getInstance()
        notifyFork(root)
        # Setup a new output directory
        parent = options.outdir
        options.outdir = simout % {
                "parent" : parent,
                "fork_seq" : fork_count,
                "pid" : os.getpid(),
                }
        _m5.core.setOutputDir(options.outdir)
    else:
        fork_count += 1

    return pid

from _m5.core import disableAllListeners, listenersDisabled
from _m5.core import listenersLoopbackOnly
from _m5.core import curTick
