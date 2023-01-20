# Copyright (c) 2021 The Regents of the University of California
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

from typing import Generator, Optional
import m5.stats
from ..components.processors.abstract_processor import AbstractProcessor
from ..components.processors.switchable_processor import SwitchableProcessor
from ..resources.resource import SimpointResource
from m5.util import warn
from pathlib import Path

"""
In this package we store generators for simulation exit events.
"""


def warn_default_decorator(gen: Generator, type: str, effect: str):
    """A decortator for generators which will print a warning that it is a
    default generator.
    """

    def wrapped_generator(*args, **kw_args):
        warn(
            f"No behavior was set by the user for {type}."
            f" Default behavior is {effect}."
        )
        for value in gen(*args, **kw_args):
            yield value

    return wrapped_generator


def exit_generator():
    """
    A default generator for an exit event. It will return True, indicating that
    the Simulator run loop should exit.
    """
    while True:
        yield True


def switch_generator(processor: AbstractProcessor):
    """
    A default generator for a switch exit event. If the processor is a
    SwitchableProcessor, this generator will switch it. Otherwise nothing will
    happen.
    """
    is_switchable = isinstance(processor, SwitchableProcessor)
    while True:
        if is_switchable:
            yield processor.switch()
        else:
            yield False


def dump_reset_generator():
    """
    A generator for doing statstic dump and reset. It will reset the simulation
    statistics and then dump simulation statistics.
    The Simulation run loop will continue after executing the behavior of the
    generator.
    """
    while True:
        m5.stats.dump()
        m5.stats.reset()
        yield False


def save_checkpoint_generator(checkpoint_dir: Optional[Path] = None):
    """
    A generator for taking a checkpoint. It will take a checkpoint with the
    input path and the current simulation Ticks.
    The Simulation run loop will continue after executing the behavior of the
    generator.
    """
    if not checkpoint_dir:
        from m5 import options

        checkpoint_dir = Path(options.outdir)
    while True:
        m5.checkpoint((checkpoint_dir / f"cpt.{str(m5.curTick())}").as_posix())
        yield False


def reset_stats_generator():
    """
    This generator resets the stats every time it is called. It does not dump
    the stats before resetting them.
    """
    while True:
        m5.stats.reset()
        yield False


def dump_stats_generator():
    """
    This generator dumps the stats every time it is called.
    """
    while True:
        m5.stats.dump()
        yield False


def skip_generator():
    """
    This generator does nothing when on the exit event.
    The simulation will continue after this generator.
    """
    while True:
        yield False


def simpoints_save_checkpoint_generator(
    checkpoint_dir: Path, simpoint: SimpointResource
):
    """
    A generator for taking multiple checkpoints for SimPoints. It will save the
    checkpoints in the checkpoint_dir path with the SimPoints' index.
    The Simulation run loop will continue after executing the behavior of the
    generator until all the SimPoints in the simpoint_list has taken a
    checkpoint.
    """
    simpoint_list = simpoint.get_simpoint_start_insts()
    count = 0
    last_start = -1
    while True:
        m5.checkpoint((checkpoint_dir / f"cpt.SimPoint{count}").as_posix())
        last_start = simpoint_list[count]
        count += 1
        # When the next SimPoint starting instruction is the same as the last
        # one, it will take a checkpoint for it with index+1. Because of there
        # are cases that the warmup length is larger than multiple SimPoints
        # starting instructions, then they might cause duplicates in the
        # simpoint_start_ints.
        while (
            count < len(simpoint_list) and last_start == simpoint_list[count]
        ):
            m5.checkpoint((checkpoint_dir / f"cpt.SimPoint{count}").as_posix())
            last_start = simpoint_list[count]
            count += 1
        # When there are remaining SimPoints in the list, let the Simulation
        # loop continues, otherwise, exit the Simulation loop.
        if count < len(simpoint_list):
            yield False
        else:
            yield True
