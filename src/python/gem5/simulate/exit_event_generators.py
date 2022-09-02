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

import m5.stats
from ..components.processors.abstract_processor import AbstractProcessor
from ..components.processors.switchable_processor import SwitchableProcessor
from m5.util import warn
from pathlib import Path

"""
In this package we store generators for simulation exit events.
"""


def defaultBehaviorWarning(type, effect):
    warn(
        "As no behavior was set by the user, default behavior is being carried"
        f" out.\n Type: {type} \n Detail: {effect} \n"
    )


def default_exit_generator():
    """
    A default generator for an exit event. It will return True, indicating that
    the Simulator run loop should exit.
    """
    defaultBehaviorWarning(
        "default_exit_generator",
        "A default generator for an exit event. It will return True, "
        "indicating that the Simulator run loop should exit.",
    )
    while True:
        yield True


def default_switch_generator(processor: AbstractProcessor):
    """
    A default generator for a switch exit event. If the processor is a
    SwitchableProcessor, this generator will switch it. Otherwise nothing will
    happen.
    """
    defaultBehaviorWarning(
        "default_switch_generator",
        "A default generator for a switch exit event.If the processor is a "
        "SwitchableProcessor, this generator will switch it. Otherwise nothing"
        " will happen.",
    )
    is_switchable = isinstance(processor, SwitchableProcessor)
    while True:
        if is_switchable:
            yield processor.switch()
        else:
            yield False


def default_workbegin_generator():
    """
    A default generator for a workbegin exit event. It will reset the
    simulation statistics.
    """
    defaultBehaviorWarning(
        "default_workbegin_generator",
        "A default generator for a workbegin exit event. It will reset the "
        "simulation statistics.",
    )
    while True:
        m5.stats.reset()
        yield False


def default_workend_generator():
    """
    A default generator for a workend exit event. It will dump the simulation
    statistics.
    """
    defaultBehaviorWarning(
        "default_workend_generator",
        "A default generator for a workend exit event. It will dump the "
        "simulation statistics.",
    )
    while True:
        m5.stats.dump()
        yield False


def default_simpoint_generator():
    """
    A default generator for SimPoints. It will do nothing.
    The Simulation run loop will continue after executing the behavior of the
    generator.
    """
    defaultBehaviorWarning(
        "default_simpoint_generator",
        "A default generator for SimPoints. It will do nothing.",
    )
    while True:
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


def save_checkpoint_generator(checkpoint_dir: Path):
    """
    A generator for taking a checkpoint. It will take a checkpoint with the
    input path and the current simulation Ticks.
    The Simulation run loop will continue after executing the behavior of the
    generator.
    """
    while True:
        m5.checkpoint((checkpoint_dir / f"cpt.{str(m5.curTick())}").as_posix())
        yield False
