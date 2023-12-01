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

from enum import Enum


class ExitEvent(Enum):
    """
    An enum class holding all the supported simulator exit events.

    The simulator will exit in certain conditions. The simulate package has
    been designed to categorize these into sensible states of exit, listed
    below.
    """

    EXIT = "exit"  # A standard vanilla exit.
    WORKBEGIN = "workbegin"  # An exit because a ROI has been reached.
    WORKEND = "workend"  # An exit because a ROI has ended.
    SWITCHCPU = "switchcpu"  # An exit needed to switch CPU cores.
    FAIL = "fail"  # An exit because the simulation has failed.
    CHECKPOINT = "checkpoint"  # An exit to load a checkpoint.
    SCHEDULED_TICK = "scheduled tick exit"
    MAX_TICK = "max tick"  # An exit due to a maximum tick value being met.
    USER_INTERRUPT = (  # An exit due to a user interrupt (e.g., cntr + c)
        "user interupt"
    )
    SIMPOINT_BEGIN = "simpoint begins"
    MAX_INSTS = "number of instructions reached"
    PERF_COUNTER_ENABLE = "performance counter enabled"
    PERF_COUNTER_DISABLE = "performance counter disabled"
    PERF_COUNTER_RESET = "performance counter reset"
    PERF_COUNTER_INTERRUPT = "performance counter interrupt"

    @classmethod
    def translate_exit_status(cls, exit_string: str) -> "ExitEvent":
        """
        This function will translate common exit strings to their correct
        ExitEvent categorization.

        .. note::

            At present, we do not guarantee this list is complete, as
            there are no bounds on what string may be returned by the simulator
            given an exit event.
        """

        if exit_string == "m5_workbegin instruction encountered":
            return ExitEvent.WORKBEGIN
        elif exit_string == "workbegin":
            return ExitEvent.WORKBEGIN
        elif exit_string == "m5_workend instruction encountered":
            return ExitEvent.WORKEND
        elif exit_string == "workend":
            return ExitEvent.WORKEND
        elif exit_string == "m5_exit instruction encountered":
            return ExitEvent.EXIT
        elif exit_string == "exiting with last active thread context":
            return ExitEvent.EXIT
        elif exit_string == "simulate() limit reached":
            return ExitEvent.MAX_TICK
        elif exit_string == "Tick exit reached":
            return ExitEvent.SCHEDULED_TICK
        elif exit_string == "switchcpu":
            return ExitEvent.SWITCHCPU
        elif exit_string == "m5_fail instruction encountered":
            return ExitEvent.FAIL
        elif exit_string == "checkpoint":
            return ExitEvent.CHECKPOINT
        elif exit_string == "user interrupt received":
            return ExitEvent.USER_INTERRUPT
        elif exit_string == "simpoint starting point found":
            return ExitEvent.SIMPOINT_BEGIN
        elif exit_string == "a thread reached the max instruction count":
            return ExitEvent.MAX_INSTS
        elif exit_string == "performance counter enabled":
            return ExitEvent.PERF_COUNTER_ENABLE
        elif exit_string == "performance counter disabled":
            return ExitEvent.PERF_COUNTER_DISABLE
        elif exit_string == "performance counter reset":
            return ExitEvent.PERF_COUNTER_RESET
        elif exit_string == "performance counter interrupt":
            return ExitEvent.PERF_COUNTER_INTERRUPT
        elif exit_string.endswith("will terminate the simulation.\n"):
            # This is for the traffic generator exit event
            return ExitEvent.EXIT
        elif exit_string.endswith("is finished updating the memory.\n"):
            # This is for the gups generator exit event
            return ExitEvent.EXIT
        raise NotImplementedError(
            f"Exit event '{exit_string}' not implemented"
        )
