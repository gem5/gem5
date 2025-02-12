# Copyright (c) 2025  The Regents of the University of California
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

from abc import abstractmethod
from pathlib import Path
from typing import (
    Any,
    Dict,
    Optional,
)

import m5
from m5 import options
from m5.util import warn

from gem5.simulate.exit_event import ExitEvent
from gem5.utils.override import overrides


class ExitHandler:
    def __init__(self, payload: Dict[str, str]) -> None:
        self._payload = payload

    def handle(self, simulator: "Simulator") -> bool:
        self._process(simulator)
        return self._exit_simulation()

    def get_handler_description(self) -> str:
        return f"Exit Handler {self.__class__.__name__} called."

    @abstractmethod
    def _process(self, simulator: "Simulator") -> None:
        raise NotImplementedError(
            "Method '_process' must be implemented by a subclass"
        )

    @abstractmethod
    def _exit_simulation(self) -> bool:
        raise NotImplementedError(
            "Method '_exit_simulation' must be implemented by a subclass"
        )


class ClassicGeneratorExitHandler(ExitHandler):

    def __init__(self, payload: Dict[str, str]) -> None:
        super().__init__(payload)
        self._exit_on_completion = None

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        #  # Translate the exit event cause to the exit event enum.
        exit_enum = ExitEvent.translate_exit_status(
            simulator.get_last_exit_event_cause()
        )

        # Check to see the run is corresponding to the expected execution
        # order (assuming this check is demanded by the user).
        if simulator._expected_execution_order:
            expected_enum = simulator._expected_execution_order[
                simulator._exit_event_count
            ]
            if exit_enum.value != expected_enum.value:
                raise Exception(
                    f"Expected a '{expected_enum.value}' exit event but a "
                    f"'{exit_enum.value}' exit event was encountered."
                )

        # Record the current tick and exit event enum.
        simulator._tick_stopwatch.append(
            (exit_enum, simulator.get_current_tick())
        )

        try:
            # If the user has specified their own generator for this exit
            # event, use it.
            self._exit_on_completion = next(
                simulator._on_exit_event[exit_enum]
            )
        except StopIteration:
            # If the user's generator has ended, throw a warning and use
            # the default generator for this exit event.
            warn(
                "User-specified generator/function list for the exit "
                f"event'{exit_enum.value}' has ended. Using the default "
                "generator."
            )
            self._exit_on_completion = next(
                simulator._default_on_exit_dict[exit_enum]
            )

        except KeyError:
            # If the user has not specified their own generator for this
            # exit event, use the default.
            self._exit_on_completion = next(
                simulator._default_on_exit_dict[exit_enum]
            )

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        assert (
            self._exit_on_completion is not None
        ), "Exit on completion boolean var is not set."
        return self._exit_on_completion


class ScheduledExitEventHandler(ExitHandler):
    """A handler designed to be the default for  an Exit scheduled to occur
    at a specified tick. For example, these Exit exits can be triggered through be
    src/python/m5/simulate.py's `scheduleTickExitFromCurrent` and
    `scheduleTickExitAbsolute` functions.

    It will exit the simulation loop by default.

    The `justification` and `scheduled_at_tick` methods are provided to give
    richer information about this schedule exit.
    """

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    def justification(self) -> Optional[str]:
        """Returns the justification for the scheduled exit event.

        This information may be passed to when scheduling the event event to
        remind the user why the event was scheduled, or used in cases where
        lots of scheduled events are used and there is a need to differentiate
        them.

        Returns "None" if this information was not set.
        """
        return (
            None
            if "justification" not in self._payload
            else self._payload["justification"]
        )

    def scheduled_at_tick(self) -> Optional[int]:
        """Returns the tick in which the event was scheduled on (not scheduled
        to occur, but the tick the event was created).

        Returns "None" if this information is not available.
        """
        return (
            None
            if "scheduled_at_tick" not in self._payload
            else int(self._payload["scheduled_at_tick"])
        )

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


class KernelBootedExitHandler(ExitHandler):

    @overrides(ExitHandler)
    def get_handler_description(self):
        return "Kernel booted."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class AfterBootExitHandler(ExitHandler):

    @overrides(ExitHandler)
    def get_handler_description(self):
        return "Started `after_boot.sh` script."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class AfterBootScriptExitHandler(ExitHandler):

    @overrides(ExitHandler)
    def get_handler_description(self):
        return "Finished `after_boot.sh` script."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


class ToTickExitHandler(ExitHandler):
    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


class CheckpointExitHandler(ExitHandler):
    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        checkpoint_dir = simulator._checkpoint_path
        if not checkpoint_dir:
            checkpoint_dir = options.outdir
        m5.checkpoint(
            (Path(checkpoint_dir) / f"cpt.{str(m5.curTick())}").as_posix()
        )

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class WorkBeginExitHandler(ExitHandler):

    @overrides(ExitHandler)
    def get_handler_description(self):
        return "Started executing Region of Interest (ROI)."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.reset()

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class WorkEndExitHandler(ExitHandler):

    @overrides(ExitHandler)
    def get_handler_description(self):
        return "Finished executing Region of Interest (ROI)."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False
