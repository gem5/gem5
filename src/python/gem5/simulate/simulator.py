# Copyright (c) 2021-2024 The Regents of the University of California
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

import os
import sys
from pathlib import Path
from typing import (
    Callable,
    Dict,
    Generator,
    List,
    Optional,
    Tuple,
    Type,
    Union,
)

import m5
import m5.options
import m5.ticks
from m5.ext.pystats.simstat import SimStat
from m5.stats import addStatVisitor
from m5.util import warn

from gem5.components.boards.abstract_board import AbstractBoard

from ..resources.resource import WorkloadResource
from .exit_event import ExitEvent
from .exit_handler import (
    ClassicGeneratorExitHandler,
    ExitHandler,
)


class Simulator:
    """
    This Simulator class is used to manage the execution of a gem5 simulation.

    Example
    -------

    Examples using the Simulator class can be found under
    ``configs/example/gem5_library``.

    The most basic run would be as follows:

    .. code-block::

            simulator = Simulator(board=board)
             simulator.run()


    This will run a simulation and execute default behavior for exit events.
    """

    # Here we declare the modules which should not be imported into any gem5
    # standard library run. The key is the module (e.g,
    # "import common.Options") and the value is the reason, which will be
    # output in the case this module is imported.
    # This is checked with the `run` function is executed.
    _banned_modules = {
        "common.Options": "The options provided by 'Options' are not "
        "compatible with the gem5 standard library.",
    }

    def __init__(
        self,
        board: AbstractBoard,
        full_system: Optional[bool] = None,
        on_exit_event: Optional[
            Dict[
                ExitEvent,
                Union[
                    Generator[Optional[bool], None, None],
                    List[Callable],
                    Callable,
                ],
            ]
        ] = None,
        expected_execution_order: Optional[List[ExitEvent]] = None,
        checkpoint_path: Optional[Path] = None,
        max_ticks: Optional[int] = m5.MaxTick,
        id: Optional[int] = None,
    ) -> None:
        """
        :param board: The board to be simulated.
        :param full_system: Whether to run as a full-system simulation or not.
                            This is optional and used to override default
                            behavior. If not set, whether or not to run in FS
                            mode will be determined via the board's
                            ``is_fullsystem()`` function.
                            **Warning: This parameter is deprecated. The board
                            determines if the simulation is full system or not.
                            This parameter will be removed in a future gem5
                            release.**
        :param on_exit_event: An optional map to specify what to execute on
                              each exit event. There are three possibilities here:
                              a generator, a list of functions, or a single function.

                                       1. Generator: The generator may yield a boolean each
                                         time the associated exit event is encountered. If
                                         ``True`` the simulator will exit the simulation loop.

                              2. List of functions: Each function must be callable
                                with no mandatory arguments and return a boolean
                                specifying if the Simulation should exit the
                                simulation loop. Upon each exit event the list will
                                pop the start of the list and execute it. If the list
                                is empty the default behavior for that exit event will
                                be executed.

                              3. Single function: The function must be callable with
                                no mandatory arguments and return a boolean specifying
                                if the Simulation should exit or not. This function is
                                executed each time the associated exit event is encountered.

                                See `ClassicGeneratorExitHandler` for more details
        :param checkpoint_path: An optional parameter specifying the directory of
                                the checkpoint to instantiate from. When the path
                                is ``None``, no checkpoint will be loaded. By default,
                                the path is ``None``. **This parameter is deprecated.
                                Please set the checkpoint when setting the board's
                                workload**.
        :param max_ticks: The maximum number of ticks to execute  in the
                          simulation run before exiting with a ``MAX_TICK``
                          exit event. If not set this value is to `m5.MaxTick`,
                          the last value allowed in the tick variable. At
                          present this is an unsigned 64-bit integer, and
                          herefore is set to 2^4-1. Prior to intialization,
                          max tickks can also be set via the `set_max_ticks`
                          function.
        :param id: An optional parameter specifying the ID of the simulation.
        This is particularly useful when running muliple simuations in
        parallel. The ID can be unique and descriptive of the simulation. If
        not set, the ID will be a hash of the instantiated system and
        Simulator configuration. Note, the latter means the ID only available
        after the Simulator has been instantiated. The ID can be obtained via
        the `get_id` method.
        :param exit_event_handler_id_map: An optional parameter specifying the
        mapping each exit event IDs the Exit Event handler class responsible
        for handling them. The Simulator provides sensible defaults for stdlib
        exit events, but this parameter allows the user to override these
        or add handlers for custom exit events. Use
        `ExitHandler.get_handler_map` to see the mapping.

        See ClassicGeneratorExitHandler for details on
        """

        if full_system is not None:
            warn(
                "Setting the full_system parameter via the Simulator "
                "constructor is deprecated and will be removed in future "
                "releases of gem5. "
                "The board determines if the simulation is full system or not "
                "via it's `is_fullsystem` method."
            )

        self.set_max_ticks(max_ticks)

        if id:
            self.set_id(id)

        self._instantiated = False
        self._board = board
        self._full_system = full_system
        self._tick_stopwatch = []

        self._last_exit_event = None
        self._exit_event_count = 0

        if checkpoint_path:
            warn(
                "Setting the checkpoint path via the Simulator constructor is "
                "deprecated and will be removed in future releases of gem5. "
                "Please set this through via the appropriate workload "
                "function (i.e., `set_se_binary_workload` or "
                "`set_kernel_disk_workload`). If both are set the workload "
                "function set takes precedence."
            )

        self._checkpoint_path = checkpoint_path

        # Set up the classic event generators.
        ClassicGeneratorExitHandler.set_exit_event_map(
            on_exit_event, expected_execution_order, board
        )

        # A simple mapping of ticks to exit event.
        # This can help in cases where the order and number of exits thus far
        # matters (say an exit event acts differently for the Nth time it is
        # hit)
        self._exit_event_id_log = {}

    def switch_processor(self) -> None:
        """
        Switch the processor. This is a convenience function to call the
        processor's switch function.
        """
        self._board.get_processor().switch()

    def get_exit_handler_id_map(self) -> Dict[int, Type[ExitHandler]]:
        """
        Returns the exit handler ID map. This is a dictionary mapping exit
        event IDs to the ExitEvent handler class responsible for handling them.
        """
        return ExitHandler.get_handler_map()

    def set_id(self, id: str) -> None:
        """Set the ID of the simulator.

        As, in the caae of multisim, this ID will be used to create an
        output subdirectory, there needs to be rules on what an ID can be.
        For now, this function encoures that IDs can only be alphanumeric
        characters with underscores  and dashes. Uunderscores and dashes cannot
        be at the start or end of the ID and  the ID must start with at least
        one letter.

        :param id: The ID of the simulator.
        """

        if not id:
            raise ValueError("ID cannot be an empty string.")

        if not id[0].isalpha():
            raise ValueError("ID must start with a letter.")

        if not id[-1].isalnum():
            raise ValueError(
                "ID must end with a alphanumeric value (a digit "
                "or a character)."
            )

        if not all(char.isalnum() or char in ["_", "-"] for char in id):
            raise ValueError(
                "ID can only contain alphanumeric characters, "
                "underscores, and dashes."
            )
        self._id = id

    def get_id(self) -> Optional[str]:
        """
        Returns the ID of the simulation. This is particularly useful when
        running multiple simulations in parallel. The ID can be unique and
        descriptive of the simulation. It is set via the contructor or the
        `set_id` function. None if not set by either.
        """

        if hasattr(self, "_id") and self._id:
            return self._id

        return None

    def set_max_ticks(self, max_tick: int) -> None:
        """Set the absolute (not relative) maximum number of ticks to run the
        simulation for. This is the maximum number of ticks to run the
        simulation for before exiting with a ``MAX_TICK`` exit event.
        """
        if max_tick > m5.MaxTick:
            raise ValueError(
                f"Max ticks must be less than {m5.MaxTick}, not {max_tick}"
            )
        self._max_ticks = max_tick

    def get_max_ticks(self) -> int:
        assert hasattr(self, "_max_ticks"), "Max ticks not set"
        return self._max_ticks

    def schedule_simpoint(self, simpoint_start_insts: List[int]) -> None:
        """
        Schedule ``SIMPOINT_BEGIN`` exit events

        .. warning::

            SimPoints only work with one core.

        :param simpoint_start_insts: A list of number of instructions
                                    indicating the starting point of
                                    the SimPoints.
        """
        if self._board.get_processor().get_num_cores() > 1:
            warn("SimPoints only work with one core")
        self._board.get_processor().get_cores()[0].set_simpoint(
            simpoint_start_insts, self._instantiated
        )

    def schedule_max_insts(self, inst: int) -> None:
        """
        Schedule a ``MAX_INSTS`` exit event when any thread in any core
        reaches the given number of instructions.

        :param insts: A number of instructions to run to.
        """
        for core in self._board.get_processor().get_cores():
            core._set_inst_stop_any_thread(inst, self._instantiated)

    def get_instruction_count(self) -> int:
        """
        Returns the number of instructions executed by all cores.

        Note: This total is the sum since the last call to reset stats.
        """
        return self._board.get_processor().get_total_instructions()

    def get_workload(self) -> WorkloadResource:
        """
        Returns the workload of the board.
        """
        return self._board.get_workload()

    def get_stats(self) -> Dict:
        """
        Obtain the current simulation statistics as a Dictionary, conforming
        to a JSON-style schema.

        :raises Exception: An exception is raised if this function is called
                           before ``run()``. The board must be initialized
                           before obtaining statistics.
        """

        return self.get_simstats().to_json()

    def get_simstats(self) -> SimStat:
        """
        Obtains the `SimStat` of the current simulation.

        :raises Exception: An exception is raised if this function is called
                           before ``run()``. The board must be initialized
                           before obtaining statistics.
        """

        if not self._instantiated:
            raise Exception(
                "Cannot obtain simulation statistics prior to initialization."
            )

        return m5.stats.gem5stats.get_simstat(self._root)

    def add_text_stats_output(self, path: str) -> None:
        """
        This function is used to set an output location for text stats. If
        specified, when stats are dumped they will be output to this location
        as a text file file, in addition to any other stats' output locations
        specified.

        :param path: That path in which the file should be output to.
        """
        path_path = Path(path)
        parent = path_path.parent

        if (
            not parent.is_dir()
            or not os.access(parent, os.W_OK)
            or (
                path_path.exists()
                and (path_path.is_dir() or not os.access(path_path, os.W_OK))
            )
        ):
            raise Exception(
                f"Specified text stats output path '{path}' is invalid."
            )
        addStatVisitor(path)

    def add_json_stats_output(self, path: str) -> None:
        """
        This function is used to set an output location for JSON. If specified,
        when stats are dumped they will be output to this location as a JSON
        file, in addition to any other stats' output locations specified.

        :param path: That path in which the JSON should be output to.
        """
        path_path = Path(path)
        parent = path_path.parent

        if (
            not parent.is_dir()
            or not os.access(parent, os.W_OK)
            or (
                path_path.exists()
                and (path_path.is_dir() or not os.access(path_path, os.W_OK))
            )
        ):
            raise Exception(
                f"Specified json stats output path '{path}' is invalid."
            )
        addStatVisitor(f"json://{path}")

    def get_last_exit_event_cause(self) -> str:
        """
        Returns the last exit event cause.
        """
        return self._last_exit_event.getCause()

    def get_last_exit_event_code(self) -> int:
        """
        Returns the last exit event status code
        """
        return self._last_exit_event.getCode()

    def get_hypercall_id(self) -> int:
        """
        Returns the hypercall ID.
        """
        return self._last_exit_event.getHypercallId()

    def get_current_tick(self) -> int:
        """
        Returns the current tick.
        """
        return m5.curTick()

    def get_tick_stopwatch(self) -> List[Tuple[ExitEvent, int]]:
        """
        Returns a list of tuples, which each tuple specifying an exit event
        and the ticks at that event.
        """
        return self._tick_stopwatch

    def get_roi_ticks(self) -> List[int]:
        """
        Returns a list of the tick counts for every ROI encountered (specified
        as a region of code between a Workbegin and Workend exit event).
        """
        start = 0
        to_return = []
        for exit_event, tick in self._tick_stopwatch:
            if exit_event == ExitEvent.WORKBEGIN:
                start = tick
            elif exit_event == ExitEvent.WORKEND:
                to_return.append(tick - start)

        return to_return

    def get_exit_event_id_log(self) -> Dict[int, str]:
        """
        Returns a dictionary mapping tick at which an exit event was encountered
        to the exit event description.
        """
        return self._exit_event_id_log

    def show_exit_event_messages(self) -> None:
        """
        Show exit event messages. This will print the exit event messages to
        the console.
        """
        m5.options.show_exit_event_messages = True

    def override_outdir(self, new_outdir: Path) -> None:
        """This function can be used to override the output directory locatiomn
        Assiming the path passed is valid, the directory will be created
        and set as the new output directory, thus overriding what was set at
        the gem5 command line. Is there fore advised this function is used with
        caution. Its primary use is for swaning multiple gem5 processes from
        a gem5 process to allow the child processes their own output directory.

        :param new_outdir: The new output directory to be used instead of that
                           set at the gem5 command line.
        """

        if self._instantiated:
            raise Exception(
                "Cannot override the output directory after the simulation "
                "has been instantiated."
            )
        from m5 import options

        from _m5.core import setOutputDir

        new_outdir.mkdir(parents=True, exist_ok=True)

        if not new_outdir.exists():
            raise Exception(f"Directory '{new_outdir}' does not exist")

        if not new_outdir.is_dir():
            raise Exception(f"'{new_outdir}' is not a directory")

        options.outdir = str(new_outdir)
        setOutputDir(options.outdir)

    def _instantiate(self) -> None:
        """
        This method will instantiate the board and carry out necessary
        boilerplate code before the instantiation such as setting up root and
        setting the sim_quantum (if running in KVM mode).
        """

        if not self._instantiated:
            # Before anything else we run the AbstractBoard's
            # `_pre_instantiate` function. This returns the root object which
            # is required for instantiation.
            self._root = self._board._pre_instantiate(
                full_system=self._full_system
            )

            # m5.instantiate() takes a parameter specifying the path to the
            # checkpoint directory. If the parameter is None, no checkpoint
            # will be restored.
            if self._board._checkpoint:
                m5.instantiate(self._board._checkpoint.as_posix())
            else:
                m5.instantiate(self._checkpoint_path)
            self._instantiated = True

            # Let the board know that instantiate has been called so it can do
            # any final things.
            self._board._post_instantiate()

    def run(self, max_ticks: Optional[int] = None) -> None:
        """
        This function will start or continue the simulator run and handle exit
        events accordingly.

        :param max_ticks: The maximum number of ticks to execute per simulation
                          run. If this ``max_ticks`` value is met, a ``MAX_TICK``
                          exit event is received, if another simulation exit
                          event is met the tick count is reset. This is the
                          **maximum number of ticks per simulation run.
        """

        if max_ticks and max_ticks != self._max_ticks:
            warn(
                "Max ticks has already been set prior to setting it through "
                "the run call. In these cases the max ticks set through the "
                "`run` function is used"
            )
            self.set_max_ticks(max_ticks)

        # Check to ensure no banned module has been imported.
        for banned_module in self._banned_modules.keys():
            if banned_module in sys.modules:
                raise Exception(
                    f"The banned module '{banned_module}' has been included. "
                    "Please do not use this in your simulations. "
                    f"Reason: {self._banned_modules[banned_module]}"
                )

        # We instantiate the board if it has not already been instantiated.
        self._instantiate()

        # This while loop will continue until an a generator yields True.
        while True:
            self._last_exit_event = m5.simulate(self.get_max_ticks())
            exit_event_hypercall_id = self._last_exit_event.getHypercallId()
            if (
                exit_event_hypercall_id
                not in self.get_exit_handler_id_map().keys()
            ):
                warn(
                    f"Warning: Exit event type ID "
                    f"{self._last_exit_event.getHypercallId()} "
                    f"not in exit handler ID map. Reentering simulation loop."
                )
                continue
            exit_handler = self.get_exit_handler_id_map()[
                exit_event_hypercall_id
            ](self._last_exit_event.getPayload())

            if m5.options.show_exit_event_messages:
                print(
                    f"Exit event: {exit_handler.get_handler_description()} called at tick {self.get_current_tick()}"
                )

            exit_on_completion = exit_handler.handle(self)
            self._exit_event_id_log[self.get_current_tick()] = (
                exit_handler.get_handler_description()
            )

            # If the generator returned True we will return from the Simulator
            # run loop. In the case of a function: if it returned True.
            if exit_on_completion:
                return

    def save_checkpoint(self, checkpoint_dir: Path) -> None:
        """
        This function will save the checkpoint to the specified directory.

        :param checkpoint_dir: The path to the directory where the checkpoint
                               will be saved.
        """
        m5.checkpoint(str(checkpoint_dir))
