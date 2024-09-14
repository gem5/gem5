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

import os
import sys
from io import StringIO
from pathlib import Path
from typing import (
    Callable,
    Dict,
    Generator,
    List,
    Optional,
    Tuple,
    Union,
)

import m5
import m5.ticks
from m5.ext.pystats.simstat import SimStat
from m5.objects import Root
from m5.stats import addStatVisitor
from m5.util import warn

from ..components.boards.abstract_board import AbstractBoard
from ..components.processors.switchable_processor import SwitchableProcessor
from .exit_event import ExitEvent
from .exit_event_generators import (
    dump_stats_generator,
    exit_generator,
    reset_stats_generator,
    save_checkpoint_generator,
    skip_generator,
    spatter_exit_generator,
    switch_generator,
    warn_default_decorator,
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


        ``on_exit_event`` usage notes
        ---------------------------

        With Generators
        ===============

        The ``on_exit_event`` parameter specifies a Python generator for each
        exit event. `next(<generator>)` is run each time an exit event. The
        generator may yield a boolean. If this value of this boolean is ``True``
        the Simulator run loop will exit, otherwise
        the Simulator run loop will continue execution. If the generator has
        finished (i.e. a ``StopIteration`` exception is thrown when
        ``next(<generator>)`` is executed), then the default behavior for that
        exit event is run.

        As an example, a user may specify their own exit event setup like so:

        .. code-block::

            def unique_exit_event():
                processor.switch()
                yield False
                m5.stats.dump()
                yield False
                yield True

            simulator = Simulator(
                board=board
                on_exit_event = {
                    ExitEvent.Exit : unique_exit_event(),
                },
            )


        This will execute ``processor.switch()`` the first time an exit event is
        encountered, will dump gem5 statistics the second time an exit event is
        encountered, and will terminate the Simulator run loop the third time.

        With a list of functions
        ========================

        Alternatively, instead of passing a generator per exit event, a list of
        functions may be passed. Each function must take no mandatory arguments
        and return True if the simulator is to exit after being called.

        An example:

        .. code-block::

            def stop_simulation() -> bool:
                return True

            def switch_cpus() -> bool:
                processor.switch()
                return False

            def print_hello() -> None:
                # Here we don't explicitly return a boolean, but the simulator
                # treats a None return as False. Ergo the Simulation loop is not
                # terminated.
                print("Hello")


            simulator = Simulator(
                board=board,
                on_exit_event = {
                    ExitEvent.Exit : [
                        print_hello,
                        switch_cpus,
                        print_hello,
                        stop_simulation
                    ],
                },
            )


        Upon each ``EXIT`` type exit event the list will function as a queue,
        with the top function of the list popped and executed. Therefore, in
        this example, the first ``EXIT`` type exit event will cause ``print_hello``
        to be executed, and the second ``EXIT`` type exit event will cause the
        ``switch_cpus`` function to run. The third will execute ``print_hello``
        again before finally, on the forth exit event will call
        ``stop_simulation`` which will stop the simulation as it returns ``False``.

        With a function
        ===============
        A single function can be passed. In this case every exit event of that
        type will execute that function every time. The function should not
        accept any mandatory parameters and return a boolean specifying if the
        simulation loop should end after it is executed.
        An example:

        .. code-block::

            def print_hello() -> bool:
                print("Hello")
                return False
            simulator = Simulator(
                board=board,
                on_exit_event = {
                    ExitEvent.Exit : print_hello
                },
            )

        The above will print "Hello" on every ``Exit`` type Exit Event. As the
        function returns False, the simulation loop will not end on these
        events.


        Exit Event defaults
        ===================

        Each exit event has a default behavior if none is specified by the
        user. These are as follows:

            * ExitEvent.EXIT:  exit simulation
            * ExitEvent.CHECKPOINT: take a checkpoint
            * ExitEvent.FAIL : exit simulation
            * ExitEvent.SWITCHCPU: call ``switch`` on the processor
            * ExitEvent.WORKBEGIN: reset stats
            * ExitEvent.WORKEND: exit simulation
            * ExitEvent.USER_INTERRUPT: exit simulation
            * ExitEvent.MAX_TICK: exit simulation
            * ExitEvent.SCHEDULED_TICK: exit simulation
            * ExitEvent.SIMPOINT_BEGIN: reset stats
            * ExitEvent.MAX_INSTS: exit simulation

        These generators can be found in the ``exit_event_generator.py`` module.

        """

        self.set_max_ticks(max_ticks)

        if id:
            self.set_id(id)

        # We specify a dictionary here outlining the default behavior for each
        # exit event. Each exit event is mapped to a generator.
        self._default_on_exit_dict = {
            ExitEvent.EXIT: exit_generator(),
            ExitEvent.CHECKPOINT: warn_default_decorator(
                save_checkpoint_generator,
                "checkpoint",
                "creating a checkpoint and continuing",
            )(),
            ExitEvent.FAIL: exit_generator(),
            ExitEvent.SPATTER_EXIT: warn_default_decorator(
                spatter_exit_generator,
                "spatter exit",
                "dumping and resetting stats after each sync point. "
                "Note that there will be num_cores*sync_points spatter_exits.",
            )(spatter_gen=board.get_processor()),
            ExitEvent.SWITCHCPU: warn_default_decorator(
                switch_generator,
                "switch CPU",
                "switching the CPU type of the processor and continuing",
            )(processor=board.get_processor()),
            ExitEvent.WORKBEGIN: warn_default_decorator(
                reset_stats_generator,
                "work begin",
                "resetting the stats and continuing",
            )(),
            ExitEvent.WORKEND: warn_default_decorator(
                dump_stats_generator,
                "work end",
                "dumping the stats and continuing",
            )(),
            ExitEvent.USER_INTERRUPT: exit_generator(),
            ExitEvent.MAX_TICK: exit_generator(),
            ExitEvent.SCHEDULED_TICK: exit_generator(),
            ExitEvent.SIMPOINT_BEGIN: warn_default_decorator(
                skip_generator,
                "simpoint begin",
                "resetting the stats and continuing",
            )(),
            ExitEvent.MAX_INSTS: warn_default_decorator(
                exit_generator,
                "max instructions",
                "exiting the simulation",
            )(),
            ExitEvent.KERNEL_PANIC: exit_generator(),
            ExitEvent.KERNEL_OOPS: exit_generator(),
        }

        if on_exit_event:
            self._on_exit_event = {}
            for key, value in on_exit_event.items():
                if isinstance(value, Generator):
                    self._on_exit_event[key] = value
                elif isinstance(value, List):
                    # In instances where we have a list of functions, we
                    # convert this to a generator.
                    self._on_exit_event[key] = (func() for func in value)
                elif isinstance(value, Callable):
                    # In instances where the user passes a lone function, the
                    # function is called on every exit event of that type. Here
                    # we convert the function into an infinite generator.

                    # We check if the function is a generator. If it is we
                    # throw a warning as this is likely a mistake.
                    import inspect

                    if inspect.isgeneratorfunction(value):
                        warn(
                            f"Function passed for '{key.value}' exit event "
                            "is not a generator but a function that returns "
                            "a generator. Did you mean to do this? (e.g., "
                            "did you mean `ExitEvent.EVENT : gen()` instead "
                            "of `ExitEvent.EVENT : gen`)"
                        )

                    def function_generator(func: Callable):
                        while True:
                            yield func()

                    self._on_exit_event[key] = function_generator(func=value)
                else:
                    raise Exception(
                        f"`on_exit_event` for '{key.value}' event is "
                        "not a Generator or List[Callable]."
                    )
        else:
            self._on_exit_event = self._default_on_exit_dict

        self._instantiated = False
        self._board = board
        self._full_system = full_system
        self._expected_execution_order = expected_execution_order
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
            # `_pre_instantiate` function.
            self._board._pre_instantiate()

            root = Root(
                full_system=(
                    self._full_system
                    if self._full_system is not None
                    else self._board.is_fullsystem()
                ),
                board=self._board,
            )

            # We take a copy of the Root in case it's required elsewhere
            # (for example, in `get_stats()`).
            self._root = root

            # The following is a bit of a hack. If a simulation is to use a KVM
            # core then the `sim_quantum` value must be set. However, in the
            # case of using a SwitchableProcessor the KVM cores may be
            # switched out and therefore not accessible via `get_cores()`.
            # This is the reason for the `isinstance` check.
            #
            # We cannot set the `sim_quantum` value in every simulation as
            # setting it causes the scheduling of exits to be off by the
            # `sim_quantum` value (something necessary if we are using KVM
            # cores). Ergo we only set the value of KVM cores are present.
            #
            # There is still a bug here in that if the user is switching to and
            # from KVM and non-KVM cores via the SwitchableProcessor then the
            # scheduling of exits for the non-KVM cores will be incorrect. This
            # will be fixed at a later date.
            processor = self._board.processor
            if any(core.is_kvm_core() for core in processor.get_cores()) or (
                isinstance(processor, SwitchableProcessor)
                and any(core.is_kvm_core() for core in processor._all_cores())
            ):
                m5.ticks.fixGlobalFrequency()
                root.sim_quantum = m5.ticks.fromSeconds(0.001)

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

            # Translate the exit event cause to the exit event enum.
            exit_enum = ExitEvent.translate_exit_status(
                self.get_last_exit_event_cause()
            )

            # Check to see the run is corresponding to the expected execution
            # order (assuming this check is demanded by the user).
            if self._expected_execution_order:
                expected_enum = self._expected_execution_order[
                    self._exit_event_count
                ]
                if exit_enum.value != expected_enum.value:
                    raise Exception(
                        f"Expected a '{expected_enum.value}' exit event but a "
                        f"'{exit_enum.value}' exit event was encountered."
                    )

            # Record the current tick and exit event enum.
            self._tick_stopwatch.append((exit_enum, self.get_current_tick()))

            try:
                # If the user has specified their own generator for this exit
                # event, use it.
                exit_on_completion = next(self._on_exit_event[exit_enum])
            except StopIteration:
                # If the user's generator has ended, throw a warning and use
                # the default generator for this exit event.
                warn(
                    "User-specified generator/function list for the exit "
                    f"event'{exit_enum.value}' has ended. Using the default "
                    "generator."
                )
                exit_on_completion = next(
                    self._default_on_exit_dict[exit_enum]
                )
            except KeyError:
                # If the user has not specified their own generator for this
                # exit event, use the default.
                exit_on_completion = next(
                    self._default_on_exit_dict[exit_enum]
                )

            self._exit_event_count += 1

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
