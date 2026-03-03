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

"""Exit handler helpers for the gem5 Python stdlib.

This module is the stdlib side of gem5's hypercall-driven simulation-exit
infrastructure. A hypercall exit may originate inside the simulator, when a
model calls an ``exitSimulationLoop`` API with a hypercall ID, or in the guest
application, when code linked with the m5 library calls ``m5_hypercall``. The
``Simulator`` run loop reads that ID from the resulting exit event, looks it up
in ``ExitHandler.get_handler_map()``, constructs the matching ``ExitHandler``
with the event payload, and calls ``handle()``.

Terminology
-----------

``hypercall``
    The broad guest/simulator request mechanism. In this file, hypercalls are
    used to route simulation exits to Python handlers.

``hypercall ID``
    The integer value carried by the exit event. This is the runtime dispatch
    key. All handler registration eventually resolves to this integer.

``ExitHypercall``
    The pybind11 enum exposing gem5's built-in exit-handler hypercall IDs,
    generated from the C++ ``gem5::ExitHypercall`` enum. Use enum members such
    as ``ExitHypercall.WORK_BEGIN`` for built-in handlers.

``HypercallId``
    The Python type accepted by registration helpers: either an
    ``ExitHypercall`` enum member for a built-in hypercall or a raw ``int`` for
    a custom/user-defined hypercall ID. Strings are intentionally not accepted
    as selectors so call sites do not rely on loose name parsing.

Typical usage
-------------

Subclass ``ExitHandler`` for full control::

    class MyWorkBeginHandler(ExitHandler, hypercall=ExitHypercall.WORK_BEGIN):
        def _process(self, simulator: "Simulator") -> None:
            ...

        def _exit_simulation(self) -> bool:
            return False

Use ``register_exit_handler`` for simple function-based handlers::

    def handle_custom(simulator: "Simulator", payload: HypercallPayload) -> bool:
        return False

    register_exit_handler(None, handle_custom, "custom", hypercall_id=4096)
"""

import json
import socket
from abc import (
    ABC,
    abstractmethod,
)
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    Generator,
    List,
    Optional,
    Type,
    Union,
)

import m5
from m5 import options
from m5.util import warn

from _m5 import event as _m5_event

# Public finite-set selector for the hypercalls built into gem5. This pybind11
# enum is generated from the same C++ ExitHypercall enum that is backed by
# include/gem5/hypercall_ids.h.
ExitHypercall = _m5_event.ExitHypercall

from ..utils.override import overrides
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

"""Structured key/value metadata attached to a hypercall exit event.

The C++/pybind interface currently transports payload values as strings, so
handlers should parse values explicitly when they need integers, booleans, or
other richer types.
"""
HypercallPayload = Dict[str, str]

"""A hypercall identifier accepted by the Python ExitHandler registry.

Use ``ExitHypercall`` enum members for gem5-defined exit-handler hypercalls.
Use a raw ``int`` only for custom/user-defined hypercall IDs that are not part
of gem5's built-in ``ExitHypercall`` enum.
"""
HypercallId = Union[int, ExitHypercall]

"""Callable used by ``register_exit_handler``.

The function receives the active ``Simulator`` and the hypercall payload. It
returns ``True`` when the simulator should leave the run loop after handling the
event, or ``False`` to continue.
"""
ExitHandlerFunction = Callable[["Simulator", HypercallPayload], bool]


def _resolve_hypercall_id(hypercall: HypercallId) -> int:
    """Return the integer dispatch key for a hypercall selector.

    Args:
        hypercall: Either an ``ExitHypercall`` enum member for a gem5-defined
            hypercall or an integer custom/user-defined hypercall ID.

    Returns:
        The integer hypercall ID used as the key in ``ExitHandler``'s registry.

    Raises:
        TypeError: If ``hypercall`` is not an ``ExitHypercall`` or ``int``.
    """
    if isinstance(hypercall, ExitHypercall):
        return int(hypercall.value)
    if isinstance(hypercall, int):
        return hypercall
    raise TypeError(
        "hypercall must be an ExitHypercall enum member or an integer "
        f"hypercall ID, not {type(hypercall).__name__}"
    )


def _select_hypercall(
    hypercall: Optional[HypercallId],
    hypercall_id: Optional[int],
) -> Optional[HypercallId]:
    """Choose one of the public selector arguments.

    Args:
        hypercall: Preferred selector argument. Use an ``ExitHypercall`` enum
            member for gem5-defined hypercalls or an integer for custom IDs.
        hypercall_id: Explicit integer-only alias for custom/raw IDs. This
            exists to make custom ID registration self-documenting at call sites.

    Returns:
        The selected hypercall identifier, or ``None`` if neither was supplied.

    Raises:
        TypeError: If both selector arguments are supplied.
    """
    if hypercall is not None and hypercall_id is not None:
        raise TypeError("Specify only one of 'hypercall' or 'hypercall_id'.")
    return hypercall if hypercall is not None else hypercall_id


class ExitHandler(ABC):
    """Base class for all stdlib hypercall exit handlers.

    Subclasses register themselves when the class is created. The class-level
    registry maps an integer hypercall ID to the handler class responsible for
    that ID. The ``Simulator`` run loop uses this map after each hypercall exit
    event to choose which handler to instantiate.

    A subclass normally specifies ``hypercall=ExitHypercall.SOME_VALUE``. For
    custom hypercall IDs outside gem5's enum, use ``hypercall_id=<int>``.
    Subclasses of an existing registered handler inherit the base handler's ID
    unless they explicitly provide a different one.
    """

    # Shared registry used by the Simulator to map a hypercall ID to the
    # ExitHandler subclass that should be instantiated for that exit event.
    _handler_map: Dict[int, Type["ExitHandler"]] = {}

    # Hypercall ID handled by this class. Each registered subclass gets its own
    # value, while the base ExitHandler class keeps None.
    _handler_id: Optional[int] = None

    def __init_subclass__(
        cls,
        *,
        hypercall: Optional[HypercallId] = None,
        hypercall_id: Optional[int] = None,
        hypercall_num: Optional[int] = None,
        **kwargs: Any,
    ) -> None:
        """Register subclasses by the hypercall ID they handle.

        Args:
            hypercall: Preferred selector. Use an ``ExitHypercall`` enum member
                for a gem5-defined hypercall or an integer custom ID.
            hypercall_id: Explicit integer-only selector for raw/custom IDs.
            hypercall_num: Deprecated compatibility alias for ``hypercall_id``.
            **kwargs: Additional subclass keyword arguments for other base
                classes.

        Raises:
            TypeError: If both selector arguments are supplied, or if no
                selector can be determined from this class or its bases.

        ``hypercall_id`` is accepted when callers want to pass a raw/custom
        integer ID explicitly. New handlers for gem5-defined hypercalls should
        usually use ``hypercall`` with an ``ExitHypercall`` enum member.
        Subclasses of existing handlers inherit the base handler's hypercall
        unless they explicitly specify a different one.
        """
        super().__init_subclass__(**kwargs)

        selector = _select_hypercall(hypercall, hypercall_id)
        if hypercall_num is not None:
            if selector is not None:
                raise TypeError(
                    "Specify only one of 'hypercall', 'hypercall_id', or "
                    "'hypercall_num'."
                )
            selector = hypercall_num

        if selector is None:
            hypercall_id = cls._handler_id
        else:
            hypercall_id = _resolve_hypercall_id(selector)

        if hypercall_id is None:
            raise TypeError(
                f"Hypercall identifier must be provided for {cls.__name__}. "
                f"Use `class {cls.__name__}(ExitHandler, hypercall=<hypercall>):`"
            )

        cls._handler_id = hypercall_id
        ExitHandler._handler_map[hypercall_id] = cls

    @classmethod
    def get_handler_id(cls) -> Optional[int]:
        """Returns the hypercall ID handled by this class.

        Returns ``None`` for the base ``ExitHandler`` class.
        """
        return cls._handler_id

    @classmethod
    def get_handler_map(cls) -> Dict[int, Type["ExitHandler"]]:
        """Returns the mapping of hypercall IDs to handler classes."""
        return cls._handler_map

    def __init__(self, payload: HypercallPayload) -> None:
        """Create a handler for a single hypercall exit event.

        Args:
            payload: String key/value metadata attached to the exit event.
        """
        self._payload = payload

    def handle(self, simulator: "Simulator") -> bool:
        """Run this handler and report whether simulation should stop.

        Args:
            simulator: The active ``Simulator`` instance.

        Returns:
            ``True`` if ``Simulator.run()`` should return after this handler;
            ``False`` if the simulator should re-enter the event loop.
        """
        self._process(simulator)
        return self._exit_simulation()

    def get_handler_description(self) -> str:
        """Return a human-readable description for logs/status output."""
        return f"Exit Handler {self.__class__.__name__} called."

    @abstractmethod
    def _process(self, simulator: "Simulator") -> None:
        """Perform the handler's side effects.

        Args:
            simulator: The active ``Simulator`` instance.
        """
        raise NotImplementedError(
            "Method '_process' must be implemented by a subclass"
        )

    @abstractmethod
    def _exit_simulation(self) -> bool:
        """Return whether simulation should stop after ``_process``.

        Returns:
            ``True`` to leave ``Simulator.run()``; ``False`` to continue.
        """
        raise NotImplementedError(
            "Method '_exit_simulation' must be implemented by a subclass"
        )


def _ExitHandlerFactory(
    hypercall: HypercallId,
    func: ExitHandlerFunction,
    description: str,
) -> Type[ExitHandler]:
    """Create an ``ExitHandler`` subclass around a plain Python callable.

    Args:
        hypercall: Built-in ``ExitHypercall`` enum member or custom integer ID
            to bind to the generated handler class.
        func: Function to call when the hypercall is handled.
        description: Human-readable description stored on the handler instance.

    Returns:
        A newly defined ``ExitHandler`` subclass registered for ``hypercall``.
    """

    class NewExitHandler(ExitHandler, hypercall=hypercall):
        def __init__(self, payload: HypercallPayload) -> None:
            super().__init__(payload)
            self.should_exit = False
            self.description = description

        @overrides(ExitHandler)
        def _process(self, simulator: "Simulator") -> None:
            self.should_exit = func(simulator, self._payload)

        @overrides(ExitHandler)
        def _exit_simulation(self) -> bool:
            return self.should_exit

        @overrides(ExitHandler)
        def get_handler_description(self) -> str:
            return self.description

    return NewExitHandler


def register_exit_handler(
    hypercall: Optional[HypercallId],
    func: ExitHandlerFunction,
    description: str,
    *,
    hypercall_id: Optional[int] = None,
) -> Type[ExitHandler]:
    """Register a new exit handler for the provided hypercall.

    ``hypercall`` may be an ``ExitHypercall`` enum value for gem5-defined
    hypercalls or an ``int`` for custom/user-defined hypercalls. Use the
    ``hypercall_id`` keyword only when passing a raw/custom integer ID.

    Args:
        hypercall: Preferred selector. Use an ``ExitHypercall`` enum member for
            gem5-defined hypercalls or an integer for a custom ID. Pass ``None``
            only when using the keyword-only ``hypercall_id`` argument.
        func: Function called with ``(simulator, payload)`` when the hypercall
            is handled. It returns whether simulation should stop.
        description: Human-readable summary for logging/status output.
        hypercall_id: Explicit raw/custom integer hypercall ID.

    Returns:
        A new ``ExitHandler`` subclass bound to the requested hypercall.

    Raises:
        ValueError: If neither selector is provided or both are provided.

    Note:
        Use ``hypercall`` with an ``ExitHypercall`` enum member for gem5-defined
        hypercalls; use ``hypercall_id`` when registering a raw/custom integer
        ID that is not part of the built-in enum.
    """
    try:
        selector = _select_hypercall(hypercall, hypercall_id)
    except TypeError as exc:
        raise ValueError(str(exc)) from exc
    if selector is None:
        raise ValueError("A hypercall identifier must be provided.")
    return _ExitHandlerFactory(selector, func, description)


class ScheduledExitEventHandler(
    ExitHandler, hypercall=ExitHypercall.SCHEDULED_EXIT
):
    """Default handler for simulator-scheduled tick exits.

    ``SCHEDULED_EXIT`` is used when gem5 itself schedules a future exit, for
    example via ``m5.scheduleTickExitFromCurrent`` or the stdlib
    ``Simulator.set_hypercall_*_max_ticks`` helpers. This handler exits the
    simulator run loop by default. The payload may include:

    ``justification``
        Human-readable reason for scheduling the exit.
    ``scheduled_at_tick``
        Tick at which the scheduled exit was created.
    """

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    def justification(self) -> Optional[str]:
        """Returns why this scheduled exit event was created, if available."""
        return self._payload.get("justification")

    def scheduled_at_tick(self) -> Optional[int]:
        """Returns the tick at which this exit was scheduled, if available."""
        tick = self._payload.get("scheduled_at_tick")
        return None if tick is None else int(tick)

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


class KernelBootedExitHandler(
    ExitHandler, hypercall=ExitHypercall.KERNEL_BOOTED
):
    """Handle the guest reporting that the kernel has booted.

    The default behavior is to continue simulation.
    """

    @overrides(ExitHandler)
    def get_handler_description(self) -> str:
        return "Kernel booted."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class AfterBootExitHandler(ExitHandler, hypercall=ExitHypercall.AFTER_BOOT):
    """Handle the guest entering the ``after_boot`` hook.

    The default behavior is to continue simulation.
    """

    @overrides(ExitHandler)
    def get_handler_description(self) -> str:
        return "Started `after_boot.sh` script."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class AfterBootScriptExitHandler(
    ExitHandler, hypercall=ExitHypercall.AFTER_BOOT_SCRIPT
):
    """Handle the guest completing ``after_boot.sh``.

    The default behavior is to exit the simulator run loop.
    """

    @overrides(ExitHandler)
    def get_handler_description(self) -> str:
        return "Finished `after_boot.sh` script."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        pass

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


class CheckpointExitHandler(ExitHandler, hypercall=ExitHypercall.CHECKPOINT):
    """Take a gem5 checkpoint and continue simulation.

    The checkpoint is written below ``simulator._checkpoint_path`` when set,
    otherwise below the current gem5 output directory.
    """

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


class WorkBeginExitHandler(ExitHandler, hypercall=ExitHypercall.WORK_BEGIN):
    """Handle the start of a region of interest.

    The default behavior resets gem5 statistics and continues simulation.
    """

    @overrides(ExitHandler)
    def get_handler_description(self) -> str:
        return "Started executing Region of Interest (ROI)."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.reset()

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class WorkEndExitHandler(ExitHandler, hypercall=ExitHypercall.WORK_END):
    """Handle the end of a region of interest.

    The default behavior dumps gem5 statistics and continues simulation.
    """

    @overrides(ExitHandler)
    def get_handler_description(self) -> str:
        return "Finished executing Region of Interest (ROI)."

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class OrchestratorExitHandler(
    ExitHandler, hypercall=ExitHypercall.ORCHESTRATOR
):
    """Handle out-of-band orchestration/status requests.

    The orchestrator hypercall is used by tooling that wants to query or adjust
    a running simulation. The payload selects a function and may include a Unix
    domain socket path for the JSON response. Supported functions include
    ``status``, ``get_stats``, and ``update_debug_flags``.
    """

    def _get_status(self, simulator: "Simulator") -> Dict[str, Any]:
        """Collect basic simulator status for an orchestrator response.

        Args:
            simulator: The active ``Simulator`` instance.

        Returns:
            A JSON-serializable dictionary of workload, tick, simulator ID, and
            instruction-count metadata.
        """
        import _m5.core

        if not simulator.get_workload():
            workload_id = "No workload set"
        else:
            workload_id = simulator.get_workload().get_id()
        return {
            "workload": workload_id,
            "tick": simulator.get_current_tick(),
            "ticks_per_second": _m5.core.getClockFrequency(),
            "sim_id": simulator.get_id(),
            "curr_instructions_executed": simulator.get_instruction_count(),
        }

    def _add_debug_flags(self, debug_flags: List[str]) -> Dict[str, str]:
        """Enable or disable gem5 debug flags.

        Args:
            debug_flags: Flag names to enable. Prefix a name with ``-`` to
                disable it instead.

        Returns:
            A dictionary listing enabled, disabled, and invalid flags. Values
            are strings so the response is compatible with the current payload
            conventions.
        """
        from m5 import debug

        flags_disabled = []
        flags_enabled = []
        invalid_flags = []
        for flag in debug_flags:
            off = False
            if flag.startswith("-"):
                flag = flag[1:]
                off = True

            if flag not in debug.flags:
                invalid_flags.append(flag)
                continue

            if off:
                flags_disabled.append(flag)
                debug.flags[flag].disable()
            else:
                flags_enabled.append(flag)
                debug.flags[flag].enable()

        return {
            "flags_enabled": str(flags_enabled),
            "flags_disabled": str(flags_disabled),
            "invalid_flags": str(invalid_flags),
        }

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        try:
            socket_path = self._payload.get("response_socket")
            function = self._payload.get("function", "status")
            arguments = self._payload.get("arguments")
            if socket_path:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(socket_path)

                if function == "status":
                    response = json.dumps(self._get_status(simulator))
                elif function == "get_stats":
                    stats = simulator.get_stats()
                    response = json.dumps(stats)
                elif function == "update_debug_flags":
                    debug_flags = arguments.split(",")
                    response = json.dumps(self._add_debug_flags(debug_flags))
                else:
                    response = json.dumps(
                        {"error": f"Unknown function: {function}"}
                    )

                sock.send(response.encode())
                sock.close()

        except Exception as e:
            print(f"Error in OrchestratorExitHandler: {e}")

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class ClassicGeneratorExitHandler(
    ExitHandler, hypercall=ExitHypercall.CLASSIC_GENERATOR
):
    """A handler designed to be the default for the classic exit event.

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
    again before finally, on the fourth exit event, ``stop_simulation`` will
    stop the simulation by returning ``True``.

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
        * ExitEvent.WORKEND: dump stats
        * ExitEvent.USER_INTERRUPT: exit simulation
        * ExitEvent.MAX_TICK: exit simulation
        * ExitEvent.SCHEDULED_TICK: exit simulation
        * ExitEvent.SIMPOINT_BEGIN: reset stats
        * ExitEvent.MAX_INSTS: exit simulation

    These generators can be found in the ``exit_event_generator.py`` module.
    """

    def __init__(self, payload: HypercallPayload) -> None:
        """Create a classic-generator compatibility handler instance.

        Args:
            payload: Payload containing at least the legacy ``cause`` when the
                event came through the new structured hypercall path.
        """
        super().__init__(payload)
        self._exit_on_completion: Optional[bool] = None

    @classmethod
    def set_exit_event_map(
        cls,
        on_exit_event: Optional[
            Dict[
                ExitEvent,
                Union[
                    Generator[Optional[bool], None, None],
                    List[Callable[[], Optional[bool]]],
                    Callable[[], Optional[bool]],
                ],
            ]
        ],
        expected_execution_order: Optional[List[ExitEvent]],
        board: Optional["Board"],
    ) -> None:
        """Configure legacy ``ExitEvent`` handling for classic exits.

        This bridges the older message/cause based exit mechanism to the newer
        hypercall-dispatch mechanism. ``ClassicGeneratorExitHandler`` receives
        the classic exit cause, translates it to an ``ExitEvent``, then advances
        the matching user-supplied or default generator.

        Args:
            on_exit_event: Optional mapping from ``ExitEvent`` to a generator,
                list of zero-argument callables, or a single zero-argument
                callable. Generators/callables yield or return ``True`` to exit
                the simulator run loop and ``False``/``None`` to continue.
            expected_execution_order: Optional sequence of ``ExitEvent`` values
                used to validate that exits occur in the expected order.
            board: Board whose processor is used by default handlers such as
                CPU switching and spatter synchronization.
        """
        # We specify a dictionary here outlining the default behavior for each
        # exit event. Each exit event is mapped to a generator.
        cls._default_on_exit_dict = {
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
            cls._on_exit_event = {}
            for key, value in on_exit_event.items():
                if isinstance(value, Generator):
                    cls._on_exit_event[key] = value
                elif isinstance(value, List):
                    # In instances where we have a list of functions, we
                    # convert this to a generator.
                    cls._on_exit_event[key] = (func() for func in value)
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

                    def function_generator(
                        func: Callable[[], Optional[bool]],
                    ) -> Generator[Optional[bool], None, None]:
                        while True:
                            yield func()

                    cls._on_exit_event[key] = function_generator(func=value)
                else:
                    raise Exception(
                        f"`on_exit_event` for '{key.value}' event is "
                        "not a Generator or List[Callable]."
                    )
        else:
            cls._on_exit_event = cls._default_on_exit_dict

        cls._expected_execution_order = expected_execution_order

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        # Translate the exit event cause to the exit event enum. New-style
        # ClassicGenerator exits store the legacy cause in the structured
        # payload rather than the GlobalSimLoopExitEvent cause field.
        cause = self._payload.get("cause")
        if cause is None:
            cause = simulator.get_last_exit_event_cause()
        exit_enum = ExitEvent.translate_exit_status(cause)

        # Check to see the run is corresponding to the expected execution
        # order (assuming this check is demanded by the user).
        if self._expected_execution_order:
            expected_enum = self._expected_execution_order[
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
            self._exit_on_completion = next(self._on_exit_event[exit_enum])
        except StopIteration:
            # If the user's generator has ended, throw a warning and use
            # the default generator for this exit event.
            warn(
                "User-specified generator/function list for the exit "
                f"event'{exit_enum.value}' has ended. Using the default "
                "generator."
            )
            self._exit_on_completion = next(
                self._default_on_exit_dict[exit_enum]
            )

        except KeyError:
            # If the user has not specified their own generator for this
            # exit event, use the default.
            self._exit_on_completion = next(
                self._default_on_exit_dict[exit_enum]
            )

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        assert (
            self._exit_on_completion is not None
        ), "Exit on completion boolean var is not set."
        return self._exit_on_completion
