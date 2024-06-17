# Copyright (c) 2024 The Regents of the University of California
# All Rights Reserved.
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

"""This module contains the gem5 MultiSim framework. The gem5 MultiSim
work functions by allowing a user to specify multiple simulations to run from
a single gem5 config script. The MuliSim framework will then run these
simulations in parallel using the Python multiprocessing library.

The framework works by having the user add the simulators in a configuration
via the `add_simulator` function, each with a unique, user specified, id. This
adds the different simulations to run in parallel to a global list. The
MultiSim framework then uses the Python multiprocessing library to run the
simulations in parallel by loading the config script as a module in each child
process and then selecting the simulation to run via the iid in the global set
of scheduled simulators jobs to run.
The only difference between the child processes is the id of the simulator.

Important notes
---------------

1. You cannot load/instantiate the simulators in the main process. You cannot
even load the config script (i.e., `import config_script`). This means the
config script is passed as a string referencing the config script as a module.
This script is then passed to the child processes to load.

2. The config script cannot accept parameters. It must be parameterless.
"""

import importlib
import multiprocessing
from pathlib import Path
from typing import (
    Optional,
    Set,
)

# A global variable which __main__.py flips to `True` when multisim is run as
# an executable module.
module_run = False

# A global variable to store the simulators to run in parallel. If `None`, then
# `None` is passed to the `multiprocessing.Pool` which instructs
# multiprocessing to use the maximum number of available threads.
# threads.
_num_processes = None

_multi_sim: Set["Simulator"] = set()


def _load_module(module_path: Path) -> None:
    """Load the module at the given path."""
    spec = importlib.util.spec_from_file_location(
        "gem5target", str(module_path)
    )
    modulevar = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulevar)


def _get_simulator_ids_child_process(id_list, module_path: Path) -> None:
    """Get the ids of the simulations to be run.

    This function is passed to the Python multiprocessing module and run with
    the correct module path in the `_get_simulator_ids` function. This function
    is run in a child process which loads the module (config script) then reads
    the IDs.

    Note: We run this as child process as we cannot load the config script as
    a module in the main process. This function is used in
    `get_simulator_ids` and should be used separately.
    """

    _load_module(module_path)
    global _multi_sim
    if len(id_list) != 0:
        id_list *= 0
    id_list.extend([sim.get_id() for sim in _multi_sim])


def _get_num_processes_child_process(
    num_processes_dict, module_path: Path
) -> None:
    """Get the ids of the simulations to be run.

    This function is passed to the Python multiprocessing module and run with
    the correct module path in the `_get_simulator_ids` function. This function
    is run in a child process which loads the module (config script) then reads
    the IDs.

    Note: We run this as child process as we cannot load the config script as
    a module in the main process. This function is used in
    `get_simulator_ids` and should be used separately.
    """

    _load_module(module_path)
    global _num_processes
    num_processes_dict["num_processes"] = _num_processes


def get_simulator_ids(config_module_path: Path) -> list[str]:
    """This is a  hack to determine the IDs of the simulations we are to run.
    The only way we can know is by importing the module, which we can only do
    in the child processes. We therefore create a child process with the
    sole purpose of importing the module and returning the IDs via
    a `multiprocessing.Manager` dictionary.

    This function handles the creation of the `multiprocessing.Manager` and the
    `multiprocessing.Process`. It then waits for the process to finish to then
    return the ids as a set of strings.
    """

    manager = multiprocessing.Manager()
    id_list = manager.list()
    p = multiprocessing.Process(
        target=_get_simulator_ids_child_process,
        args=(id_list, config_module_path),
    )
    p.start()
    p.join()
    return id_list


def get_num_processes(config_module_path: Path) -> Optional[int]:
    manager = multiprocessing.Manager()
    num_processes_dict = manager.dict()
    p = multiprocessing.Process(
        target=_get_num_processes_child_process,
        args=(num_processes_dict, config_module_path),
    )
    p.start()
    p.join()
    return num_processes_dict["num_processes"]


def _run(module_path: Path, id: str) -> None:
    """Run the simulator with the ID specified."""

    _load_module(module_path)

    global _multi_sim
    sim_list = [sim for sim in _multi_sim if sim.get_id() == id]

    assert len(sim_list) != 0, f"No simulator with id '{id}' found."
    assert len(sim_list) == 1, f"Multiple simulators with id '{id}' found."
    import m5

    subdir = Path(Path(m5.options.outdir) / Path(sim_list[0].get_id()))
    sim_list[0].override_outdir(subdir)

    sim_list[0].run()


def run(module_path: Path, processes: Optional[int] = None) -> None:
    """Run the simulators specified in the module in parallel.

    :param module_path: The path to the module containing the simulators to
    run.
    :param processes: The number of processes to run in parallel. If not
    specified, the number of available threads will be used.
    """

    assert len(_multi_sim) == 0, (
        "Simulators instantiated in main thread instead of child thread "
        "(prior to determining number of jobs)."
    )

    # Get the simulator IDs. This both provides us a list of targets
    # and, by-proxy, the number of jobs.
    ids = get_simulator_ids(module_path)
    max_num_processes = get_num_processes(module_path)

    assert len(_multi_sim) == 0, (
        "Simulators instantiated in main thread instead of child thread "
        "(after determining number of jobs)."
    )

    # Setup the multiprocessing pool. If the number of processes is not
    # specified (i.e. `None`) the default is the number or available threads.
    from ..multiprocessing.context import gem5Context

    pool = gem5Context().Pool(processes=max_num_processes, maxtasksperchild=1)

    # Use the starmap function to create N child processes each with same
    # module path (the config script specifying all simulations using MultiSim)
    # but a different ID. The ID is used to select the correct simulator to
    # run.
    pool.starmap(_run, zip([module_path for _ in range(len(ids))], tuple(ids)))


def set_num_processes(num_processes: int) -> None:
    """Set the max number of processes to run in parallel.

    :param num_processes: The number of processes to run in parallel.
    """
    if num_processes < 1:
        raise ValueError("Number of processes must be greater than 0.")
    if isinstance(num_processes, int):
        global _num_processes
        _num_processes = num_processes
    else:
        raise ValueError("Number of processes must be an integer.")


def num_simulators() -> int:
    """Returns the number of simulators added to the MultiSim."""
    return len(_multi_sim)


def add_simulator(simulator: "Simulator") -> None:
    """Add a single simulator to the Multisim. Doing so informs the simulators
    to run this simulator via multiprocessing.

    **Note:** If this function is not run using the MultiSim module then the
    user will be prompted to either do so if they desire multiple gem5
    processes or to pass the id of the simulator to run. This function will
    attempt to run the simulation with the id passed as an argument. If
    such simulation exists the simulation will end without failure (or any
    simulations having been run).

    :param simulator: The simulator to add to the multisim.
    :param id: The id of the simulator. This is used to reference the
    simululation. This is particularly important when referencing the correct
    m5out subdirectory.
    """

    global _multi_sim
    if not simulator.get_id():
        # The default simulator id is the length of the current set of
        # simulators. This is used to ensure that the simulator has a unique
        # id.
        simulator.set_id(f"sim_{len(_multi_sim)}")
    _multi_sim.add(simulator)

    # The following code is used to enable a user to run a single simulation
    # from the config script, based on an ID, in the case the config script is
    # passed a traditional gem5 config and not via the multisim module.
    global module_run
    if not module_run:
        import argparse

        parser = argparse.ArgumentParser(
            description="Run a specific simulation based on the id."
        )
        parser.add_argument(
            "id",
            type=str,
            nargs="?",
            default=None,
            help="The id of the simulator to run.",
        )
        parser.add_argument(
            "-l",
            "--list",
            help="List the ids of the simulators to run.",
            action="store_true",
        )
        args = parser.parse_args()
        if args.list:
            print(simulator.get_id())
        elif not args.id:
            raise Exception(
                "If running this script directly as a configuration script "
                "then a single argument must be specified: the id of the "
                "simulator to run. This will run the simulation associated "
                "with that id and no other. If thgfe intent is instead to run "
                "the script via the MultiSim utility then run this script via "
                "the multisim module: "
                "`<gem5> -m gem5.utils.multisim <config_script>`.\n\n"
                "To list the ids of the simulators to run use the `--list` "
                "(`-l`)  flag."
            )
        elif args.id == simulator.get_id():
            import m5

            subdir = Path(Path(m5.options.outdir) / Path(simulator.get_id()))
            simulator.override_outdir(subdir)
            simulator.run()
