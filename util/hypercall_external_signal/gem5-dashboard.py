# Copyright (c) 2025 The Regents of the University of California
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

import argparse
import json
import logging
import os
import platform
import signal
import subprocess
import time
from typing import (
    Any,
    Dict,
    List,
    Optional,
)

import tqdm
from orchestrator_request import (
    send_and_receive_hypercall,
)

try:
    from gem5.resources.resource import obtain_resource
except:
    pass

logger = logging.getLogger(__name__)


def find_gem5_pids() -> List[int]:
    """
    Find the PID of a running gem5 process that belongs to the current user.

    Searches for processes containing 'gem5' in their name.
    Filters out processes that:
    - Belong to other users
    - Are part of gem5's multisim infrastructure
    - Cannot receive signals (permission error)

    :return: List of valid gem5 process IDs
    :raises ValueError: If no gem5 process found
    """
    gem5_pids = []
    current_user = os.getuid()

    def is_valid_gem5_process(pid_str, cmdline=""):
        """Check if the process is a valid gem5 process."""
        # Skip the current process
        if int(pid_str) == os.getpid():
            return False

        # Skip multisim processes
        if any(
            pattern in cmdline
            for pattern in [
                "gem5.utils.multisim",
                "multiprocessing.resource_tracker",
            ]
        ):
            return False

        # On macOS, check for additional patterns
        if platform.system() == "Darwin" and (
            "multiprocessing.spawn" in cmdline and "--outdir" not in cmdline
        ):
            return False

        # Check if we can send signals to the process (i.e., we own it)
        try:
            pid = int(pid_str)
            # Use SIGCONT instead of signal 0
            os.kill(pid, signal.SIGCONT)
            return True
        except (OSError, PermissionError):
            # Process doesn't exist anymore or belongs to another user
            return False

    # Platform-specific process detection
    if platform.system() == "Linux":
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue

            try:
                # Check process owner
                stat_info = os.stat(f"/proc/{pid}")
                if stat_info.st_uid != current_user:
                    continue

                # Read process name from /proc/[pid]/comm
                with open(f"/proc/{pid}/comm") as f:
                    comm = f.read().strip()

                    if "gem5" in comm:
                        with open(f"/proc/{pid}/cmdline") as cmd_file:
                            cmdline = cmd_file.read().strip()
                            if is_valid_gem5_process(pid, cmdline):
                                gem5_pids.append(int(pid))
            except (OSError, PermissionError):
                # Skip processes we can't read
                continue

    # macOS doesn't have /proc, so we need a different approach
    elif platform.system() == "Darwin":
        # Filter for gem5 processes
        gem5_pids_proc = subprocess.run(["pgrep", "gem5"], capture_output=True)

        if gem5_pids_proc.returncode == 0:
            str_gem5_pids = gem5_pids_proc.stdout.decode("UTF-8").split("\n")
            str_gem5_pids = [pid for pid in str_gem5_pids if pid]

            for pid in str_gem5_pids:
                cmdline = subprocess.run(
                    ["ps", "-p", pid], capture_output=True
                ).stdout.decode("UTF-8")

                if is_valid_gem5_process(pid, cmdline):
                    gem5_pids.append(int(pid))

    else:
        logger.error(f"Unsupported platform: {platform.system()}")

    if not gem5_pids:
        raise ValueError("No valid gem5 process found")

    return gem5_pids


def format_ticks_time(ticks):
    """
    Convert tick count to human-readable time format.
    1 tick = 1 picosecond.
    """
    if ticks < 1_000:
        return f"{ticks}ps"
    elif ticks < 1_000_000:
        return f"{ticks/1000:.2f}ns"
    elif ticks < 1_000_000_000:
        return f"{ticks/1_000_000:.2f}μs"
    elif ticks < 1_000_000_000_000:
        return f"{ticks/1_000_000_000:.2f}ms"
    else:
        return f"{ticks/1_000_000_000_000:.2f}s"


class Gem5ProgressBar:
    """A reusable progress bar for monitoring a gem5 simulation."""

    def __init__(self, pid: int, position: int = 0, max_retries: int = 3):
        """
        Initialize a progress bar for a gem5 process.

        Args:
            pid: The process ID of the gem5 simulation to monitor
            position: The vertical position of this progress bar in a multi-bar
                      display
            max_retries: Maximum number of initialization retries
        """
        self.pid = pid
        self.position = position
        self.active = True
        self.bar = None
        self.max_retries = max_retries
        self.retries = 0
        self._initialize()

    def _initialize(self):
        """Initialize the progress bar with simulation data."""
        try:
            # Get initial simulation status
            curr_info = json.loads(
                send_and_receive_hypercall(self.pid, "status")
            )

            workload = obtain_resource(curr_info["workload"])
            self.max_insts = workload.get_estimated_instructions() or 0
            logger.info(
                f"Max instructions for workload {workload} is "
                f"{self.max_insts}"
            )
            self.current_insts = curr_info["instruction_count"]
            self.sim_id = curr_info["sim_id"]
            self.workload_id = curr_info["workload"]
            self.current_ticks = curr_info["tick"]

            # Create the actual tqdm progress bar
            desc_format = f"{self.pid}|{self.sim_id}|{self.workload_id}"
            self.bar = tqdm.tqdm(
                total=self.max_insts,
                initial=self.current_insts,
                desc=desc_format,
                unit="insts",
                position=self.position,
                leave=True,
                dynamic_ncols=True,
            )

            # Initial postfix update
            self.bar.set_postfix(
                {"SimTime": format_ticks_time(self.current_ticks)},
                refresh=False,
            )

            # Reset retry counter on successful initialization
            self.retries = 0
            logger.info(
                f"Successfully initialized progress bar for PID {self.pid}"
            )
        except Exception as e:
            self.retries += 1
            if self.retries < self.max_retries:
                logger.warning(
                    f"Error initializing progress bar for PID {self.pid} "
                    f"(attempt {self.retries}/{self.max_retries}): {e}"
                )
                logger.exception(e)
                logger.warning(
                    f"Will retry initialization during next update cycle"
                )
                # Keep active so update() will try again
                self.active = True
            else:
                logger.error(
                    f"Failed to initialize progress bar for PID {self.pid} "
                    f"after {self.max_retries} attempts: {e}"
                )
                self.active = False

    def update(self) -> bool:
        """
        Update the progress bar with current simulation status.
        If not initialized yet, retry initialization.

        Returns:
            bool: True if update was successful, False otherwise
        """
        if not self.active:
            return False

        # If bar isn't initialized yet, retry initialization
        if self.bar is None:
            self._initialize()
            # Still initializing, return True to keep trying
            return self.active

        try:
            # Get current simulation status
            curr_info = json.loads(
                send_and_receive_hypercall(self.pid, "status")
            )

            # Update values
            self.current_insts = curr_info["instruction_count"]
            self.current_ticks = curr_info["tick"]

            # Update the progress bar
            self.bar.n = self.current_insts
            self.bar.set_postfix(
                {"SimTime": format_ticks_time(self.current_ticks)},
                refresh=True,
            )

            return True
        except Exception as e:
            logger.error(
                f"Error updating progress bar for PID {self.pid}: {e}"
            )
            self.active = False
            return False

    def close(self):
        """Clean up the progress bar."""
        if self.bar:
            self.bar.close()


class ProgressBarManager:
    """Manages multiple Gem5ProgressBar instances."""

    def __init__(self, pids: List[int], auto_discover: bool = False):
        """
        Initialize the progress bar manager.

        Args:
            pids: List of initial gem5 process IDs to monitor
            auto_discover: Whether to automatically discover new gem5 processes
        """
        self.pids = set(pids)
        self.update_interval = 5
        self.progress_bars = {}
        self.running = True
        self.auto_discover = auto_discover
        self.last_discovery_time = 0
        self.discovery_interval = (
            15  # Check for new processes every 15 seconds
        )
        self._initialize_bars()
        self._print_dashboard_label()
        self.run()

    def _initialize_bars(self, new_pids=None):
        """
        Initialize progress bars for PIDs.

        Args:
            new_pids: Optional list of new PIDs to initialize bars for
        """
        pids_to_initialize = new_pids if new_pids is not None else self.pids
        for pid in pids_to_initialize:
            if pid in self.progress_bars:
                continue

            position = len(self.progress_bars)
            self.progress_bars[pid] = Gem5ProgressBar(pid, position=position)

    def _discover_new_processes(self):
        """Discover new gem5 processes and create progress bars for them."""
        current_time = time.time()

        if current_time - self.last_discovery_time < self.discovery_interval:
            return

        self.last_discovery_time = current_time

        try:
            current_pids = set(find_gem5_pids())

            new_pids = current_pids - self.pids

            if new_pids:
                logger.info(
                    f"Discovered {len(new_pids)} new gem5 process(es): "
                    f"{new_pids}"
                )

                self.pids.update(new_pids)

                self._initialize_bars(new_pids)

        except Exception as e:
            logger.error(f"Error discovering new gem5 processes: {e}")

    def _rearrange_positions(self):
        """Rearrange progress bar positions to account for closed bars."""
        active_bars = [
            bar
            for bar in self.progress_bars.values()
            if bar.active and bar.bar is not None
        ]

        for i, bar in enumerate(active_bars):
            if bar.position != i:
                bar.position = i
                if bar.bar:
                    # Update the tqdm bar's position
                    bar.bar.pos = i

        # clear the screen so outdated prints of active progress bars in a
        # lower position than the exited progress bar do not linger on the
        # dashboard
        self._print_dashboard_label()
        # refresh bars only after all positions have been updated. This
        # prevents the same bar from being printed multiple times.
        for bar in active_bars:
            bar.bar.refresh()

    def run(self):
        """Run update loop for all progress bars."""
        try:
            while self.running and (
                any(bar.active for bar in self.progress_bars.values())
                or self.auto_discover
            ):
                time.sleep(self.update_interval)

                if self.auto_discover:
                    self._discover_new_processes()

                for bar in self.progress_bars.values():
                    if not bar.active:
                        continue
                    bar.update()

                # fill in gaps from closed bars if necessary
                self._rearrange_positions()

        except KeyboardInterrupt:
            self.running = False
        finally:
            for bar in self.progress_bars.values():
                bar.close()

    def _print_dashboard_label(self) -> None:
        screen_width = os.get_terminal_size()[0]
        os.system("cls||clear")
        left_justified_label = " pid | simulation id | workload: "
        print(left_justified_label, end="")
        right_justified_label = (
            " current insts / total insts [elapsed<remaining, rate, "
            "simulated time]"
        )
        print(
            right_justified_label.rjust(
                screen_width - len(left_justified_label)
            )
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pid",
        help="Enter the pid(s) of the gem5 process(es) to display. "
        "If entering multiple pids, please separate pids using spaces, "
        "e.g. --pid 12345 12346. If --pid is not passed, the dashboard will "
        "automatically detect the currently running gem5 processes, but will "
        "not continue to discover new processes unless --auto-discover is "
        "passed.",
        nargs="*",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--auto-discover",
        help="Automatically discover and monitor new gem5 processes",
        action="store_true",
    )

    args = parser.parse_args()

    if args.pid is None:
        try:
            pids = find_gem5_pids()
            logger.debug(f"Found gem5 processes: {pids}")
        except ValueError:
            if args.auto_discover:
                logger.info(
                    "No gem5 processes found, waiting for new processes..."
                )
                pids = []
            else:
                logger.error("No gem5 processes found. Exiting.")
                exit(1)
    else:
        pids = args.pid

    ProgressBarManager(pids, auto_discover=args.auto_discover)


if __name__ == "__m5_main__":
    # Disable logging to stdout/stderr
    logging.getLogger().handlers.clear()
    log_file = "gem5_progress.log"
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    logger.info("Starting gem5 progress bar manager")
    main()
elif __name__ == "__main__":
    logger.error("This script is intended to be run from gem5")
    exit(1)
