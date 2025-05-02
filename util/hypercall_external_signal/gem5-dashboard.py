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

from gem5.resources.resource import obtain_resource

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

    # Mac OS doesn't have /proc, so we need a different approach
    elif platform.system() == "Darwin":
        # Get processes for current user
        user_processes = subprocess.run(
            ["ps", "-U", str(current_user)], capture_output=True
        ).stdout.decode("UTF-8")

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

    # Add support for other platforms here if needed
    else:
        logger.error(f"Unsupported platform: {platform.system()}")

    if not gem5_pids:
        raise ValueError("No valid gem5 process found")

    return gem5_pids


def format_ticks_time(ticks):
    """Convert tick count to human-readable time format"""
    # Define tick to time unit conversions (example values - adjust based on gem5's tick rate)
    # Assuming 1 tick = 1 picosecond for gem5
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


def format_instruction_count(count):
    """Format instruction count with appropriate suffix (K, M, B, T)"""
    if count < 1_000:
        return f"{count}"
    elif count < 1_000_000:
        return f"{count/1_000:.4g}K"
    elif count < 1_000_000_000:
        return f"{count/1_000_000:.4g}M"
    elif count < 1_000_000_000_000:
        return f"{count/1_000_000_000:.4g}B"
    else:
        return f"{count/1_000_000_000_000:.4g}T"


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

            # Get workload information
            workload = obtain_resource(curr_info["workload"])
            self.max_insts = workload.get_estimated_instructions() or 0

            # Store simulation properties
            self.current_insts = curr_info["instruction_count"]
            self.sim_id = curr_info["sim_id"]
            self.workload_id = curr_info["workload"]
            self.current_ticks = curr_info["tick"]

            # Create the actual tqdm progress bar
            desc_format = f"{self.sim_id}|{self.workload_id}"
            self.bar = tqdm.tqdm(
                total=self.max_insts,
                initial=self.current_insts,
                desc=desc_format,
                unit="insts",
                position=self.position,
                leave=True,
            )

            # Initial postfix update
            self.bar.set_postfix(
                {"SimTime": format_ticks_time(self.current_ticks)},
                refresh=False,
            )

            # Reset retry counter on successful initialization
            self.retries = 0

        except Exception as e:
            self.retries += 1
            if self.retries < self.max_retries:
                logger.warning(
                    f"Error initializing progress bar for PID {self.pid} (attempt {self.retries}/{self.max_retries}): {e}"
                )
                logger.warning(
                    f"Will retry initialization during next update cycle"
                )
                # Keep active so update() will try again
                self.active = True
            else:
                logger.error(
                    f"Failed to initialize progress bar for PID {self.pid} after {self.max_retries} attempts: {e}"
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
        self.update_interval = 5  # Update every 5 seconds
        self.progress_bars = {}
        self.running = True
        self.auto_discover = auto_discover
        self.last_discovery_time = 0
        self.discovery_interval = (
            15  # Check for new processes every 15 seconds
        )
        self._initialize_bars()
        self.run()

    def _initialize_bars(self, new_pids=None):
        """
        Initialize progress bars for PIDs.

        Args:
            new_pids: Optional list of new PIDs to initialize bars for
        """
        pids_to_initialize = new_pids if new_pids is not None else self.pids

        for pid in pids_to_initialize:
            # Skip if we already have a progress bar for this PID
            if pid in self.progress_bars:
                continue

            # Calculate position (number of existing bars)
            position = len(self.progress_bars)
            self.progress_bars[pid] = Gem5ProgressBar(pid, position=position)

    def _discover_new_processes(self):
        """Discover new gem5 processes and create progress bars for them."""
        current_time = time.time()

        # Only check for new processes at the specified interval
        if current_time - self.last_discovery_time < self.discovery_interval:
            return

        self.last_discovery_time = current_time

        try:
            # Find all current gem5 processes
            current_pids = set(find_gem5_pids())

            # Identify new PIDs that we're not already monitoring
            new_pids = current_pids - self.pids

            if new_pids:
                logger.info(
                    f"Discovered {len(new_pids)} new gem5 process(es): {new_pids}"
                )

                # Update our set of known PIDs
                self.pids.update(new_pids)

                # Initialize progress bars for new PIDs
                self._initialize_bars(new_pids)

        except Exception as e:
            logger.error(f"Error discovering new gem5 processes: {e}")

    def _rearrange_positions(self):
        """Rearrange progress bar positions to account for closed bars."""
        active_bars = [
            bar for pid, bar in self.progress_bars.items() if bar.active
        ]

        for i, bar in enumerate(active_bars):
            if bar.position != i:
                bar.position = i
                if bar.bar:
                    # Update the tqdm bar's position
                    bar.bar.pos = i
                    # Force refresh to reflect new position
                    bar.bar.refresh()

    def run(self):
        """Run update loop for all progress bars."""
        try:
            while self.running and (
                any(bar.active for bar in self.progress_bars.values())
                or self.auto_discover
            ):
                # Sleep between updates
                time.sleep(self.update_interval)

                # Check for new processes if auto-discovery is enabled
                if self.auto_discover:
                    self._discover_new_processes()

                # Update each active progress bar
                for pid, bar in list(self.progress_bars.items()):
                    if not bar.active:
                        continue

                    if not bar.update():
                        # If update failed, bar is now inactive
                        # We'll rearrange positions at the end of the loop
                        pass

                # Rearrange positions if needed to fill in gaps from closed bars
                self._rearrange_positions()

        except KeyboardInterrupt:
            self.running = False
        finally:
            # Clean up all progress bars
            for bar in self.progress_bars.values():
                bar.close()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--pid",
        help="Enter the pid of the gem5 process for which you would like to "
        "observe the progress",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--single",
        help="Monitor a single process instead of using the manager",
        action="store_true",
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
                # If auto-discovering, start with empty list and wait for processes
                logger.info(
                    "No gem5 processes found, waiting for new processes..."
                )
                pids = []
            else:
                # Otherwise exit with error
                logger.error("No gem5 processes found. Exiting.")
                exit(1)
    else:
        pids = [args.pid]

    if args.single and len(pids) > 0:
        # Single progress bar mode - useful for scripts that want to embed a progress bar
        bar = Gem5ProgressBar(pids[0])
        try:
            while bar.active:
                time.sleep(5)
                bar.update()
        except KeyboardInterrupt:
            pass
        finally:
            bar.close()
    else:
        # Multi-bar manager mode
        ProgressBarManager(pids, auto_discover=args.auto_discover)


if __name__ == "__m5_main__":
    # Disable logging to stdout/stderr
    logging.getLogger().handlers.clear()
    # Set up logging to file
    log_file = "gem5_progress.log"
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    logger.info("Starting gem5 progress bar manager")
    main()
elif __name__ == "__main__":
    print("This script is intended to be run from gem5")
    exit(1)
