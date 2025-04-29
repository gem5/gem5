#!/usr/bin/env python3
# Copyright (c) Jason Lowe-Power
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

"""
A utility script for sending hypercalls to gem5 and receiving responses
via Unix sockets.

This script provides a command-line interface for communicating with a running
gem5 simulation. It supports sending various commands and receiving
JSON-formatted responses.

Supported functions:
    * status: Get current simulation status (workload, ticks, etc.)
    * get_stats: Retrieve simulation statistics

Usage::

    orchestrator-request.py [--pid PID] <function>

Arguments:
    * --pid: Process ID of gem5 (optional, auto-detected if not specified)
    * function: Command to execute (status|get_stats)

Dependencies:
    * transmitter.py: For sending hypercalls to gem5

Examples::

    # Auto-detect gem5 process and get status
    $ python orchestrator-request.py status

    # Get stats from specific gem5 process
    $ python orchestrator-request.py --pid 1234 get_stats

Response Format:
    Example status response::

        {
            "workload": "x86-ubuntu-22.04",
            "tick": 1234567,
            "sim_id": "abc123",
            "instruction_count": 9876
        }

Error Handling:
    * Raises ValueError if no gem5 process found or multiple processes found
    * Raises TimeoutError if gem5 doesn't respond within timeout period
    * Handles SIGINT (Ctrl+C) gracefully with proper cleanup

Note:
    Requires a running gem5 simulation with hypercall support enabled
    (gem5-v25.0 or later).
"""

import argparse
import json
import logging
import os
import platform
import select
import signal
import socket
import subprocess
import sys
from typing import List

from transmitter import send_signal

logger = logging.getLogger(__name__)
socket_path = None
sock = None


def find_gem5_pids() -> List[int]:
    """
    Find the PID of a running gem5 process.

    Searches through /proc for a process containing 'gem5' in its name.

    :return: Process ID of the gem5 process
    :raises ValueError: If no gem5 process found or if multiple gem5 processes
                        are found
    """
    gem5_pids = []

    # List all processes in /proc
    if platform.system() == "Linux":
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue

            try:
                # Read process name from /proc/[pid]/comm
                with open(f"/proc/{pid}/comm") as f:
                    comm = f.read().strip()
                    if "gem5" in comm:
                        with open(f"/proc/{pid}/cmdline") as cmd_file:
                            cmdline = cmd_file.read().strip()
                            # exclude multisim processes that aren't simulations
                            # using both comm and cmdline. If only cmdline is
                            # used, we also pick up processes that have "gem5"
                            # in their names but aren't gem5 simulations.
                            if (
                                "gem5.utils.multisim" not in cmdline
                                and "multiprocessing.resource_tracker"
                                not in cmdline
                            ):
                                gem5_pids.append(int(pid))
            except (OSError, PermissionError):
                # Skip processes we can't read
                continue

    # Mac OS doesn't have /proc, so we have to do something else.
    elif platform.system() == "Darwin":
        gem5_pids_proc = subprocess.run(["pgrep", "gem5"], capture_output=True)
        if gem5_pids_proc.returncode != 0:
            raise ValueError("Could not list gem5 processes!")
        str_gem5_pids = gem5_pids_proc.stdout.decode("UTF-8").split("\n")
        # remove empty strings
        str_gem5_pids = [pid for pid in str_gem5_pids if pid]
        logger.debug(f"gem5 processes: {str_gem5_pids}")
        gem5_pids = []
        for pid in str_gem5_pids:
            cmdline = subprocess.run(
                ["ps", "-p", pid], capture_output=True
            ).stdout.decode("UTF-8")
            # exclude multisim processes
            if (
                "gem5.utils.multisim" not in cmdline
                and "multiprocessing.resource_tracker" not in cmdline
                and not (
                    "multiprocessing.spawn" in cmdline
                    and "--outdir" not in cmdline
                )
            ):
                gem5_pids.append(int(pid))
        logger.debug(f"gem5 processes that are simulations: {gem5_pids}")
    if not gem5_pids:
        raise ValueError("No gem5 process found")
    return gem5_pids


def find_gem5_single_pid() -> int:
    gem5_pids = find_gem5_pids()
    if len(gem5_pids) > 1:
        raise ValueError(f"Multiple gem5 processes found: {gem5_pids}")
    return gem5_pids[0]


def cleanup(signum=None, frame=None) -> None:
    """
    Clean up socket resources and handle process termination.

    Closes open sockets and removes socket files. When called as a signal
    handler, also exits the process.

    :param signum: Signal number that triggered handler (if called as handler)
    :param frame: Current stack frame (if called as handler)
    """
    global sock, socket_path
    if sock:
        sock.close()
    if socket_path and os.path.exists(socket_path):
        os.unlink(socket_path)
    if signum is not None:  # Only exit if signal handler
        sys.exit(0)


def receive_full_message(conn: socket.socket, timeout: float = 30.0) -> str:
    """
    Receive a complete message from a socket connection.

    Reads data in chunks until connection is closed or timeout occurs.
    Handles messages larger than the standard buffer size.

    :param conn: Connected socket to read from
    :param timeout: Maximum time to wait for data in seconds
    :return: Complete decoded message
    :raises TimeoutError: If no data received within timeout period
    """
    chunks = []
    while True:
        ready, _, _ = select.select([conn], [], [], timeout)
        if not ready:
            raise TimeoutError("Timeout waiting for complete response")

        chunk = conn.recv(4096)
        if not chunk:  # Connection closed by remote
            break
        chunks.append(chunk)

    return b"".join(chunks).decode()


def send_and_receive_hypercall(pid: int, function: str) -> str:
    """
    Send a hypercall to gem5 and wait for response.

    Creates Unix socket, sends hypercall via transmitter, and waits for
    response.

    :param pid: Process ID of gem5 instance
    :type pid: int
    :param function: Hypercall function to execute ('status'|'get_stats')
    :type function: str
    :return: JSON response from gem5
    :rtype: str
    :raises TimeoutError: If gem5 doesn't respond within timeout
    :raises ValueError: If payload or response format is invalid
    """
    global sock, socket_path

    socket_path = f"/tmp/hypercall_{pid}.sock"

    try:
        os.unlink(socket_path)
    except OSError:
        pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(socket_path)
    sock.listen(1)

    try:
        payload = json.dumps(
            {"function": function, "response_socket": socket_path}
        )
        send_signal(pid, 1000, payload)
        if function == "get_progress_bar_inst_count":
            # for the progress bar: gem5 may take > 30 s to respond, but we don't
            # want the dashboard to exit early. If we use timeout=None, however, the
            # dashboard might get stuck in some cases, such as if the simulation
            # exits while we are waiting.
            timeout = 2.0
        else:
            timeout = 30.0

        ready, _, _ = select.select([sock], [], [], timeout)
        if not ready:
            if function == "get_progress_bar_inst_count":
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    return "ended"
                else:
                    return "timeout"
            elif function == "get_progress_bar_init_stats":
                logger.debug(
                    "returning timeout for" " get_progress_bar_init_stats"
                )
                return "timeout"
            else:
                raise TimeoutError("Timeout waiting for gem5 response")

        conn, addr = sock.accept()
        try:
            return receive_full_message(conn, timeout=timeout)
        finally:
            conn.close()
    finally:
        cleanup()


def write_response_to_file(response: str, filename: str) -> None:
    """
    Write response to specified file.

    :param response: JSON response string
    :param filename: Path to output file
    """
    with open(filename, "w") as f:
        f.write(response)


def main():
    signal.signal(signal.SIGINT, cleanup)

    parser = argparse.ArgumentParser(description="Send hypercalls to gem5")
    parser.add_argument(
        "--pid",
        type=int,
        help="Process ID to send hypercall to "
        "(auto-detected if not specified)",
    )
    parser.add_argument(
        "function", choices=["status", "get_stats"], help="Function to execute"
    )
    parser.add_argument(
        "--output", type=str, help="Write response to specified file"
    )
    args = parser.parse_args()

    try:
        pid = args.pid if args.pid is not None else find_gem5_single_pid()
        response = send_and_receive_hypercall(pid, args.function)
        if args.output:
            write_response_to_file(response, args.output)
        else:
            print(f"Response: {response}")
    except (ValueError, TimeoutError) as e:
        logger.error(f"Error: {str(e)}")
        sys.exit(1)
    except OSError as e:
        logger.error(f"File error: {str(e)}")
        sys.exit(1)
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
