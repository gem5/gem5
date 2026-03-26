# Copyright (c) 2026 The Regents of the University of California
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

import json
import subprocess
import time
from pathlib import Path
from typing import Dict

from textual import log


class Gem5DataManager:
    """
    Centralized manager for fetching and caching gem5 process data.

    This class handles all communication with gem5 processes via hypercalls,
    maintaining a cache to avoid redundant requests. It provides a single
    point of access for gem5-specific data needed by dashboard columns.
    """

    def __init__(self, cache_ttl: int = 5):
        """
        Initialize the data manager.

        Args:
            cache_ttl: Time-to-live for cached data in seconds (default: 5)
        """
        self._cache: Dict[str, dict] = {}
        self._cache_time: Dict[str, float] = {}
        self._cache_ttl = cache_ttl

    def get_data(self, pid: str) -> dict:
        """
        Get gem5 data for a specific process.

        Returns cached data if still valid, otherwise fetches fresh data
        from gem5 via hypercall.

        Args:
            pid: Process ID as a string

        Returns:
            Dictionary containing gem5 data. Returns empty dict on failure.
        """
        if self._is_cache_valid(pid):
            return self._cache[pid]

        data = self._fetch_from_gem5(pid)
        self._cache[pid] = data
        self._cache_time[pid] = time.time()
        return data

    def invalidate(self, pid: str) -> None:
        """
        Invalidate cached data for a specific process.

        Args:
            pid: Process ID as a string
        """
        if pid in self._cache:
            del self._cache[pid]
        if pid in self._cache_time:
            del self._cache_time[pid]

    def clear_cache(self) -> None:
        """Clear all cached data."""
        self._cache.clear()
        self._cache_time.clear()

    def _is_cache_valid(self, pid: str) -> bool:
        """
        Check if cached data for a process is still valid.

        Args:
            pid: Process ID as a string

        Returns:
            True if cache exists and is not expired, False otherwise
        """
        if pid not in self._cache:
            return False

        elapsed = time.time() - self._cache_time.get(pid, 0)
        return elapsed < self._cache_ttl

    def _fetch_from_gem5(self, pid: str) -> dict:
        """
        Fetch dashboard data from gem5 via hypercall.

        Uses subprocess to call orchestrator_request.py, which sends a
        hypercall to gem5 and waits for the response via Unix socket.

        Args:
            pid: Process ID as a string

        Returns:
            Dictionary containing gem5 response data, or empty dict on failure
        """
        try:
            script_path = (
                Path(__file__).parent
                / "helpers/dashboard_hypercall_request.py"
            )
            log(f"Fetching gem5 data for PID {pid}")

            # Call dashboard_hypercall_request.py as a subprocess
            result = subprocess.run(
                ["python3", str(script_path), "--pid", pid],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode != 0:
                log.error(f"Subprocess failed for PID {pid}: {result.stderr}")
                return {}

            # Parse response
            # The script outputs: "Response: {json}"
            response = result.stdout.strip()
            log(f"Raw response for PID {pid}: {response}")

            # Remove "Response: " prefix if present
            if response.startswith("Response: "):
                response = response[10:]

            data = json.loads(response)
            log(f"Successfully fetched gem5 data for PID {pid}")
            return data

        except subprocess.TimeoutExpired:
            log.error(f"Timeout fetching gem5 data for PID {pid}")
            return {}
        except json.JSONDecodeError as e:
            log.error(
                f"JSON decode error for PID {pid}: {e}, response: {response}"
            )
            return {}
        except FileNotFoundError:
            log.error(f"orchestrator-request.py not found at {script_path}")
            return {}
        except Exception as e:
            log.error(
                f"Unexpected error fetching data for PID {pid}: "
                f"{type(e).__name__}: {e}"
            )
            return {}
