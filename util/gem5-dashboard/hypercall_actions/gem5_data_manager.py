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
import time
from pathlib import Path
from typing import (
    Dict,
    List,
    Optional,
)

from hypercall_actions.dashboard_hypercall_request import get_gem5_data
from textual import log


class Gem5DataManager:
    """
    Centralized manager for fetching and caching gem5 process data.

    This class handles all communication with gem5 processes via hypercalls,
    maintaining a cache to avoid redundant requests. It provides a single
    point of access for gem5-specific data needed by dashboard columns.

    The set of metrics requested from gem5 is derived from the
    ``required_metrics`` field of each active column definition.  Only the
    metrics that are actually displayed are fetched, keeping hypercall
    payloads small.

    An optional ``metrics_ext`` path can be provided to forward a Python
    extension file to every gem5 process on each hypercall, allowing users
    to add custom metrics without modifying gem5 or restarting it.
    """

    def __init__(
        self,
        cache_ttl: int = 5,
        columns: Optional[List[dict]] = None,
    ):
        """
        Initialize the data manager.

        Args:
            cache_ttl: Time-to-live for cached data in seconds (default: 5)
            columns: List of column definition dicts from ``table_column_map``.
                Each entry may contain a ``required_metrics`` list; the union
                of all such lists is sent as the ``metrics`` argument on every
                hypercall.
        """
        self._cache: Dict[int, dict] = {}
        self._cache_time: Dict[int, float] = {}
        self._cache_ttl = cache_ttl

        # Auto-detect the fixed extension file shipped alongside the dashboard.
        # Users add custom metrics by editing that file; no path config needed.
        _ext_path = (
            Path(__file__).parent.parent / "dashboard_metrics_ext.py"
        ).resolve()
        self._metrics_ext: Optional[str] = (
            str(_ext_path) if _ext_path.exists() else None
        )

        # Derive the deduplicated list of metric names needed by active columns.
        seen = set()
        requested: List[str] = []
        for col in columns or []:
            for name in col.get("required_metrics", []):
                if name not in seen:
                    seen.add(name)
                    requested.append(name)
        # None means "collect all", only pass an explicit list when columns actually declare their requirements.
        self._requested_metrics: Optional[List[str]] = (
            requested if requested else None
        )

    async def get_data(self, pid: int) -> dict:
        """
        Get gem5 data for a specific process.

        Returns cached data if still valid, otherwise fetches fresh data
        from gem5 via hypercall.

        Args:
            pid: Process ID

        Returns:
            Dictionary containing gem5 data. Returns empty dict on failure.
        """
        if self._is_cache_valid(pid):
            return self._cache[pid]

        data = await self._fetch_from_gem5(pid)
        self._cache[pid] = data
        self._cache_time[pid] = time.time()
        return data

    def invalidate(self, pid: int) -> None:
        """
        Invalidate cached data for a specific process.

        Args:
            pid: Process ID
        """
        if pid in self._cache:
            del self._cache[pid]
        if pid in self._cache_time:
            del self._cache_time[pid]

    def clear_cache(self) -> None:
        """Clear all cached data."""
        self._cache.clear()
        self._cache_time.clear()

    def _is_cache_valid(self, pid: int) -> bool:
        """
        Check if cached data for a process is still valid.

        Args:
            pid: Process ID

        Returns:
            True if cache exists and is not expired, False otherwise
        """
        if pid not in self._cache:
            return False

        elapsed = time.time() - self._cache_time.get(pid, 0)
        return elapsed < self._cache_ttl

    async def _fetch_from_gem5(self, pid: int) -> dict:
        """
        Fetch dashboard data from gem5 via hypercall (in-process, fully async).

        Args:
            pid: Process ID

        Returns:
            Dictionary containing gem5 response data, or empty dict on failure
        """
        response = ""
        try:
            log(f"Fetching gem5 data for PID {pid}")
            response = await get_gem5_data(
                pid,
                metrics=self._requested_metrics,
                metrics_ext=self._metrics_ext,
            )
            data = json.loads(response)
            log(f"Successfully fetched gem5 data for PID {pid}")
            return data
        except TimeoutError:
            log.error(f"Timeout fetching gem5 data for PID {pid}")
            return {}
        except json.JSONDecodeError as e:
            log.error(
                f"JSON decode error for PID {pid}: {e}, response: {response}"
            )
            return {}
        except Exception as e:
            log.error(
                f"Unexpected error fetching data for PID {pid}: "
                f"{type(e).__name__}: {e}"
            )
            return {}
