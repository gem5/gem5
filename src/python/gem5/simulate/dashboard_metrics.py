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

"""
Central metric registry for the gem5 dashboard.

Built-in metrics are registered at module load. Additional metrics can be
registered at runtime by passing a Python extension file path via the
``metrics_ext`` field of hypercall 999's payload. The extension file is
expected to call :func:`register_dashboard_metric` for each metric it defines.

The extension file is re-executed only when its modification time changes,
so a running gem5 process will automatically pick up edits without restart.

Example extension file (``my_metrics.py``)::

    import os
    import psutil
    from gem5.simulate.dashboard_metrics import register_dashboard_metric

    def host_rss_mb(simulator):
        return round(
            psutil.Process(os.getpid()).memory_info().rss / 1024 ** 2, 2
        )

    register_dashboard_metric("host_rss_mb", host_rss_mb)

Pass the file path to the dashboard via ``--metrics-ext /path/to/my_metrics.py``
and it will be forwarded to every running gem5 process in the next hypercall.
"""

import importlib.util
import os
from typing import (
    Any,
    Callable,
    Dict,
    List,
    Optional,
)

# Maps metric name -> callable(simulator) -> Any
_METRICS: Dict[str, Callable] = {}

# Maps extension file path -> mtime at last successful load.
# Re-execution is skipped when mtime is unchanged.
_loaded_extensions: Dict[str, float] = {}


def register_dashboard_metric(name: str, fn: Callable) -> None:
    """Register *fn* as the collector for the metric named *name*.

    If a metric with that name already exists it is replaced. This allows
    extension files to override built-in metrics when needed.
    """
    action = "overriding" if name in _METRICS else "registering"
    print(f"[dashboard_metrics] {action} metric: {name!r}")
    _METRICS[name] = fn


def collect(
    simulator, requested: Optional[List[str]] = None
) -> Dict[str, Any]:
    """Collect and return metric values from gem5.

    :param simulator: The running simulator object passed by the exit handlers
    :param requested: List of metric names to collect.  When ``None`` all
        registered metrics are collected.
    :returns: A dict mapping each metric name to its value.  If a metric
        raises an exception its value is ``{"error": "<message>"}``.
    """
    names = requested if requested is not None else list(_METRICS.keys())

    out: Dict[str, Any] = {}
    for name in names:
        fn = _METRICS.get(name)
        if fn is None:
            out[name] = {"error": f"Unknown metric: {name}"}
            continue
        try:
            out[name] = fn(simulator)
        except Exception as exc:
            out[name] = {"error": str(exc)}
    return out


def available() -> List[str]:
    """Return the names of all currently registered metrics."""
    return list(_METRICS.keys())


def load_extension(path: str) -> None:
    """Load a Python extension file that registers additional metrics.

    The file is executed only if it has not been loaded before, or if its
    modification time has changed since the last load. This means a user can
    edit the extension file and the next hypercall will automatically pick up
    the changes without restarting gem5.

    The extension file is expected to call
    :func:`register_dashboard_metric` for each metric it defines.

    :param path: Absolute or relative path to the Python extension file.
    """
    try:
        mtime = os.path.getmtime(path)
    except OSError as exc:
        print(f"Error checking extension file {path}: {exc}")
        return

    if _loaded_extensions.get(path) == mtime:
        return  # unchanged since last load

    spec = importlib.util.spec_from_file_location("_gem5_dashboard_ext", path)
    if spec is None or spec.loader is None:
        print(f"Error creating spec for extension file {path}")
        return

    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:
        import traceback

        print(f"Error executing extension file {path}: {exc}")
        traceback.print_exc()
        return

    _loaded_extensions[path] = mtime


# Pre-defined built-in metrics


def _workload(simulator) -> str:
    if not simulator.get_workload():
        return "N/A"
    return simulator.get_workload().get_id()


def _curr_instructions_executed(simulator) -> int:
    return simulator.get_instruction_count()


register_dashboard_metric("workload", _workload)
register_dashboard_metric(
    "curr_instructions_executed", _curr_instructions_executed
)
