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
Dashboard metrics extension file.

Add custom metrics here by defining a function that accepts a
``Simulator`` instance and returns a JSON-serialisable value, then
registering it with :func:`register_dashboard_metric`.

This file is loaded by every running gem5 process on each dashboard
hypercall (signal 999).  It is re-executed only when its modification
time changes, so edits take effect on the next refresh without
restarting gem5 or the dashboard.

To add a fully custom column to the dashboard table:

**Step 1 — define and register the metric here**::

    def my_metric(simulator):
        # simulator is a gem5.simulate.simulator.Simulator instance.
        # Return any JSON-serialisable value (int, float, str, dict, …)
        return simulator.get_current_tick()

    register_dashboard_metric("my_metric", my_metric)

**Step 2 — create a column function** in
``util/gem5-dashboard/table_columns/my_metric_column.py``::

    def get_my_metric(proc, gem5_data=None):
        if gem5_data is None:
            gem5_data = {}
        value = gem5_data.get("my_metric", None)
        return str(value) if value is not None else "N/A"

**Step 3 — register the column** in
``util/gem5-dashboard/table_column_map.py``::

    from table_columns.my_metric_column import get_my_metric

    COLUMNS = [
        ...
        {
            "name": "My Metric",
            "key": "My Metric",
            "width": 15,
            "func": get_my_metric,
            "required_metrics": ["my_metric"],  # must match the registered name
        },
    ]

The ``required_metrics`` list tells ``Gem5DataManager`` which metric names
to request from gem5 on each hypercall.  Only metrics listed there are
fetched, keeping payloads small.

No gem5 rebuild is required — this file is loaded at runtime.
"""

import os

from gem5.simulate.dashboard_metrics import register_dashboard_metric


def sim_ticks(simulator):
    """Return the current tick and simulated time in nanoseconds."""
    import _m5.core

    tick = simulator.get_current_tick()
    tps = _m5.core.getClockFrequency()
    return {"ticks": tick}


register_dashboard_metric("sim_ticks", sim_ticks)
