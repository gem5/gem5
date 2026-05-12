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

import os

import psutil
from action_registry import ENABLED_ACTIONS
from hypercall_actions.gem5_data_manager import Gem5DataManager
from process_details import ProcessDetails
from table_column_map import COLUMNS
from textual.app import (
    App,
    ComposeResult,
)
from textual.containers import Horizontal
from textual.widgets import (
    DataTable,
    Footer,
    Header,
)


class Gem5Dashboard(App):
    CSS = """
    DataTable {
        width: 1fr;
        height: 100%;
        border-right: solid $primary;
    }

    DataTable.full-width {
        width: 100%;
        border-right: none;
    }

    """
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("r", "refresh", "Refresh"),
        ("t", "toggle_sidebar", "Toggle Sidebar"),
    ]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Initialize the centralized gem5 data manager with the active column
        # set so it knows which metrics to request from gem5 on each hypercall.
        self.gem5_data_manager = Gem5DataManager(cache_ttl=2, columns=COLUMNS)

    def compose(self) -> ComposeResult:
        yield Header()
        # Use Horizontal to split screen: Table (Left) | Details (Right)
        with Horizontal():
            yield DataTable(id="proc_table")
            yield ProcessDetails(actions=ENABLED_ACTIONS, id="sidebar")
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one(DataTable)
        table.cursor_type = "row"
        table.zebra_stripes = True
        for col in COLUMNS:
            table.add_column(
                col["name"], key=col["key"], width=col.get("width")
            )

        self.update_processes()
        self.set_interval(2, self.update_processes)

    def action_toggle_sidebar(self) -> None:
        """Toggle sidebar with 't' key"""
        sidebar = self.query_one(ProcessDetails)
        table = self.query_one(DataTable)
        sidebar.toggle_class("sidebar-hidden")
        table.toggle_class("full-width")

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        # Get the PID (row key) from the event
        pid = event.row_key.value
        sidebar = self.query_one(ProcessDetails)
        table = self.query_one(DataTable)

        # Hide sidebar if same PID is selected again
        # Otherwise, show sidebar with new PID
        if sidebar.get_current_pid() == pid:
            sidebar.toggle_class("sidebar-hidden")
            table.toggle_class("full-width")
        else:
            sidebar.set_pid(pid)
            sidebar.remove_class("sidebar-hidden")
            table.remove_class("full-width")

    def update_processes(self) -> None:
        """
        Fetches running gem5 processes, applies filters, and updates the TUI table.
        """
        table = self.query_one(DataTable)
        sidebar = self.query_one(ProcessDetails)
        current_pids = set()

        # We iterate over all processes provided by psutil
        # attrs defines what data we want to fetch to avoid extra syscalls
        attrs = ["pid", "name", "username", "status", "cmdline", "create_time"]

        for proc in psutil.process_iter(attrs):
            try:
                # Skip the dashboard process itself
                if proc.pid == os.getpid():
                    continue

                # 1. Ownership Filter: psutil handles this internally usually,
                # but we explicitly check if the process belongs to the current user.
                if proc.info["username"] != psutil.Process().username():
                    continue

                # 2. Name Filter: Check if 'gem5' is in the name
                # We also check cmdline as gem5
                cmd_list = proc.info["cmdline"] or []
                cmd_str = " ".join(cmd_list)
                gem5_binaries = ["gem5.opt", "gem5.debug", "gem5.fast"]

                if not any(
                    gem5_bin in proc.info["name"] for gem5_bin in gem5_binaries
                ) and not any(
                    gem5_bin in cmd_str for gem5_bin in gem5_binaries
                ):
                    print(
                        f"process name: {proc.info['name']}, cmdline: {proc.info['cmdline']}"
                    )

                    continue

                # 3. Multisim & Wrapper Filter (logic from original dashboard PR)
                # We filter out the orchestration scripts to show only the actual simulation
                exclusion_patterns = [
                    "gem5.utils.multisim",
                    "multiprocessing.resource_tracker",
                    "multiprocessing.spawn",
                ]

                if any(pattern in cmd_str for pattern in exclusion_patterns):
                    continue

                # 4. macOS specific check (Ported from original dashboard PR)
                # If it's a spawn process without an outdir, ignore it
                if (
                    "multiprocessing.spawn" in cmd_str
                    and "--outdir" not in cmd_str
                ):
                    continue

                pid = str(proc.info["pid"])
                current_pids.add(pid)

                # Fetch gem5 data once for this process
                gem5_data = self.gem5_data_manager.get_data(pid)

                row_data = []
                for col in COLUMNS:
                    try:
                        value = col["func"](proc, gem5_data)
                        row_data.append(value)
                    except Exception:
                        row_data.append("N/A")

                if pid in table.rows:
                    for col, value in zip(COLUMNS, row_data):
                        table.update_cell(pid, col["key"], value)
                else:
                    table.add_row(*row_data, key=pid)

            except (
                psutil.NoSuchProcess,
                psutil.AccessDenied,
                psutil.ZombieProcess,
            ):
                # Process died or we lost permission while iterating
                continue

        # Cleanup: Remove rows for processes that are no longer running and reset sidebar if needed
        rows_to_remove = [
            row_key for row_key in table.rows if row_key not in current_pids
        ]
        for row_key in rows_to_remove:
            table.remove_row(row_key)
            # Also invalidate cache for removed processes
            self.gem5_data_manager.invalidate(row_key)

        if sidebar.current_pid and sidebar.current_pid not in current_pids:
            sidebar.reset()


if __name__ == "__main__":
    app = Gem5Dashboard()
    app.run()
