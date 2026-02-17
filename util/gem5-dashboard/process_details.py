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

from typing import List

from dashboard_action import DashboardAction
from textual import work
from textual.app import ComposeResult
from textual.containers import Vertical
from textual.widgets import (
    Button,
    Label,
    Static,
)


class ProcessDetails(Vertical):
    """A sidebar widget to show details and actions for a selected PID."""

    DEFAULT_CSS = """
    ProcessDetails {
        width: 30%;
        border-left: solid green;
        padding: 1;
    }
    .hidden {
        display: none;
    }
    ProcessDetails.sidebar-hidden {
        display: none;
    }
    """
    current_pid: str | None = None

    def __init__(self, actions: List[DashboardAction], **kwargs):
        super().__init__(**kwargs)
        self.actions = actions

    def compose(self) -> ComposeResult:
        yield Label("Select a process...", id="lbl_status")
        yield Label("Output:", classes=" details-view hidden")
        yield Static("", id="output_log", classes="details-view hidden")

        for action in self.actions:
            yield Button(
                action.label,
                id=action.id,
                variant=action.variant,
                classes="details-view hidden",
            )

    def set_pid(self, pid: str):
        """Called by the main app when a row is clicked."""
        self.current_pid = pid
        self.query_one("#lbl_status", Label).update(f"Selected PID: {pid}")

        # Reveal the buttons
        self.query(".hidden").remove_class("hidden")
        self.query_one("#output_log", Static).update("")  # Clear old logs

    def get_current_pid(self) -> str | None:
        """Returns the currently selected PID, or None if none is selected."""
        return self.current_pid

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        if not self.current_pid:
            return

        # Find the action implementation that matches the button ID
        selected_action = next(
            (a for a in self.actions if a.id == event.button.id), None
        )

        if selected_action:
            self.run_action(selected_action)

    @work(exclusive=True)  # Runs in a worker thread so UI doesn't freeze
    async def run_action(self, action: DashboardAction):
        log_box = self.query_one("#output_log", Static)
        await action.execute(self.current_pid, log_box)

    def reset(self):
        """Resets the sidebar to its default empty state."""
        if self.current_pid is None:
            return  # Already reset

        self.current_pid = None
        self.query_one("#lbl_status", Label).update("Select a process...")
        self.query_one("#output_log", Static).update("")

        # Hide everything again
        self.query(".details-view").add_class("hidden")
