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

from abc import abstractmethod

from actions.dashboard_action import DashboardAction
from hypercall_actions.dashboard_hypercall_request import send_gem5_action
from textual.widgets import Static


class Gem5HypercallAction(DashboardAction):
    """
    Base class for actions that communicate with gem5 via hypercall
    (signal 998).

    Subclasses must implement `action_code`. The default `execute` sends the
    hypercall and displays the response in the console. Override `execute` for
    custom pre/post-processing, or call `send_gem5_action` directly from
    within your own `execute` implementation.
    """

    @property
    @abstractmethod
    def action_code(self) -> str:
        """Action identifier sent to gem5 for dispatch, e.g. 'checkpoint'."""
        pass

    def get_arguments(self, pid: int) -> dict:
        """
        Override to pass additional key-value arguments to gem5.

        Args:
            pid: The process ID of the selected simulation.

        Returns:
            Dictionary of arguments to include in the hypercall payload.
        """
        return {}

    async def execute(self, pid: int, console: Static) -> None:
        console.update(
            f"Sending [bold]{self.action_code}[/bold] to PID {pid}..."
        )
        try:
            result = await send_gem5_action(
                pid, self.action_code, self.get_arguments(pid)
            )
            console.update(f"[bold green]Done:[/bold green] {result}")
        except TimeoutError:
            console.update(
                "[bold red]Error:[/bold red] gem5 did not respond in time."
            )
        except Exception as e:
            console.update(f"[bold red]Error:[/bold red] {str(e)}")
