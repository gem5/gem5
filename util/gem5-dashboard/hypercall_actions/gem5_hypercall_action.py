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
