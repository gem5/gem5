import argparse
import asyncio
import json
import logging
import os
import sys
import tempfile
from typing import (
    Dict,
    List,
    Optional,
)

_hypercall_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),  # helpers/
    "..",
    "..",
    "..",  # -> gem5-dashboard/ -> util/ -> gem5-root/
    "util",
    "hypercall_external_signal",
)
sys.path.insert(0, os.path.normpath(_hypercall_dir))
from orchestrator_request import find_gem5_pid
from transmitter import send_signal_async

logger = logging.getLogger(__name__)


async def _make_hypercall(
    pid: int, hypercall_id: int, extra_payload: dict
) -> str:
    """
    Core hypercall helper: opens a Unix socket, sends hypercall_id to pid
    with extra_payload merged into the payload, and waits for gem5's response.

    :param pid: Process ID of the target gem5 process
    :param hypercall_id: Dispatch ID gem5 uses to route the hypercall
    :param extra_payload: Additional fields to include in the JSON payload
    :return: Raw JSON string containing gem5's response
    :rtype: str
    """
    fd, socket_path = tempfile.mkstemp(
        suffix=".sock", prefix=f"gem5_{pid}_", dir="/tmp"
    )
    os.close(fd)
    os.unlink(socket_path)

    response: list[str] = []
    got_response = asyncio.Event()

    async def handle_connection(
        reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        try:
            data = await asyncio.wait_for(reader.read(-1), timeout=30.0)
            response.append(data.decode())
        finally:
            writer.close()
            got_response.set()

    server = await asyncio.start_unix_server(
        handle_connection, path=socket_path
    )
    try:
        payload = json.dumps({"response_socket": socket_path, **extra_payload})
        await send_signal_async(pid, hypercall_id, payload)
        await asyncio.wait_for(got_response.wait(), timeout=5.0)
        return response[0] if response else ""
    finally:
        server.close()
        await server.wait_closed()
        try:
            os.unlink(socket_path)
        except OSError:
            pass


async def get_gem5_data(
    pid: int,
    metrics: Optional[List[str]] = None,
    metrics_ext: Optional[str] = None,
) -> str:
    """
    Fetch data from gem5 used by the dashboard via hypercall (signal 999).

    :param pid: Process ID of the target gem5 process
    :param metrics: Optional list of metric names to collect.  When ``None``
        gem5 collects all registered metrics.
    :param metrics_ext: Optional path to a Python extension file that
        registers additional metrics via ``register_dashboard_metric``.
    :return: Raw JSON string containing gem5 metric data
    :rtype: str
    """
    extra: dict = {}
    if metrics is not None:
        extra["metrics"] = ",".join(metrics)
    if metrics_ext is not None:
        extra["metrics_ext"] = metrics_ext
    return await _make_hypercall(pid, 999, extra)


async def send_gem5_action(
    pid: int, action: str, arguments: Optional[Dict] = None
) -> str:
    """
    Send an action hypercall to gem5 (signal 998).

    :param pid: Process ID of the target gem5 process
    :param action: Action identifier for gem5 to dispatch (e.g. 'checkpoint')
    :param arguments: Optional key-value arguments passed alongside the action
    :return: Raw JSON string containing gem5's response
    :rtype: str
    """
    return await _make_hypercall(
        pid, 998, {"action": action, "arguments": arguments or {}}
    )


def main():
    parser = argparse.ArgumentParser(description="Send hypercalls to gem5")
    parser.add_argument(
        "--pid",
        type=int,
        help="Process ID to send hypercall to "
        "(auto-detected if not specified)",
    )
    parser.add_argument(
        "--metrics",
        nargs="*",
        help="Metric names to collect (default: all registered metrics)",
    )
    parser.add_argument(
        "--metrics-ext",
        help="Path to a Python extension file that registers additional metrics",
    )
    args = parser.parse_args()

    async def _run():
        try:
            pid = args.pid if args.pid is not None else find_gem5_pid()
            metrics: Optional[List[str]] = (
                args.metrics if args.metrics else None
            )
            response = await get_gem5_data(
                pid,
                metrics=metrics,
                metrics_ext=args.metrics_ext,
            )
            print(f"Response: {response}")
        except (ValueError, TimeoutError) as e:
            logger.error(f"Error: {str(e)}")
            sys.exit(1)
        except OSError as e:
            logger.error(f"File error: {str(e)}")
            sys.exit(1)

    asyncio.run(_run())


if __name__ == "__main__":
    main()
