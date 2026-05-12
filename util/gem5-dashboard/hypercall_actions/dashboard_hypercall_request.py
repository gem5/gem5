import argparse
import json
import logging
import os
import select
import signal
import socket
import sys
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
from orchestrator_request import (
    cleanup,
    find_gem5_pid,
    receive_full_message,
)
from transmitter import send_signal

logger = logging.getLogger(__name__)
socket_path = None
sock = None


def _make_hypercall(pid: int, signal_num: int, extra_payload: dict) -> str:
    """
    Core hypercall helper: opens a Unix socket, sends signal_num to pid
    with extra_payload merged into the payload, and waits for gem5's response.

    :param pid: Process ID of the target gem5 process
    :param signal_num: Signal number to send to gem5
    :param extra_payload: Additional fields to include in the JSON payload
    :return: Raw JSON string containing gem5's response
    :rtype: str
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
            {
                "response_socket": socket_path,
                **extra_payload,
            }
        )
        send_signal(pid, signal_num, payload)

        ready, _, _ = select.select([sock], [], [], 30.0)
        if not ready:
            raise TimeoutError("Timeout waiting for gem5 response")

        conn, addr = sock.accept()
        try:
            return receive_full_message(conn)
        finally:
            conn.close()
    finally:
        cleanup()


def get_gem5_data(
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
    return _make_hypercall(pid, 999, extra)


def send_gem5_action(
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
    return _make_hypercall(
        pid, 998, {"action": action, "arguments": arguments or {}}
    )


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
        "--metrics",
        nargs="*",
        help="Metric names to collect (default: all registered metrics)",
    )
    parser.add_argument(
        "--metrics-ext",
        help="Path to a Python extension file that registers additional metrics",
    )
    args = parser.parse_args()

    try:
        pid = args.pid if args.pid is not None else find_gem5_pid()
        metrics: Optional[List[str]] = args.metrics if args.metrics else None
        response = get_gem5_data(
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
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
