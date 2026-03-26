import argparse
import json
import logging
import os
import select
import signal
import socket
import sys

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


def get_gem5_data(pid: int) -> dict:
    """
    Fetch data from gem5 used by the dashboard via hypercall.
    :param pid: Process ID of the target gem5 process
    :return: Dictionary containing gem5 data
    :rtype: dict

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
            }
        )
        send_signal(pid, 999, payload)

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


def main():
    signal.signal(signal.SIGINT, cleanup)

    parser = argparse.ArgumentParser(description="Send hypercalls to gem5")
    parser.add_argument(
        "--pid",
        type=int,
        help="Process ID to send hypercall to "
        "(auto-detected if not specified)",
    )
    args = parser.parse_args()

    try:
        pid = args.pid if args.pid is not None else find_gem5_pid()
        response = get_gem5_data(pid)
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
