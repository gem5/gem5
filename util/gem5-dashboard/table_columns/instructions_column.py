import json
import os
import subprocess
import time
from pathlib import Path

from textual import log

_instruction_cache = {}
_cache_timestamp = {}


def get_current_instructions(proc):
    """
    Get current instructions executed from gem5.
    Uses subprocess to avoid FD conflicts with Textual.
    """
    pid_int = proc.info["pid"]
    pid = str(pid_int)
    current_time = time.time()

    # Check cache - refresh every 5 seconds
    if pid in _instruction_cache:
        if current_time - _cache_timestamp.get(pid, 0) < 2:
            return _instruction_cache[pid]

    try:
        # Find the orchestrator-request.py script
        script_path = (
            Path(__file__).parent.parent / "helpers/orchestrator_request.py"
        )
        log(f"Calling orchestrator script for PID {pid}")

        # Call orchestrator-request.py as a subprocess
        result = subprocess.run(
            ["python3", str(script_path), "--pid", str(pid_int), "status"],
            capture_output=True,
            text=True,
            timeout=5,
        )

        if result.returncode != 0:
            log.error(f"Subprocess failed for PID {pid}: {result.stderr}")
            return _instruction_cache.get(pid, "N/A")

        # Parse response
        # The script outputs: "Response: {json}"
        response = result.stdout.strip()
        log(f"Raw stdout for PID {pid}: {response}")

        # Remove "Response: " prefix if present
        if response.startswith("Response: "):
            response = response[10:]

        data = json.loads(response)
        instructions = data.get("curr_instructions_executed", 0)
        result_str = f"{instructions:,}"

        # Cache result
        _instruction_cache[pid] = result_str
        _cache_timestamp[pid] = current_time

        log(f"Successfully cached instructions for PID {pid}: {result_str}")
        return result_str

    except subprocess.TimeoutExpired:
        log.error(f"Timeout getting instructions for PID {pid}")
        return _instruction_cache.get(pid, "Timeout")
    except json.JSONDecodeError as e:
        log.error(
            f"JSON decode error for PID {pid}: {e}, response was: {response}"
        )
        return _instruction_cache.get(pid, "N/A")
    except FileNotFoundError:
        log.error(f"orchestrator-request.py not found at {script_path}")
        return "Script Not Found"
    except Exception as e:
        log.error(f"Unexpected error for PID {pid}: {type(e).__name__}: {e}")
        return _instruction_cache.get(pid, "Error")
