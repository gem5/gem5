# gem5 Dashboard

A terminal-based GUI (TUI) for monitoring and managing running gem5 processes. Built with Python and Textual.

## Getting Started

1. **Create a virtual environment** (recommended):

    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

2. **Install dependencies:**

    ```bash
    pip install -r requirements.txt
    ```

### Usage

Run the dashboard from the root directory of the utility:

```bash
python3 gem5_dashboard.py
```

* **View Processes:** The table automatically refreshes every 2 seconds to show running gem5 simulations.
* **Inspect Details:** Click on a row to view the full command, process status, and available actions in the sidebar.
* **Kill Process:** Select a process and click the "Kill Process" button in the sidebar to terminate it.

---

## Architecture Overview

The dashboard uses a modular design with clean separation between data sources:

### Data Flow

1. **Process Data (psutil)**: Fast OS-level info (PID, User, Status, Command)
2. **gem5 Data (hypercalls)**: Simulation-specific data (Instructions, Ticks, etc.)
   - Centralized via `Gem5DataManager` class
   - Single hypercall per process per refresh cycle
   - Automatic caching with 2-second TTL

### Key Components

- **`gem5_dashboard.py`**: Main TUI application
- **`gem5_data_manager.py`**: Centralized gem5 data fetching and caching
- **`table_columns/`**: Column implementations (receive both psutil and gem5 data)
- **`actions/`**: User actions (Kill, etc.)
- Communicates with gem5 using utilities from `hypercall_actions/dashboard_hypercall_request.py`

---

## Extending the Dashboard

The dashboard uses a plugin-based architecture. You can add new columns or actions without touching the core UI code.

---

### Adding a New Column

Columns receive two data sources: fast OS-level data from `psutil` and gem5-specific data fetched once per refresh via hypercall. Your function picks what it needs from either source.

**Step 1: Create a column file in `table_columns/`**

Name the file after the value it displays (e.g., `ticks_column.py`). The function signature must be `(proc, gem5_data=None) -> str`.

```python
# table_columns/ticks_column.py
def get_ticks(proc, gem5_data=None):
    if gem5_data is None:
        gem5_data = {}
    ticks = gem5_data.get("tick", None)
    return f"{ticks:,}" if ticks else "N/A"
```

- Use `proc` (a `psutil.Process` info dict) for OS-level fields like PID, status, or CPU usage.
- Use `gem5_data` for simulation-specific fields. These come from the gem5 hypercall response; if gem5 is unreachable, this will be an empty dict — always guard with a default.

**Step 2: Register in `table_column_map.py`**

```python
from table_columns.ticks_column import get_ticks

COLUMNS = [
    {"name": "PID",   "key": "PID",   "width": 10, "func": get_pid},
    {"name": "Ticks", "key": "Ticks", "width": 15, "func": get_ticks},  # new
    # ...
]
```

The `"width"` key is optional. Omit it to let the column stretch to fill remaining space.

**Step 3 (if needed): Ensure `gem5DashboardExitHandler` returns the field**

If your column reads from `gem5_data`, make sure the gem5-side hypercall handler `gem5DashboardExitHandler` includes that key in its JSON response (e.g., `"tick": 123456`). The `Gem5DataManager` handles caching and fetching automatically.

---

### Adding a New Action (OS-level)

Use this path for actions that do not need to communicate with gem5

**Step 1: Create the action file in `actions/`**

Name the file after the action (e.g., `dump_stats_action.py`). Inherit from `DashboardAction` and implement `label`, `id`, `variant`, and `execute`.

```python
# actions/dump_stats_action.py
from actions.dashboard_action import DashboardAction
from textual.widgets import Static

class DumpStatsAction(DashboardAction):
    label = "Dump Stats"
    id = "act_dump_stats"
    variant = "primary"  # 'primary', 'default', 'success', 'warning', 'error'

    async def execute(self, pid: str, console: Static) -> None:
        console.update(f"Dumping stats for PID {pid}...")
        # ... your logic here ...
        console.update("[bold green]Done![/bold green]")
```

- `id` must be unique across all actions
- `console.update(...)` accepts Rich markup for coloured output.

**Step 2: Register in `action_registry.py`**

```python
from actions.kill_process_action import KillAction
from actions.dump_stats_action import DumpStatsAction  # 1. import

ENABLED_ACTIONS = [
    KillAction(),
    DumpStatsAction(),  # 2. add to list
]
```

The dashboard automatically renders a button for each entry and wires up the click event.

---

### Adding a New Action (gem5-communicating)

Use this path when an action needs to send a command to a running gem5 process.

The hypercall sends a JSON payload of the form:

```json
{ "action": "checkpoint", "arguments": { "path": "/tmp/cpt" } }
```

gem5 receives signal 998, reads `action` field to handle logic for that action.

**Step 1: Create the action file in `actions/`**

Inherit from `Gem5HypercallAction` instead of `DashboardAction`. Implement `action_code` and optionally `get_arguments`.

```python
# actions/checkpoint_action.py
from hypercall_actions.gem5_hypercall_action import Gem5HypercallAction

class CheckpointAction(Gem5HypercallAction):
    label = "Checkpoint"
    id = "act_checkpoint"
    variant = "success"

    @property
    def action_code(self) -> str:
        return "checkpoint"

    def get_arguments(self, pid: str) -> dict:
        # Optional: return extra fields included in the hypercall payload.
        return {"path": f"/tmp/gem5_cpt_{pid}"}
```

`Gem5HypercallAction` provides a default `execute` that:
1. Sends the hypercall via `send_gem5_action(pid, action_code, arguments)`
2. Displays the response or error in the console

Override `execute` if you need custom pre/post-processing around the hypercall.

**Step 2: Register in `action_registry.py`**

```python
from actions.kill_process_action import KillAction
from actions.checkpoint_action import CheckpointAction  # 1. import

ENABLED_ACTIONS = [
    KillAction(),
    CheckpointAction(),  # 2. add to list
]
```

**Step 3: Update `gem5DashboardActionExitHandler` in gem5**

On the gem5 side, update the `gem5DashboardActionExitHandler` with the new `action` field from the payload to return the required data. The dashboard expects a JSON response dict back.
