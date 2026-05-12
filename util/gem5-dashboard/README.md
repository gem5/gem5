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

There are two cases: your column reads only OS-level data (no gem5 needed), or it reads a gem5 metric.

---

#### Case A — OS-level data only (no gem5 metric needed)

**Step 1: Create a column file in `table_columns/`**

```python
# table_columns/cpu_percent_column.py
def get_cpu_percent(proc, gem5_data=None):
    try:
        return f"{proc.cpu_percent():.1f}%"
    except Exception:
        return "N/A"
```

**Step 2: Register in `table_column_map.py`** with an empty `required_metrics` list

```python
from table_columns.cpu_percent_column import get_cpu_percent

COLUMNS = [
    ...
    {"name": "CPU%", "key": "CPU%", "width": 8,
     "func": get_cpu_percent, "required_metrics": []},
]
```

---

#### Case B — gem5 metric data

This requires three steps: define the metric inside gem5, expose it in a column function, then declare the dependency in the column map.

**Step 1: Register the metric in `dashboard_metrics_ext.py`**

Open `util/gem5-dashboard/dashboard_metrics_ext.py` and add a function that accepts a `Simulator` and returns a JSON-serialisable value, then register it:

```python
def my_metric(simulator):
    return simulator.get_current_tick()

register_dashboard_metric("my_metric", my_metric)
```

No gem5 rebuild is needed — the file is reloaded automatically whenever its modification time changes.

**Step 2: Create a column file in `table_columns/`**

The function receives the metric value under the key you registered:

```python
# table_columns/my_metric_column.py
def get_my_metric(proc, gem5_data=None):
    if gem5_data is None:
        gem5_data = {}
    value = gem5_data.get("my_metric", None)
    return str(value) if value is not None else "N/A"
```

- `gem5_data` is an empty dict when gem5 is unreachable — always guard with a default.

**Step 3: Register in `table_column_map.py`** with the metric name in `required_metrics`

```python
from table_columns.my_metric_column import get_my_metric

COLUMNS = [
    ...
    {"name": "My Metric", "key": "My Metric", "width": 15,
     "func": get_my_metric, "required_metrics": ["my_metric"]},
]
```

`Gem5DataManager` reads `required_metrics` across all active columns and requests only that set from gem5 on each hypercall — no other changes needed.

> **Built-in metrics** (`workload`, `curr_instructions_executed`) are always available without `dashboard_metrics_ext.py`. Only custom metrics need to be added there.

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
