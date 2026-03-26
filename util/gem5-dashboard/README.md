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
- Communicates with gem5 using utilities from `helpers/dashboard_hypercall_request.py`

---

## Extending the Dashboard

The dashboard uses a plugin-based architecture. You can add new features without touching the core UI code.

### Adding a New Column

To add a new column (e.g., "Simulation Ticks"):

1. **Create column file** in `table_columns/`:

```python
# table_columns/ticks_column.py
def get_ticks(proc, gem5_data=None):
    """Get current simulation tick count."""
    if gem5_data is None:
        gem5_data = {}

    ticks = gem5_data.get("tick", None)
    return f"{ticks:,}" if ticks else "N/A"
```

2. **Register in `table_column_map.py`**:

```python
from table_columns.ticks_column import get_ticks

COLUMNS = [
    {"name": "PID", "key": "PID", "width": 10, "func": get_pid},
    {"name": "Ticks", "key": "Ticks", "width": 15, "func": get_ticks},  # Add here
    # ... other columns
]
```

3. **Ensure gem5 returns the data**: The hypercall response must include the field (e.g., `"tick": 123456`).

The `Gem5DataManager` handles fetching, and your column just extracts the value.

### Adding a New Action

To add new actions (like "Dump Stats" or "Pause Simulation"):

### Step 1: Create the Action Class

Create a new file in the `actions/` folder (e.g., `actions/my_action.py`). Your class must inherit from `DashboardAction` and implement the required properties.

```python
from textual.widgets import Static
from dashboard_action import DashboardAction

class MyAction(DashboardAction):
    # 1. Label: Text shown on the button
    label = "my action"

    # 2. ID: Unique identifier for the button
    id = "act_my_action"

    # 3. Variant: Button color ('primary', 'default', 'success', 'warning', 'error')
    variant = "primary"

    async def execute(self, pid: str, console: Static) -> None:
        """
        Define what happens when the button is clicked.

        Args:
            pid: The process ID of the selected simulation.
            console: The UI widget to print status messages to.
        """
        console.update(f"Checking config for PID {pid}...")

        console.update("My action complete!")
```

### Step 2: Register the Action

Open `action_registry.py`, import your new class, and add it to the list.

```python
from actions.kill_process_action import KillAction
from actions.my_new_action import MyAction  # <--- 1. Import

ENABLED_ACTIONS = [
    KillAction(),
    MyAction(),  # <--- 2. Add to list
]
```

The dashboard will automatically render the button in the sidebar and wire up the click event.
