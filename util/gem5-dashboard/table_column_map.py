from table_columns.command_column import get_command
from table_columns.instructions_column import get_current_instructions
from table_columns.pid_column import get_pid
from table_columns.sim_ticks_column import get_sim_ticks
from table_columns.status_column import get_status
from table_columns.user_column import get_user
from table_columns.workload_id_column import get_workload_id

# To be used in gem5-dashboard to define table columns for displaying gem5 processes
# Each column is defined with its display name, key, width(optional), a function to
# extract the relevant data, and a list of gem5 metric names the column needs
# (required_metrics). Columns that rely only on psutil leave required_metrics empty.
# Gem5DataManager aggregates required_metrics across all active columns and requests
# only that set from gem5 on each hypercall.
COLUMNS = [
    {
        "name": "PID",
        "key": "PID",
        "width": 10,
        "func": get_pid,
        "required_metrics": [],
    },
    {
        "name": "User",
        "key": "User",
        "width": 15,
        "func": get_user,
        "required_metrics": [],
    },
    {
        "name": "Status",
        "key": "Status",
        "width": 12,
        "func": get_status,
        "required_metrics": [],
    },
    {
        "name": "Instructions",
        "key": "Instructions",
        "width": 20,
        "func": get_current_instructions,
        "required_metrics": ["curr_instructions_executed"],
    },
    {
        "name": "Workload ID",
        "key": "Workload ID",
        "width": 40,
        "func": get_workload_id,
        "required_metrics": ["workload"],
    },
    {
        "name": "Ticks",
        "key": "Ticks",
        "width": 20,
        "func": get_sim_ticks,
        "required_metrics": ["sim_ticks"],
    },
    {
        "name": "Command",
        "key": "Command",
        "func": get_command,
        "required_metrics": [],
    },
]
