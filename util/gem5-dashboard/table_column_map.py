from table_columns.command_column import get_command
from table_columns.instructions_column import get_current_instructions
from table_columns.pid_column import get_pid
from table_columns.status_column import get_status
from table_columns.user_column import get_user

# To be used in gem5-dashboard to define table columns for displaying gem5 processes
# Each column is defined with its display name, key, width(optional), and a function to extract the relevant data.
COLUMNS = [
    {"name": "PID", "key": "PID", "width": 10, "func": get_pid},
    {"name": "User", "key": "User", "width": 15, "func": get_user},
    {"name": "Status", "key": "Status", "width": 12, "func": get_status},
    {
        "name": "Instructions",
        "key": "Instructions",
        "width": 20,
        "func": get_current_instructions,
    },
    {"name": "Command", "key": "Command", "func": get_command},
]
