# Copyright (c) 2026 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
