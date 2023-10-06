# Copyright (c) 2022 The Regents of the University of California
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
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

from _m5 import core
from m5.util import warn

from .client import get_resource_json_obj
from .resource import obtain_resource
from .resource import WorkloadResource


def CustomWorkload(function: str, parameters: Dict[str, Any]):
    """
    A custom workload gem5 resource. It can be used to specify a custom,
    local workload.

    **Warning**: This `CustomWorkload` class is deprecated. It will be removed in a
    future release of gem5. Please use the `gem5.resources.resource.WorkloadResource`
    class instead.

    The class has been stealthily converted to a function which wraps the
    `WorkloadResource` class.
    """
    warn(
        "The `CustomWorkload` class is deprecated. Please use "
        "the `gem5.resources.resource.WorkloadResource` class instead.",
    )
    return WorkloadResource(function=function, parameters=parameters)


def Workload(
    workload_name: str,
    resource_directory: Optional[str] = None,
    resource_version: Optional[str] = None,
    clients: Optional[List] = None,
    gem5_version: Optional[str] = core.gem5Version,
):
    """
    **Warning**: The `Workload` class is deprecated. It will be removed in a
        future release of gem5. Please use the `gem5.resources.resource.WorkloadResource`
        class instead.

    The class has been stealthily converted to a function which wraps the
    `WorkloadResource` class.
    """
    warn(
        "`Workload` has been deprecated. Please use the `obtain_resource` "
        "function instead:\n\n"
        "```\n"
        "from gem5.resources.resource import obtain_resource\n"
        "workload = obtain_resource(\n"
        f'    resource_id="{workload_name}",\n'
        f'    resource_directory="{resource_directory}",\n'
        f'    gem5_version="{gem5_version}",\n'
        f"    clients={clients},\n"
        f"    resource_version={resource_version},\n"
        ")\n"
        "```",
    )
    return obtain_resource(
        workload_name,
        resource_directory=resource_directory,
        gem5_version=gem5_version,
        clients=clients,
        resource_version=resource_version,
    )
