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

from .resource import obtain_resource, WorkloadResource
from .client import get_resource_json_obj

from _m5 import core

from typing import Dict, Any, List, Optional

"""
    A workload specified locally (i.e., not via gem5-resources as with the
    `Workload` class). Here the user specifies the function and the parameters
    to be passed.

    Usage
    -----

    ```py
    workload = CustomWorkload(
        function = "set_se_binary_workload",
        parameters = {
            "binary" : obtain_resource("x86-print-this"),
            "arguments" : ["hello", 6]
        },
    )

    board.set_workload(workload)
    ```
"""


def CustomWorkload(function: str, parameters: Dict[str, Any]) -> None:
    return WorkloadResource(function=function, parameters=parameters)


"""
    The `Workload` class loads a workload's information from gem5-resources
    based on a name/id passed via the constructor.

    Usage
    -----

    ```py
    # Determine what workload we want to run.
    workload = Workload("example-workload-id")

    # Optionally we can override a parameter in the workload. In this example
    # we are going to run this workload with a difference kernel.
    workload.set_parameter("kernel",
        obtain_resource("arm64-linux-kernel-4.14.134")
    )

    # We then set this workload to the board.
    board.set_workload(workload)
    ```

"""


def Workload(
    workload_name: str,
    resource_directory: Optional[str] = None,
    resource_version: Optional[str] = None,
    clients: Optional[List] = None,
    gem5_version: Optional[str] = core.gem5Version,
):
    """
    This constructor will load the workload details from the workload with
    the given name/id.

    This function assumes the dictionary returned by the downloader's
    `get_workload_json_obj` is a dictionary. An example of the schema is
    shown below:

    ```json
    {
        "category" : "workload",
        "id" : "x86-ubuntu-18.04-echo-hello",
        "description" : "Description of workload here",
        "function" : "set_kernel_disk_workload",
        "resources" : {
            "kernel" : "x86-linux-kernel-5.4.49",
            "disk-image" : "x86-ubuntu-18.04-img"
        },
        "additional_params" : {
            "readfile_contents" : "m5_exit; echo 'hello'; m5_exit"
        }
    }
    ```

    This resource will result in the equivalent of the following action
    being taken:

    ```python
    board.set_kernel_disk_workload(
        kernel = obtain_resource("x86-linux-kernel-5.4.49"),
        disk-image = obtain_resource("x86-ubuntu-18.04-img"),
        readfile_contents = "m5_exit; echo 'hello'; m5_exit",
    )
    ```

    :param workload_name: The name of the workload in the resources.json
    file to be loaded.
    :param resource_directory: An optional parameter that specifies where
    any resources should be download and accessed from. If None, a default
    location will be used. None by default.
    :param gem5_version: The gem5 version for the Workload to be loaded.
    By default, the current gem5 version is used. This will filter
    resources which are incompatible with the current gem5 version. If
    None, no filtering will be done.
    """
    return obtain_resource(
        workload_name,
        resource_directory=resource_directory,
        gem5_version=gem5_version,
        clients=clients,
        resource_version=resource_version,
    )
