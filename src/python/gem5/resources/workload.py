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

from .downloader import get_workload_json_obj
from .resource import Resource

from typing import Dict, Any, Optional


class AbstractWorkload:
    """
    Workloads contain information needed to build a workload.

    A workload specifies a function and its parameters to run on a board to
    set a workload. Workload's are passed to board via the `AbstractBoard`'s
    `set_workload` function.

    The `AbstractBoard` has a `set_workload` function which accepts an
    AbstractWorkload. The `set_workload` function uses the `get_function_str`
    to determine which function should be called on the board and the
    `get_parameters` function specifies the parameters to be passed.

    Example
    -------

    ```py
    workload = CustomWorkload(
        function = "set_se_binary_workload",
        parameters = {
            "binary" : Resource("x86-print-this"),
            "arguments" : ["hello", 6]
        },
    )

    board.set_workload(workload)
    ```

    The above is the equivalent of:

    ```py
    board.set_se_binary_workload(
        binary = Resource("x86-print-this"),
        arguments = ["hello", 6],
    )
    ```

    Notes
    -----
    This class should not be used directly. Please use `Workload` or
    `CustomWorkload`.
    """

    def __init__(self, function: str, parameters: Dict[str, Any]) -> None:
        self._func = function
        self._params = parameters

    def get_function_str(self) -> str:
        """
        Returns the name of the workload function to be run.

        This function is called via the AbstractBoard's `set_workload`
        function. The parameters from the `get_parameters` function are passed
        to this function.
        """
        return self._func

    def get_parameters(self) -> Dict[str, Any]:
        """
        Returns a dictionary mapping the workload parameters to their values.

        These parameters are passed to the function specified by
        `get_function_str` via the AbstractBoard's `set_workload` function.
        """
        return self._params

    def set_parameter(self, parameter: str, value: Any) -> None:
        """
        Used to set or override a workload parameter

        :param parameter: The parameter of the function to set.
        :param value: The value to set to the parameter.
        """
        self._params[parameter] = value


class CustomWorkload(AbstractWorkload):
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
            "binary" : Resource("x86-print-this"),
            "arguments" : ["hello", 6]
        },
    )

    board.set_workload(workload)
    ```
    """

    def __init__(self, function: str, parameters: Dict[str, Any]) -> None:
        super().__init__(function=function, parameters=parameters)


class Workload(AbstractWorkload):
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
    workload.set_parameter("kernel", Resource("arm64-linux-kernel-4.14.134"))

    # We then set this workload to the board.
    board.set_workload(workload)
    ```

    """

    def __init__(
        self, workload_name: str, resource_directory: Optional[str] = None
    ) -> None:
        """
        This constructor will load the workload details from the workload with
        the given name/id.

        This function assumes the dictionary returned by the downloader's
        `get_workload_json_obj` is a dictionary. An example of the schema is
        shown below:

        ```json
        {
            "type" : "workload",
            "name" : "x86-ubuntu-18.04-echo-hello",
            "documentation" : "Description of workload here",
            "function" : "set_kernel_disk_workload",
            "resources" : {
                "kernel" : "x86-linux-kernel-5.4.49",
                "disk_image" : "x86-ubuntu-18.04-img"
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
            kernel = Resource("x86-linux-kernel-5.4.49"),
            disk_image = Resource("x86-ubuntu-18.04-img"),
            readfile_contents = "m5_exit; echo 'hello'; m5_exit",
        )
        ```

        :param workload_name: The name of the workload in the resources.json
        file to be loaded.
        :param resource_directory: An optional parameter that specifies where
        any resources should be download and accessed from. If None, a default
        location will be used. None by default.
        """
        workload_json = get_workload_json_obj(workload_name=workload_name)

        func = workload_json["function"]
        assert isinstance(func, str)

        params = {}
        if "resources" in workload_json:
            for key in workload_json["resources"].keys():
                assert isinstance(key, str)
                value = workload_json["resources"][key]
                assert isinstance(value, str)
                params[key] = Resource(
                    value, resource_directory=resource_directory
                )

        if "additional_params" in workload_json:
            for key in workload_json["additional_params"]:
                assert isinstance(key, str)
                params[key] = workload_json["additional_params"][key]

        super().__init__(function=func, parameters=params)
