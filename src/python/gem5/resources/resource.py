# Copyright (c) 2021-2023 The Regents of the University of California
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

from abc import ABCMeta
import os
from pathlib import Path
from m5.util import warn, fatal
from _m5 import core

from .downloader import get_resource

from .looppoint import LooppointCsvLoader, LooppointJsonLoader
from ..isas import ISA, get_isa_from_str

from typing import Optional, Dict, Union, Type, Tuple, List, Any

from .client import get_resource_json_obj

"""
Resources are items needed to run a simulation, such as a disk image, kernel,
or binary. The gem5 project provides pre-built resources, with sources, at
<resources.gem5.org>. Here we provide the `AbstractResource` class and its
various implementations which are designed to encapsulate a resource for use
in the gem5 Standard Library.

These classes may be contructed directly. E.g.:

```python
binary = BinaryResource(local_path="/path/to/binary")
```

or obtained via the gem5-resources infrastructure with the `obtain_resource`
function:

```python
binary = obtain_resource("resource name here")
```
"""


class AbstractResource:
    """
    An abstract class which all Resource classes inherit from.
    """

    __metaclass__ = ABCMeta

    def __init__(
        self,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        local_path: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
    ):
        """
        :param local_path: The path on the host system where this resource is
        located.
        :param description: Description describing this resource. Not a
        required parameter. By default is None.
        :param source: The source (as in "source code") for this resource. This
        string should navigate users to where the source for this resource
        may be found. Not a required parameter. By default is None.
        :param resource_version: Version of the resource itself.
        """

        if local_path and not os.path.exists(local_path):
            raise Exception(
                f"Local path specified for resource, '{local_path}', does not "
                "exist."
            )
        self._id = id
        self._local_path = local_path
        self._description = description
        self._source = source
        self._version = resource_version

    def get_category_name(cls) -> str:
        raise NotImplementedError

    def __str__(self):
        message = (
            f"{self.get_category_name()}({self._id}, {self._version})\n"
            "For more information, please visit "
            f"https://resources.gem5.org/resources/{self._id}?"
            f"version={self._version}"
        )
        return message

    def get_resource_version(self) -> str:
        """Returns the version of the resource."""
        return self._version

    def get_local_path(self) -> Optional[str]:
        """Returns the local path of the resource."""
        return self._local_path

    def get_description(self) -> Optional[str]:
        """Returns description associated with this resource."""
        return self._description

    def get_source(self) -> Optional[str]:
        """Returns information as to where the source for this resource may be
        found.
        """
        return self._source


class FileResource(AbstractResource):
    """A resource consisting of a single file."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        if not os.path.isfile(local_path):
            raise Exception(
                f"FileResource path specified, '{local_path}', is not a file."
            )

        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

    def get_category_name(cls) -> str:
        return "FileResource"


class DirectoryResource(AbstractResource):
    """A resource consisting of a directory."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        if not os.path.isdir(local_path):
            raise Exception(
                f"DirectoryResource path specified, {local_path}, is not a "
                "directory."
            )

        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

    def get_category_name(cls) -> str:
        return "DirectoryResource"


class DiskImageResource(FileResource):
    """A Disk Image resource."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        root_partition: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )
        self._root_partition = root_partition

    def get_root_partition(self) -> Optional[str]:
        """Returns, if applicable, the Root Partition of the disk image."""
        return self._root_partition

    def get_category_name(cls) -> str:
        return "DiskImageResource"


class BinaryResource(FileResource):
    """A binary resource."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        architecture: Optional[Union[ISA, str]] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

        self._architecture = None
        if architecture:
            if isinstance(architecture, str):
                self._architecture = get_isa_from_str(architecture)
            elif isinstance(architecture, ISA):
                self._architecture = architecture

    def get_category_name(cls) -> str:
        return "BinaryResource"

    def get_architecture(self) -> Optional[ISA]:
        """Returns the ISA this binary is compiled to."""
        return self._architecture


class BootloaderResource(BinaryResource):
    """A bootloader resource."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        architecture: Optional[Union[ISA, str]] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            architecture=architecture,
            source=source,
            resource_version=resource_version,
        )

    def get_category_name(cls) -> str:
        return "BootloaderResource"


class GitResource(DirectoryResource):
    """A git resource."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

    def get_category_name(cls) -> str:
        return "GitResource"


class KernelResource(BinaryResource):
    """A kernel resource."""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        architecture: Optional[Union[ISA, str]] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            architecture=architecture,
            resource_version=resource_version,
        )

    def get_category_name(cls) -> str:
        return "KernelResource"


class CheckpointResource(DirectoryResource):
    """A checkpoint resource. The following directory structure is expected:

    <local_path>:
       - board.physmem.store0.pmem
       - m5.cpt
    """

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

    def get_category_name(cls) -> str:
        return "CheckpointResource"


class SimpointResource(AbstractResource):
    """A simpoint resource. This resource stores all information required to
    perform a Simpoint creation and restore. It contains the Simpoint, the
    Simpoint interval, the weight for each Simpoint, the full warmup length,
    and the warmup length for each Simpoint.
    """

    def __init__(
        self,
        resource_version: Optional[str] = None,
        simpoint_interval: int = None,
        simpoint_list: List[int] = None,
        weight_list: List[float] = None,
        warmup_interval: int = 0,
        id: Optional[str] = None,
        workload_name: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        local_path: Optional[str] = None,
        **kwargs,
    ):
        """
        :param simpoint_interval: The simpoint interval.
        :param simpoint_list: The simpoint list.
        :param weight_list: The weight list.
        :param warmup_interval: The warmup interval. Default to zero (a value
        of zero means effectively not set).
        :param workload_name: Simpoints are typically associated with a
        particular workload due to their dependency on chosen input parameters.
        This field helps backtrack to that resource if required. This should
        relate to a workload "name" field in the resource.json file.
        """

        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

        self._weight_list = weight_list
        self._simpoint_list = simpoint_list
        self._simpoint_interval = simpoint_interval
        self._warmup_interval = warmup_interval
        self._workload_name = workload_name

        self._simpoint_start_insts = list(
            inst * simpoint_interval for inst in self.get_simpoint_list()
        )

        if self._warmup_interval != 0:
            self._warmup_list = self._set_warmup_list()
        else:
            self._warmup_list = [0] * len(self.get_simpoint_start_insts)

    def get_simpoint_list(self) -> List[int]:
        """Returns the a list containing all the Simpoints for the workload."""
        return self._simpoint_list

    def get_simpoint_start_insts(self) -> List[int]:
        """Returns a lst containing all the Simpoint starting instrunction
        points for the workload. This was calculated by multiplying the
        Simpoint with the Simpoint interval when it was generated."""
        return self._simpoint_start_insts

    def get_warmup_interval(self) -> int:
        """Returns the instruction length of the warmup interval."""
        return self._warmup_interval

    def get_weight_list(self) -> List[float]:
        """Returns the list that contains the weight for each Simpoint. The
        order of the weights matches that of the list returned by
        `get_simpoint_list(). I.e. `get_weight_list()[3]` is the weight for
        simpoint `get_simpoint_list()[3]`."""
        return self._weight_list

    def get_simpoint_interval(self) -> int:
        """Returns the Simpoint interval value."""
        return self._simpoint_interval

    def get_warmup_list(self) -> List[int]:
        """Returns the a list containing the warmup length for each Simpoint.
        Each warmup length in this list corresponds to the Simpoint at the same
        index in `get_simpoint_list()`. I.e., `get_warmup_list()[4]` is the
        warmup length for Simpoint `get_simpoint_list()[4]`."""
        return self._warmup_list

    def get_workload_name(self) -> Optional[str]:
        """Return the workload name this Simpoint is associated with."""
        return self._workload_name

    def _set_warmup_list(self) -> List[int]:
        """
        This function uses the warmup_interval, fits it into the
        simpoint_start_insts, and outputs a list of warmup instruction lengths
        for each SimPoint.

        The warmup instruction length is calculated using the starting
        instruction of a SimPoint to minus the warmup_interval and the ending
        instruction of the last SimPoint. If it is less than 0, then the warmup
        instruction length is the gap between the starting instruction of a
        SimPoint and the ending instruction of the last SimPoint.
        """
        warmup_list = []
        for index, start_inst in enumerate(self.get_simpoint_start_insts()):
            warmup_inst = start_inst - self.get_warmup_interval()
            if warmup_inst < 0:
                warmup_inst = start_inst
            else:
                warmup_inst = self.get_warmup_interval()
            warmup_list.append(warmup_inst)
            # change the starting instruction of a SimPoint to include the
            # warmup instruction length
            self._simpoint_start_insts[index] = start_inst - warmup_inst
        return warmup_list

    def get_category_name(cls) -> str:
        return "SimpointResource"


class LooppointCsvResource(FileResource, LooppointCsvLoader):
    """This Looppoint resource used to create a Looppoint resource from a
    pinpoints CSV file"""

    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        FileResource.__init__(
            self,
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )
        LooppointCsvLoader.__init__(self, pinpoints_file=Path(local_path))

    def get_category_name(cls) -> str:
        return "LooppointCsvResource"


class LooppointJsonResource(FileResource, LooppointJsonLoader):
    def __init__(
        self,
        local_path: str,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        region_id: Optional[Union[str, int]] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        FileResource.__init__(
            self,
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )
        LooppointJsonLoader.__init__(
            self, looppoint_file=local_path, region_id=region_id
        )

    def get_category_name(cls) -> str:
        return "LooppointJsonResource"


class SimpointDirectoryResource(SimpointResource):
    """A Simpoint diretory resource. This Simpoint Resource assumes the
    existance of a directory containing a simpoint file and a weight file."""

    def __init__(
        self,
        local_path: str,
        simpoint_file: str,
        weight_file: str,
        simpoint_interval: int,
        warmup_interval: int,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        workload_name: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        """
        :param simpoint_file: The Simpoint file. This file is a list of
        Simpoints, each on its own line. It should map 1-to-1 to the weights
        file.
        :param weight_file: The Simpoint weights file. This file is a list of
        weights, each on its own line.
        """
        self._simpoint_file = simpoint_file
        self._weight_file = weight_file

        # This is a little hack. The functions `get_simpoint_file` and
        # `get_weight_file` use the local path, so we set it here despite it
        # also being set in the `AbstractResource` constructor. This isn't
        # elegant but does not harm.
        self._local_path = local_path
        (
            simpoint_list,
            weight_list,
        ) = self._get_weights_and_simpoints_from_file()

        super().__init__(
            simpoint_interval=simpoint_interval,
            simpoint_list=simpoint_list,
            weight_list=weight_list,
            warmup_interval=warmup_interval,
            workload_name=workload_name,
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

    def get_simpoint_file(self) -> Path:
        """Return the Simpoint File path."""
        return Path(Path(self._local_path) / self._simpoint_file)

    def get_weight_file(self) -> Path:
        """Returns the Weight File path."""
        return Path(Path(self._local_path) / self._weight_file)

    def _get_weights_and_simpoints_from_file(
        self,
    ) -> Tuple[List[int], List[int]]:
        """This is a helper function to extract the weights and simpoints from
        the files.
        """
        simpoint_weight_pair = []
        with open(self.get_simpoint_file()) as simpoint_file, open(
            self.get_weight_file()
        ) as weight_file:
            while True:
                line = simpoint_file.readline()
                if not line:
                    break
                interval = int(line.split(" ", 1)[0])
                line = weight_file.readline()
                if not line:
                    fatal("not engough weights")
                weight = float(line.split(" ", 1)[0])
                simpoint_weight_pair.append((interval, weight))
        simpoint_weight_pair.sort(key=lambda obj: obj[0])
        # use simpoint to sort

        weight_list = []
        simpoint_list = []
        for simpoint, weight in simpoint_weight_pair:
            simpoint_list.append(simpoint)
            weight_list.append(weight)
        return simpoint_list, weight_list

    def get_category_name(cls) -> str:
        return "SimpointDirectoryResource"


class WorkloadResource(AbstractResource):
    """A workload resource. This resource is used to specify a workload to run
    on a board. It contains the function to call and the parameters to pass to
    that function.
    """

    def __init__(
        self,
        function: str = None,
        id: Optional[str] = None,
        resource_version: Optional[str] = None,
        description: Optional[str] = None,
        source: Optional[str] = None,
        local_path: Optional[str] = None,
        parameters: Optional[Dict[str, Any]] = {},
        **kwargs,
    ):
        """
        :param function: The function to call on the board.
        :param parameters: The parameters to pass to the function.
        """

        super().__init__(
            local_path=local_path,
            id=id,
            description=description,
            source=source,
            resource_version=resource_version,
        )

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

    def get_category_name(cls) -> str:
        return "WorkloadResource"


def obtain_resource(
    resource_id: str,
    resource_directory: Optional[str] = None,
    download_md5_mismatch: bool = True,
    resource_version: Optional[str] = None,
    clients: Optional[List] = None,
    gem5_version=core.gem5Version,
    to_path: Optional[str] = None,
    quiet: bool = False,
) -> AbstractResource:
    """
    This function primarily serves as a factory for resources. It will return
    the correct `AbstractResource` implementation based on the resource
    requested.

    :param resource_name: The name of the gem5 resource as it appears under the
    "id" field in the `resource.json` file.
    :param resource_directory: The location of the directory in which the
    resource is to be stored. If this parameter is not set, it will set to
    the environment variable `GEM5_RESOURCE_DIR`. If the environment is not
    set it will default to `~/.cache/gem5` if available, otherwise the CWD.
    **Note**: This argument is ignored if the `to_path` parameter is specified.
    :param download_md5_mismatch: If the resource is present, but does not
    have the correct md5 value, the resoruce will be deleted and
    re-downloaded if this value is True. Otherwise an exception will be
    thrown. True by default.
    :param resource_version: Version of the resource itself.
    Not a required parameter. None by default.
    :param clients: A list of clients to search for the resource. If this
    parameter is not set, it will default search all clients.
    :param gem5_version: The gem5 version to use to filter incompatible
    resource versions. By default set to the current gem5 version. If None,
    this filtering is not performed.
    :param to_path: The path to which the resource is to be downloaded. If
    None, the resource will be downloaded to the resource directory with
    the file/directory name equal to the ID of the resource. **Note**: Usage
    of this parameter will override the `resource_directory` parameter.
    :param quiet: If True, suppress output. False by default.
    """

    # Obtain the resource object entry for this resource
    resource_json = get_resource_json_obj(
        resource_id,
        resource_version=resource_version,
        clients=clients,
        gem5_version=gem5_version,
    )

    # If the "url" field is specified, the resoruce must be downloaded.
    if "url" in resource_json and resource_json["url"]:

        # If the `to_path` parameter is set, we use that as the path to which
        # the resource is to be downloaded. Otherwise, default to the
        # `resource_directory` parameter plus the resource ID.
        if not to_path:
            # If the `resource_directory` parameter is not set via this
            # function, we heck the "GEM5_RESOURCE_DIR" environment variable.
            # If this too is not set we call `_get_default_resource_dir()` to
            # determine where the resource directory is, or should be, located.
            if resource_directory == None:
                resource_directory = os.getenv(
                    "GEM5_RESOURCE_DIR", _get_default_resource_dir()
                )

            # Small checks here to ensure the resource directory is valid.
            if os.path.exists(resource_directory):
                if not os.path.isdir(resource_directory):
                    raise Exception(
                        "gem5 resource directory, "
                        "'{}', exists but is not a directory".format(
                            resource_directory
                        )
                    )

            # This is the path to which the resource is to be stored.
            to_path = os.path.join(resource_directory, resource_id)

        assert to_path

        # Here we ensure the directory in which the resource is to be stored
        # is created.
        #
        # `exist_ok=True` here as, occasionally, if multiple instance of gem5
        # are started simultaneously, a race condition can exist to create the
        # resource directory. Without `exit_ok=True`, threads which lose this
        # race will thrown a `FileExistsError` exception. `exit_ok=True`
        # ensures no exception is thrown.
        try:
            Path(to_path).parent.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            fatal(
                f"Recursive creation of the directory "
                f"'{Path(to_path).parent.absolute}' failed. \n"
                f"Perhaps the path specified, '{to_path}', is incorrect?\n"
                f"Failed with Exception:\n{e}"
            )

        # Download the resource if it does not already exist.
        get_resource(
            resource_name=resource_id,
            to_path=to_path,
            download_md5_mismatch=download_md5_mismatch,
            resource_version=resource_version,
            clients=clients,
            gem5_version=gem5_version,
            quiet=quiet,
        )

    # Obtain the type from the JSON. From this we will determine what subclass
    # of `AbstractResource` we are to create and return.
    resources_category = resource_json["category"]

    if resources_category == "resource":
        # This is a stop-gap measure to ensure to work with older versions of
        # the "resource.json" file. These should be replaced with their
        # respective specializations ASAP and this case removed.
        if "root_partition" in resource_json:
            # In this case we should return a DiskImageResource.
            root_partition = resource_json["root_partition"]
            return DiskImageResource(
                local_path=to_path,
                root_partition=root_partition,
                **resource_json,
            )
        return CustomResource(local_path=to_path)

    assert resources_category in _get_resource_json_type_map
    resource_class = _get_resource_json_type_map[resources_category]

    if resources_category == "workload":
        # This parses the "resources" and "additional_params" fields of the
        # workload resource into a dictionary of AbstractResource objects and
        # strings respectively.
        params = {}
        if "resources" in resource_json:
            for key in resource_json["resources"].keys():
                assert isinstance(key, str)
                value = resource_json["resources"][key]
                assert isinstance(value, str)
                params[key] = obtain_resource(
                    value,
                )
        if "additional_params" in resource_json:
            for key in resource_json["additional_params"].keys():
                assert isinstance(key, str)
                value = resource_json["additional_params"][key]
                assert isinstance(value, str)
                params[key] = value
        resource_json["parameters"] = params
    # Once we know what AbstractResource subclass we are using, we create it.
    # The fields in the JSON object are assumed to map like-for-like to the
    # subclass contructor, so we can pass the resource_json map directly.
    return resource_class(local_path=to_path, **resource_json)


def _get_default_resource_dir() -> str:
    """
    Obtain the default gem5 resources directory on the host system. This
    function will iterate through sensible targets until it finds one that
    works on the host system.

    :returns: The default gem5 resources directory.
    """
    test_list = [
        # First try `~/.cache/gem5`.
        os.path.join(Path.home(), ".cache", "gem5"),
        # Last resort, just put things in the cwd.
        os.path.join(Path.cwd(), "resources"),
    ]

    for path in test_list:
        if os.path.exists(path):  # If the path already exists...
            if os.path.isdir(path):  # Check to see the path is a directory.
                return path  # If so, the path is valid and can be used.
        else:  # If the path does not exist, try to create it.
            try:
                os.makedirs(path, exist_ok=False)
                return path
            except OSError:
                continue  # If the path cannot be created, then try another.

    raise Exception("Cannot find a valid location to download resources")


# The following classes exist to preserve backwards functionality between the
# API for obtaining resources in v21.1.0 and prior.


class CustomResource(AbstractResource):
    """
    A custom gem5 resource. This can be used to encapsulate a resource provided
    by a gem5 user as opposed to one available within the gem5 resources
    repository.

    **Warning**: This class is deprecated and will be removed in future
    releases of gem5. Please use the correct `AbstractResource` subclass
    instead.
    """

    def __init__(self, local_path: str, metadata: Dict = {}):
        """
        :param local_path: The path of the resource on the host system.
        :param metadata: Add metadata for the custom resource. **Warning:**
        As of v22.1.1, this parameter is not used.
        """
        warn(
            "The `CustomResource` class is deprecated. Please use an "
            "`AbstractResource` subclass instead."
        )
        if bool(metadata):  # Empty dicts cast to False
            warn(
                "the `metadata` parameter was set via the `CustomResource` "
                "constructor. This parameter is not used."
            )
        super().__init__(local_path=local_path)


class CustomDiskImageResource(DiskImageResource):
    """
    A custom disk image gem5 resource. It can be used to specify a custom,
    local disk image.

    **Warning**: This class is deprecated and will be removed in future
    releases of gem5. Please use the `DiskImageResource` class instead. This
    class is merely a wrapper for it.
    """

    def __init__(
        self,
        local_path: str,
        resource_version: Optional[str] = None,
        root_partition: Optional[str] = None,
        metadata: Dict = {},
    ):
        """
        :param local_path: The path of the disk image on the host system.
        :param root_partition: The root disk partition to use.
        :param metadata: Metadata for the resource. **Warning:** As of "
        "v22.1.1, this parameter is not used.
        :param resource_version: Version of the resource itself.
        """
        warn(
            "The `CustomDiskImageResource` class is deprecated. Please use "
            "`DiskImageResource` instead."
        )
        if bool(metadata):  # Empty dicts cast to False
            warn(
                "the `metadata` parameter was set via the "
                "`CustomDiskImageResource` constructor. This parameter is not "
                "used."
            )
        super().__init__(
            local_path=local_path,
            root_partition=root_partition,
            resource_version=resource_version,
        )


def Resource(
    resource_id: str,
    resource_directory: Optional[str] = None,
    download_md5_mismatch: bool = True,
    resource_version: Optional[str] = None,
    clients: Optional[List[str]] = None,
) -> AbstractResource:
    """
    This function was created to maintain backwards compability for v21.1.0
    and prior releases of gem5 where `Resource` was a class.

    In the interests of gem5-resource specialization, the `Resource` class
    has been dropped. Instead users are advized to use the `obtain_resource`
    function which will return the correct `AbstractResource` implementation.
    This function (disguised as a class) wraps this function.
    """

    warn(
        "`Resource` has been deprecated. Please use the `obtain_resource` "
        "function instead."
    )

    return obtain_resource(
        resource_id=resource_id,
        resource_directory=resource_directory,
        download_md5_mismatch=download_md5_mismatch,
        resource_version=resource_version,
        clients=clients,
    )


_get_resource_json_type_map = {
    "disk-image": DiskImageResource,
    "binary": BinaryResource,
    "kernel": KernelResource,
    "checkpoint": CheckpointResource,
    "git": GitResource,
    "bootloader": BootloaderResource,
    "file": FileResource,
    "directory": DirectoryResource,
    "simpoint": SimpointResource,
    "simpoint-directory": SimpointDirectoryResource,
    "resource": Resource,
    "looppoint-pinpoint-csv": LooppointCsvResource,
    "looppoint-json": LooppointJsonResource,
    "workload": WorkloadResource,
}
