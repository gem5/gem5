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
from m5.util import warn

from .downloader import get_resource, get_resources_json_obj

from ..isas import ISA, get_isa_from_str

from typing import Optional, Dict, Union, Type

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
        local_path: Optional[str] = None,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
    ):
        """
        :param local_path: The path on the host system where this resource is
        located
        :param documentation: Documentation describing this resource. Not a
        required parameter. By default is None.
        :param source: The source (as in "source code") for this resource. This
        string should navigate users to where the source for this resource
        may be found. Not a required parameter. By default is None.
        """

        if local_path and not os.path.exists(local_path):
            raise Exception(
                f"Local path specified for resource, '{local_path}', does not "
                "exist."
            )

        self._local_path = local_path
        self._documentation = documentation
        self._source = source

    def get_local_path(self) -> Optional[str]:
        """Returns the local path of the resource."""
        return self._local_path

    def get_documentation(self) -> Optional[str]:
        """Returns documentation associated with this resource."""
        return self._documentation

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
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        if not os.path.isfile(local_path):
            raise Exception(
                f"FileResource path specified, '{local_path}', is not a file."
            )

        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
        )


class DirectoryResource(AbstractResource):
    """A resource consisting of a directory."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
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
            documentation=documentation,
            source=source,
        )


class DiskImageResource(FileResource):
    """A Disk Image resource."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        root_partition: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
        )
        self._root_partition = root_partition

    def get_root_partition(self) -> Optional[str]:
        """Returns, if applicable, the Root Partition of the disk image."""
        return self._root_partition


class BinaryResource(FileResource):
    """A binary resource."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        architecture: Optional[Union[ISA, str]] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
        )

        self._architecture = None
        if architecture:
            if isinstance(architecture, str):
                self._architecture = get_isa_from_str(architecture)
            elif isinstance(architecture, ISA):
                self._architecture = architecture

    def get_architecture(self) -> Optional[ISA]:
        """Returns the ISA this binary is compiled to."""
        return self._architecture


class BootloaderResource(BinaryResource):
    """A bootloader resource."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        architecture: Optional[Union[ISA, str]] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            architecture=architecture,
            source=source,
        )


class GitResource(DirectoryResource):
    """A git resource."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
        )


class KernelResource(BinaryResource):
    """A kernel resource."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        architecture: Optional[Union[ISA, str]] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
            architecture=architecture,
        )


class CheckpointResource(DirectoryResource):
    """A checkpoint resource. The following directory structure is expected:

    <local_path>:
       - board.physmem.store0.pmem
       - m5.cpt
    """

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
        )


class SimpointResource(DirectoryResource):
    """A simpoint resource."""

    def __init__(
        self,
        local_path: str,
        documentation: Optional[str] = None,
        source: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            local_path=local_path,
            documentation=documentation,
            source=source,
        )


def obtain_resource(
    resource_name: str,
    resource_directory: Optional[str] = None,
    download_md5_mismatch: bool = True,
) -> AbstractResource:
    """
    This function primarily serves as a factory for resources. It will return
    the correct `AbstractResource` implementation based on the resource
    requested, by referencing the "resource.json" file (by default, that hosted
    at https://resources.gem5.org/resources.json). In addition to this, this
    function will download the resource if not detected in the
    `resource_directory`.

    :param resource_name: The name of the gem5 resource as it appears under the
    "name" field in the `resource.json` file.
    :param resource_directory: The location of the directory in which the
    resource is to be stored. If this parameter is not set, it will set to
    the environment variable `GEM5_RESOURCE_DIR`. If the environment is not
    set it will default to `~/.cache/gem5` if available, otherwise the CWD.
    :param download_md5_mismatch: If the resource is present, but does not
    have the correct md5 value, the resoruce will be deleted and
    re-downloaded if this value is True. Otherwise an exception will be
    thrown. True by default.
    """

    # Obtain the JSON resource entry for this resource
    resource_json = get_resources_json_obj(resource_name)

    to_path = None
    # If the "url" field is specified, the resoruce must be downloaded.
    if "url" in resource_json and resource_json["url"]:

        # If the `resource_directory` parameter is not set via this function, we
        # check the "GEM5_RESOURCE_DIR" environment variable. If this too is not
        # set we call `_get_default_resource_dir()` to determine where the
        # resource directory is, or should be, located.
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
        else:
            # `exist_ok=True` here as, occasionally, if multiple instance of
            # gem5 are started simultaneously, a race condition can exist to
            # create the resource directory. Without `exit_ok=True`, threads
            # which lose this race will thrown a `FileExistsError` exception.
            # `exit_ok=True` ensures no exception is thrown.
            os.makedirs(resource_directory, exist_ok=True)

        # This is the path to which the resource is to be stored.
        to_path = os.path.join(resource_directory, resource_name)

        # Download the resource if it does not already exist.
        get_resource(
            resource_name=resource_name,
            to_path=os.path.join(resource_directory, resource_name),
            download_md5_mismatch=download_md5_mismatch,
        )

    # Obtain the type from the JSON. From this we will determine what subclass
    # of `AbstractResource` we are to create and return.
    resources_type = resource_json["type"]

    if resources_type == "resource":
        # This is a stop-gap measure to ensure to work with older versions of
        # the "resource.json" file. These should be replaced with their
        # respective specializations ASAP and this case removed.
        if (
            "additional_metadata" in resource_json
            and "root_partition" in resource_json["additional_metadata"]
        ):
            # In this case we should return a DiskImageResource.
            root_partition = resource_json["additional_metadata"][
                "root_partition"
            ]
            return DiskImageResource(
                local_path=to_path, root_partition=root_partition
            )
        return CustomResource(local_path=to_path)

    assert resources_type in _get_resource_json_type_map
    resource_class = _get_resource_json_type_map[resources_type]

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
        root_partition: Optional[str] = None,
        metadata: Dict = {},
    ):
        """
        :param local_path: The path of the disk image on the host system.
        :param root_partition: The root disk partition to use.
        :param metadata: Metadata for the resource. **Warning:** As of "
        "v22.1.1, this parameter is not used.
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
        super().__init__(local_path=local_path, root_partition=root_partition)


def Resource(
    resource_name: str,
    resource_directory: Optional[str] = None,
    download_md5_mismatch: bool = True,
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
        resource_name=resource_name,
        resource_directory=resource_directory,
        download_md5_mismatch=download_md5_mismatch,
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
    "resource": Resource,
}
