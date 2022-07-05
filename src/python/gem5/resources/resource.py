# Copyright (c) 2021 The Regents of the University of California
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

from .downloader import get_resource, get_resources_json_obj

from typing import Optional, Dict

"""
A Resource object encapsulates a gem5 resource. Resources are items needed to
run a simulation, such as a disk image, kernel, or binary. The gem5 project
provides pre-built resources, with sources, at <resources.gem5.org>.

The purpose of this encapsulation is two fold:

1. It allows automatic retrieval of gem5 resources. E.g., specifying a resource
   which is not local will initiate a download.
2. It provides a location where code may be added to record the resources used
   within a simulation. At present this is a TODO work-item.
"""


class AbstractResource:

    __metaclass__ = ABCMeta

    def __init__(self, local_path: str, metadata: Dict = {}):
        self._local_path = local_path
        self._metadata = metadata

    def get_local_path(self) -> str:
        return self._local_path

    def get_metadata(self) -> Dict:
        """
        Returns the raw data from this resource, as seen in the
        `resources.json` file. A user may specify the metadata of a local
        resource.
        """
        return self._metadata


class CustomResource(AbstractResource):
    """
    A custom gem5 resource. This can be used to encapsulate a resource provided
    by a gem5 user as opposed to one available within the gem5 resources
    repository.
    """

    def __init__(self, local_path: str, metadata: Dict = {}):
        """
        :param local_path: The path of the resource on the host system.
        :param metadata: Add metadata for the custom resource.
        """
        super().__init__(local_path=local_path, metadata=metadata)


class CustomDiskImageResource(CustomResource):
    """
    A custom disk image gem5 resource. It can be used to specify a custom,
    local disk image.
    """

    def __init__(
        self,
        local_path: str,
        disk_root_partition: Optional[str] = None,
        metadata: Dict = {},
    ):
        """
        :param local_path: The path of the disk image on the host system.
        :param disk_root_partition: The root disk partition to use.
        :param metadata: Metadata for the resource.
        """

        # Behind the scenes, we set the the root partition via the metadata.
        # For a traditional, non-custom, resource it is the metadata that is
        # used to specify the disk image partition root. Therefore, when the
        # root disk partition specified during the construction, we apply it as
        # metadata.
        if disk_root_partition:
            disk_root_partition_dict = {
                "additional_metadata": {"root_partition": disk_root_partition}
            }
            metadata.update(disk_root_partition_dict)

        super().__init__(local_path=local_path, metadata=metadata)


class Resource(AbstractResource):
    """
    An official gem5 resources as hosted within our gem5 resources repository
    (<resources.gem5.org>).

    A user need only specify the name of the resource during construction. The
    resource will be downloaded if needed. A list of available resources can
    be obtained via `downloader.list_resources()`.
    """

    def __init__(
        self,
        resource_name: str,
        resource_directory: Optional[str] = None,
        download_md5_mismatch: bool = True,
    ):
        """
        :param resource_name: The name of the gem5 resource.
        :param resource_directory: The location of the directory in which the
        resource is to be stored. If this parameter is not set, it will set to
        the environment variable `GEM5_RESOURCE_DIR`. If the environment is not
        set it will default to `~/.cache/gem5` if available, otherwise the CWD.
        :param download_md5_mismatch: If the resource is present, but does not
        have the correct md5 value, the resoruce will be deleted and
        re-downloaded if this value is True. Otherwise an exception will be
        thrown. True by default.
        """

        if resource_directory == None:
            resource_directory = os.getenv(
                "GEM5_RESOURCE_DIR", self._get_default_resource_dir()
            )

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

        to_path = os.path.join(resource_directory, resource_name)

        super().__init__(
            local_path=to_path, metadata=get_resources_json_obj(resource_name)
        )
        get_resource(
            resource_name=resource_name,
            to_path=to_path,
            download_md5_mismatch=download_md5_mismatch,
        )

    def _get_default_resource_dir(cls) -> str:
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
                if os.path.isdir(
                    path
                ):  # Check to see the path is a directory.
                    return path  # If so, the path is valid and can be used.
            else:  # If the path does not exist, try to create it.
                try:
                    os.makedirs(path, exist_ok=False)
                    return path
                except OSError:
                    continue  # If the path cannot be created, then try another.

        raise Exception("Cannot find a valid location to download resources")
