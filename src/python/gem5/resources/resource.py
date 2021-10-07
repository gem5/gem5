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

from .downloader import get_resource

from typing import Optional

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

    def __init__(self, local_path: str):
        self._local_path = local_path

    def get_local_path(self) -> str:
        return self._local_path


class CustomResource(AbstractResource):
    """
    A custom gem5 resource. This can be used to encapsulate a resource provided
    by a gem5 user as opposed to one available within the gem5 resources
    repository.
    """

    def __init__(self, local_path: str):
        """
        :param local_path: The path of the resource on the host system.
        """
        super().__init__(local_path=local_path)


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
        override: Optional[bool] = False,
    ):
        """
        :param resource_name: The name of the gem5 resource.
        :param resource_directory: The location of the directory in which the
        resource is to be stored. If this parameter is not set, it will set to
        the environment variable `GEM5_RESOURCE_DIR`. If the environment is not
        set it will default to `~/.cache/gem5`.
        :param override: If the resource is present, but does not have the
        correct md5 value, the resoruce will be deleted and re-downloaded if
        this value is True. Otherwise an exception will be thrown. False by
        default.
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
            os.makedirs(resource_directory)

        to_path = os.path.join(resource_directory, resource_name)

        super(Resource, self).__init__(local_path=to_path)
        get_resource(
            resource_name=resource_name, to_path=to_path, override=override
        )

    def _get_default_resource_dir(cls) -> str:
        """
        Obtain the default gem5 resources directory on the host system.

        :returns: The default gem5 resources directory.
        """
        return os.path.join(Path.home(), ".cache", "gem5")
