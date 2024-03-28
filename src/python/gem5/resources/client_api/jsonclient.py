# Copyright (c) 2023 The Regents of the University of California
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

import json
from pathlib import Path
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Tuple,
    Type,
    Union,
)
from urllib import request
from urllib.error import URLError

from m5.util import warn

from .abstract_client import AbstractClient


class JSONClient(AbstractClient):
    def __init__(self, path: str):
        """
        Initializes a JSON client.

        :param path: The path to the Resource, either URL or local.
        """
        self.path = path
        self.resources = []

        if Path(self.path).is_file():
            self.resources = json.load(open(self.path))
        elif not self._url_validator(self.path):
            raise Exception(
                f"Resources location '{self.path}' is not a valid path or URL."
            )
        else:
            req = request.Request(self.path)
            try:
                response = request.urlopen(req)
            except URLError as e:
                raise Exception(
                    f"Unable to open Resources location '{self.path}': {e}"
                )
            self.resources = json.loads(response.read().decode("utf-8"))

    def get_resources_json(self) -> List[Dict[str, Any]]:
        """Returns a JSON representation of the resources."""
        return self.resources

    def get_resources(
        self,
        resource_info: List[Dict[str, str]],
        gem5_version: Optional[str] = None,
    ) -> Dict[str, Any]:
        def filter_resource(resource, resource_info):
            for resource_query in resource_info:
                gem5_version_match = False
                resource_version_match = False

                if (
                    "gem5_version" in resource_query.keys()
                    and not resource_query["gem5_version"].startswith(
                        "DEVELOP"
                    )
                ):
                    gem5_version_match = (
                        resource["gem5_version"]
                        in resource_query["gem5_version"]
                    )

                if "resource_version" in resource_query.keys():
                    resource_version_match = (
                        resource["resource_version"]
                        == resource_query["resource_version"]
                    )

                if gem5_version_match and resource_version_match:
                    return True

            return False

        filtered_resources = filter(
            lambda resource: filter_resource(resource, resource_info),
            self.resources,
        )

        resources_by_id = {}
        for resource in filtered_resources:
            if resource["resource_id"] in resources_by_id.keys():
                resources_by_id[resource["resource_id"]].append(resource)
            else:
                resources_by_id[resource["resource_id"]] = [resource]

        # Sort the resoruces by resoruce version and get the latest version.
        for id, resource_list in resources_by_id.items():
            resources_by_id[id] = self.sort_resources(resource_list)[0]

        return resources_by_id
