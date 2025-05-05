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

import copy
import json
import urllib.parse
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
from .client_query import ClientQuery


class JSONClient(AbstractClient):
    def __init__(self, path: str):
        """
        Initializes a JSON client.

        :param path: The path to the Resource, either URL or local.
        """
        self.path = path
        self.resources = []

        # Try loading as local file if it exists
        if Path(path).is_file():
            try:
                with open(path, encoding="utf-8") as f:
                    self.resources = json.load(f)
                if not isinstance(self.resources, list):
                    raise ValueError(
                        f"Invalid JSON in file '{path}': "
                        "Top-level object must be a list"
                    )
                return
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON in file '{path}': {e}")
            except Exception as e:
                raise FileNotFoundError(f"Error reading file '{path}': {e}")

        # Handle URLs (including file:// URIs)
        if self._url_validator(path):
            try:
                parsed_url = urllib.parse.urlparse(path)
                # Handle file:// URLs
                if parsed_url.scheme == "file":
                    local_path = Path(parsed_url.path)
                    if not local_path.is_file():
                        raise FileNotFoundError(f"File not found: '{path}'")

                    with local_path.open("r", encoding="utf-8") as f:
                        self.resources = json.load(f)
                    if not isinstance(self.resources, list):
                        raise ValueError(
                            f"Invalid JSON in file '{path}': "
                            "Top-level object must be a list"
                        )
                    return

                # Handle HTTP/HTTPS URLs
                req = request.Request(path)
                with request.urlopen(req) as response:
                    self.resources = json.loads(
                        response.read().decode("utf-8")
                    )
                if not isinstance(self.resources, list):
                    raise ValueError(
                        f"Invalid JSON in file '{path}': "
                        "Top-level object must be a list"
                    )
                return

            except URLError as e:
                raise ConnectionError(f"Failed to access URL '{path}': {e}")
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON from URL '{path}': {e}")
            except Exception as e:
                raise ValueError(f"Error processing URL '{path}': {e}")

        raise ValueError(f"'{path}' is not a valid file path or URL")

    def get_resources_json(self) -> List[Dict[str, Any]]:
        """Returns a JSON representation of the resources."""
        return self.resources

    def get_resources(
        self,
        client_queries: List[ClientQuery],
    ) -> Dict[str, Any]:
        def filter_resource(resource, client_queries):
            for resource_query in client_queries:
                gem5_version_match = False
                resource_version_match = False

                if (
                    resource_query.get_gem5_version() is not None
                    and not resource_query.get_gem5_version().startswith(
                        "DEVELOP"
                    )
                ):
                    gem5_version_match = any(
                        resource_query.get_gem5_version().startswith(
                            gem5_version
                        )
                        for gem5_version in resource["gem5_versions"]
                    )
                else:
                    gem5_version_match = True

                if resource_query.get_resource_version() is not None:
                    resource_version_match = (
                        resource["resource_version"]
                        == resource_query.get_resource_version()
                    )
                else:
                    resource_version_match = True

                resource_id_match = (
                    resource_query.get_resource_id() == resource["id"]
                )

                if (
                    gem5_version_match
                    and resource_version_match
                    and resource_id_match
                ):
                    return True

            return False

        filtered_resources = filter(
            lambda resource: filter_resource(resource, client_queries),
            self.resources,
        )

        resources_by_id = {}
        for resource in filtered_resources:
            if resource["id"] in resources_by_id.keys():
                resources_by_id[resource["id"]].append(resource)
            else:
                resources_by_id[resource["id"]] = [resource]

        # Sort the resoruces by resoruce version and get the latest version.
        for id, resource_list in resources_by_id.items():
            resources_by_id[id] = self.sort_resources(resource_list)[0]

        return copy.deepcopy(resources_by_id)
