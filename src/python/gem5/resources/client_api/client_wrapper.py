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

import itertools
import sys
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
)

from m5.util import warn

from _m5 import core

from .atlasclient import AtlasClient
from .jsonclient import JSONClient


class ClientWrapper:
    def __init__(self, config):
        self.clients = self.create_clients(config)

    def create_clients(
        self,
        config: Dict,
    ) -> Dict:
        """
        This function creates respective client object for each source in the
        config file according to the type of source.

        :param config: config file containing the source information

        :returns: clients: dictionary of clients for each source
        """
        clients = {}
        for client in config["sources"]:
            client_source = config["sources"][client]
            try:
                if client_source["isMongo"]:
                    clients[client] = AtlasClient(client_source)
                else:
                    clients[client] = JSONClient(client_source["url"])
            except Exception as e:
                warn(f"Error creating client {client}: {str(e)}")
        return clients

    def list_resources(
        self,
        clients: Optional[List[str]] = None,
        gem5_version: Optional[str] = core.gem5Version,
    ) -> Dict[str, List[str]]:
        clients_to_search = (
            list(self.clients.keys()) if clients is None else clients
        )
        # There's some duplications of functionality here (similar code in
        # `get_all_resources_by_id`. This code could be refactored to avoid
        # this).
        resources = []
        for client in clients_to_search:
            if client not in self.clients:
                raise Exception(f"Client: {client} does not exist")
            try:
                resources.extend(
                    self.clients[client].get_resources(
                        gem5_version=gem5_version
                    )
                )
            except Exception as e:
                warn(f"Error getting resources from client {client}: {str(e)}")

        to_return = {}
        for resource in resources:
            if resource["id"] not in to_return:
                to_return[resource["id"]] = []
            to_return[resource["id"]].append(resource["resource_version"])
        return to_return

    def get_all_resources_by_id(
        self,
        resource_id: str,
        clients: Optional[List[str]] = None,
    ) -> List[Dict]:
        """
        This function returns all the resources with the given id from all the
        sources.

        :param resource_id: The id of the resource to search for.
        :param clients: A list of clients to search through. If ``None``, all
                        clients are searched.
        :return: A list of resources as Python dictionaries.
        """
        resources = []
        if not clients:
            clients = list(self.clients.keys())
        for client in clients:
            if client not in self.clients:
                raise Exception(f"Client: {client} does not exist")
            try:
                resources.extend(
                    self.clients[client].get_resources_by_id(resource_id)
                )
            except Exception as e:
                print(
                    f"Exception thrown while getting resource '{resource_id}' "
                    f"from client '{client}'\n",
                    file=sys.stderr,
                )
                raise e
        # check if no 2 resources have the same id and version
        for res1, res2 in itertools.combinations(resources, 2):
            if res1["resource_version"] == res2["resource_version"]:
                raise Exception(
                    f"Resource {resource_id} has multiple resources with "
                    f"the same version: {res1['resource_version']}"
                )
        return resources

    def get_resource_json_obj_from_client(
        self,
        resource_id: str,
        resource_version: Optional[str] = None,
        clients: Optional[List[str]] = None,
        gem5_version: Optional[str] = core.gem5Version,
    ) -> Dict:
        """
        This function returns the resource object from the client with the
        given id and version.

        :param resource_id: The id of the resource to search for.
        :param resource_version: The version of the resource to search for.
        :param clients: A list of clients to search through. If ``None``, all
                        clients are searched.
        :param gem5_version: The gem5 version to check compatibility with. If
                             ``None``, no compatibility check is performed. By
                             default, is the current version of gem5.
        :return: The resource object as a Python dictionary if found.
                 If not found, exception is thrown.
        """
        # getting all the resources with the given id from the dictionary
        resources = self.get_all_resources_by_id(resource_id, clients)
        # if no resource with the given id is found, return None
        if len(resources) == 0:
            raise Exception(f"Resource with ID '{resource_id}' not found.")

        resource_to_return = None

        if resource_version:
            resource_to_return = self._search_version_in_resources(
                resources, resource_id, resource_version
            )

        else:
            compatible_resources = (
                self._get_resources_compatible_with_gem5_version(
                    resources, gem5_version=gem5_version
                )
            )
            if len(compatible_resources) == 0:
                resource_to_return = self._sort_resources(resources)[0]
            else:
                resource_to_return = self._sort_resources(
                    compatible_resources
                )[0]

        if gem5_version:
            self._check_resource_version_compatibility(
                resource_to_return, gem5_version=gem5_version
            )

        return resource_to_return

    def _search_version_in_resources(
        self, resources: List, resource_id: str, resource_version: str
    ) -> Dict:
        """
        Searches for the resource with the given version. If the resource is
        not found, an exception is thrown.

        :param resources: A list of resources to search through.
        :param resource_version: The version of the resource to search for.

        :return: The resource object as a Python dictionary if found.
                 If not found, ``None`` is returned.
        """
        return_resource = next(
            iter(
                [
                    resource
                    for resource in resources
                    if resource["resource_version"] == resource_version
                ]
            ),
            None,
        )
        if not return_resource:
            raise Exception(
                f"Resource {resource_id} with version '{resource_version}'"
                " not found.\nResource versions can be found at: "
                "https://resources.gem5.org/"
                f"resources/{resource_id}/versions"
            )
        return return_resource

    def _get_resources_compatible_with_gem5_version(
        self, resources: List, gem5_version: str = core.gem5Version
    ) -> List:
        """
        Returns a list of compatible resources with the current gem5 version.

        .. note::

            This function assumes if the minor component of a resource's
            gem5_version is not specified, it that the resource is compatible
            all minor versions of the same major version.

            Likewise, if no hot-fix component is specified, it is assumed that
            the resource is compatible with all hot-fix versions of the same
            minor version.

        * '20.1' would be compatible with gem5 '20.1.1.0' and '20.1.2.0'.
        * '21.5.2' would be compatible with gem5 '21.5.2.0' and '21.5.2.0'.
        * '22.3.2.4' would only be compatible with gem5 '22.3.2.4'.

        :param resources: A list of resources to filter.

        :return: A list of compatible resources as Python dictionaries.

        .. note::

            This is a big duplication of code. This functionality already
            exists in the `AbstractClient` class. This code should be refactored
            to avoid this duplication.
        """

        compatible_resources = []
        for resource in resources:
            for version in resource["gem5_versions"]:
                if gem5_version.startswith(version):
                    compatible_resources.append(resource)
        return compatible_resources

    def _sort_resources(self, resources: List) -> List:
        """
        Sorts the resources by ID.

        If the IDs are the same, the resources are sorted by version.

        :param resources: A list of resources to sort.

        :return: A list of sorted resources.
        """

        def sort_tuple(resource: Dict) -> Tuple:
            """This is used for sorting resources by ID and version. First
            the ID is sorted, then the version. In cases where the version
            contains periods, it's assumed this is to separate a
            ``major.minor.hotfix`` style versioning system. In which case, the
            value separated in the most-significant position is sorted before
            those less significant. If the value is a digit it is cast as an
            int, otherwise, it is cast as a string, to lower-case.
            """
            to_return = (resource["id"].lower(),)
            for val in resource["resource_version"].split("."):
                if val.isdigit():
                    to_return += (int(val),)
                else:
                    to_return += (str(val).lower(),)
            return to_return

        return sorted(
            resources,
            key=lambda resource: sort_tuple(resource),
            reverse=True,
        )

    def _check_resource_version_compatibility(
        self, resource: dict, gem5_version: Optional[str] = core.gem5Version
    ) -> bool:
        """
        Checks if the resource is compatible with the gem5 version.

        Prints a warning if the resource is not compatible.

        :param resource: The resource to check.
        :optional param gem5_version: The gem5 version to check
                                      compatibility with.
        :return: ``True`` if the resource is compatible, ``False`` otherwise.
        """
        if not resource:
            return False
        if (
            gem5_version
            and not gem5_version.upper().startswith("DEVELOP")
            and not self._get_resources_compatible_with_gem5_version(
                [resource], gem5_version=gem5_version
            )
        ):
            if not gem5_version.upper().startswith("DEVELOP"):
                warn(
                    f"Resource {resource['id']} with version "
                    f"{resource['resource_version']} is not known to be compatible"
                    f" with gem5 version {gem5_version}. "
                    "This may cause problems with your simulation. "
                    "This resource's compatibility "
                    "with different gem5 versions can be found here: "
                    "https://resources.gem5.org"
                    f"/resources/{resource['id']}/versions"
                )
            return False
        return True
