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
import json
import os
import sys
from pathlib import Path
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Tuple,
)

from m5.util import (
    inform,
    warn,
)

from _m5 import core

from gem5.gem5_default_config import config

from .client_api.atlasclient import AtlasClient
from .client_api.jsonclient import JSONClient


def getFileContent(file_path: Path) -> Dict:
    """
    Get the content of the file at the given path.

    :param file_path: The path of the file.

    :return: The content of the file.
    """
    if file_path.exists():
        with open(file_path) as file:
            return json.load(file)
    else:
        raise Exception(f"File not found at {file_path}")


clientwrapper = None


def _get_clientwrapper():
    global clientwrapper
    if clientwrapper is None:
        if (
            "GEM5_RESOURCE_JSON" in os.environ
            and "GEM5_RESOURCE_JSON_APPEND" in os.environ
        ):
            raise Exception(
                "Both GEM5_RESOURCE_JSON and GEM5_RESOURCE_JSON_APPEND are set. Please set only one of them."
            )
        gem5_config = {}
        # If the GEM5_RESOURCE_JSON is set, use it as the only source
        if "GEM5_RESOURCE_JSON" in os.environ:
            json_source = {
                "url": os.environ["GEM5_RESOURCE_JSON"],
                "isMongo": False,
            }
            gem5_config["sources"] = {"GEM5_RESOURCE_JSON": json_source}
            if "GEM5_CONFIG" in os.environ:
                warn(
                    f"Both GEM5_CONFIG and GEM5_RESOURCE_JSON are set.\n"
                    f"GEM5_CONFIG will be ignored in favor of the GEM5_RESOURCE_JSON environment variable."
                )
            elif (Path().cwd().resolve() / "gem5-config.json").exists():
                warn(
                    f"Both gem5-config.json and GEM5_RESOURCE_JSON are set.\n"
                    f"gem5-config.json will be ignored in favor of the GEM5_RESOURCE_JSON environment variable."
                )
            else:
                warn(
                    f"GEM5_RESOURCE_JSON is set.\n"
                    f"gem5-default-config will be ignored in favor of the GEM5_RESOURCE_JSON environment variable."
                )
        # First check if the config file path is provided in the environment variable
        elif "GEM5_CONFIG" in os.environ:
            config_file_path = Path(os.environ["GEM5_CONFIG"])
            gem5_config = getFileContent(config_file_path)
            inform("Using config file specified by $GEM5_CONFIG")
            inform(f"Using config file at {os.environ['GEM5_CONFIG']}")
        # If not, check if the config file is present in the current directory
        elif (Path().cwd().resolve() / "gem5-config.json").exists():
            config_file_path = Path().resolve() / "gem5-config.json"
            gem5_config = getFileContent(config_file_path)
            inform(f"Using config file at {config_file_path}")
        # If not, use the default config in the build directory
        else:
            gem5_config = config
            inform("Using default config")

        # If the GEM5_RESOURCE_JSON_APPEND is set, append the resources to the gem5_config
        if "GEM5_RESOURCE_JSON_APPEND" in os.environ:
            json_source = {
                "url": os.environ["GEM5_RESOURCE_JSON_APPEND"],
                "isMongo": False,
            }
            gem5_config["sources"].update(
                {"GEM5_RESOURCE_JSON_APPEND": json_source}
            )
            inform(
                f"Appending resources from {os.environ['GEM5_RESOURCE_JSON_APPEND']}"
            )

        clientwrapper = _create_clients(gem5_config)
    return clientwrapper


def list_resources(
    clients: Optional[List[str]] = None,
    gem5_version: Optional[str] = core.gem5Version,
) -> Dict[str, List[str]]:
    """
    List all the resources available

    :param clients: The list of clients to query
    :param gem5_version: The gem5 version of the resource to get. By default,
                         it is the gem5 version of the current build. If set to
                         ``None``, it will return all gem5 versions of the resource.
    :return: A Python Dict where the key is the resource id and the value is
             a list of all the supported resource versions.
    """
    _get_clientwrapper()
    return _list_all_resources(clients, gem5_version)


def get_resource_json_obj(
    resource_id,
    resource_version: Optional[str] = None,
    clients: Optional[List[str]] = None,
    gem5_version: Optional[str] = core.gem5Version,
) -> Dict:
    """
    Get the resource json object from the clients wrapper.

    :param resource_id: The resource id.
    :param resource_version: The resource version.
    :param clients: The list of clients to query.
    :param gem5_version: The gem5 versions to filter the resources based on
                         compatibility. By default, it is the gem5 version of the
                         current build. If ``None``, filtering based on compatibility
                         is not performed.
    """
    _get_clientwrapper()
    if resource_version:
        resource_info = [
            {"id": resource_id, "resource_version": resource_version}
        ]
    else:
        resource_info = [{"id": resource_id}]

    # We will return a list when we refactor ontain_resources to handle multiple
    # resources
    return _get_resource_json_obj_from_client(
        resource_info, clients, gem5_version
    )[0]


def get_multiple_resource_json_obj(
    resource_info: List[Dict[str, str]],
    clients: Optional[List[str]] = None,
    gem5_version: Optional[str] = core.gem5Version,
) -> List[Dict]:
    """
    Get the resource json object from the clients wrapper.

    :param resource_info: The resource info.
    :param clients: The list of clients to query.
    :param gem5_version: The gem5 versions to filter the resources based on
                         compatibility. By default, it is the gem5 version of the
                         current build. If ``None``, filtering based on compatibility
                         is not performed.
    """
    _get_clientwrapper()
    return _get_resource_json_obj_from_client(
        resource_info, clients, gem5_version
    )


def _create_clients(
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


def _list_all_resources(
    clients: Optional[List[str]] = None,
    gem5_version: Optional[str] = core.gem5Version,
) -> Dict[str, List[str]]:
    global clientwrapper
    clients_to_search = (
        list(clientwrapper.keys()) if clients is None else clients
    )
    # There's some duplications of functionality here (similar code in
    # `get_all_resources_by_id`. This code could be refactored to avoid
    # this).
    resources = []
    for client in clients_to_search:
        if client not in clientwrapper:
            raise Exception(f"Client: {client} does not exist")
        try:
            resources.extend(
                clientwrapper[client].get_resources(gem5_version=gem5_version)
            )
        except Exception as e:
            warn(f"Error getting resources from client {client}: {str(e)}")

    to_return = {}
    for resource in resources:
        if resource["id"] not in to_return:
            to_return[resource["id"]] = []
        to_return[resource["id"]].append(resource["resource_version"])
    return to_return


def _get_resource_json_obj_from_client(
    resource_info: List[Dict[str, str]],
    clients: Optional[List[str]] = None,
    gem5_version: Optional[str] = core.gem5Version,
) -> Dict:
    """
    This function returns the resource object from the client with the
    given id and version.

    :param resource_info: List of dictionaries containing information about the
                            resources to fetch from datasources.
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
    resources = _get_all_resources_by_id(resource_info, clients)
    # if no resource with the given id is found, return None
    if len(resources) == 0:
        raise Exception(f"Resource with ID '{resource_info}' not found.")

    resource_to_return = []

    compatible_resources = _get_resources_compatible_with_gem5_version(
        resources, resource_info, gem5_version=gem5_version
    )

    for id, resource in compatible_resources.items():
        if len(resource) == 0:
            resource_to_return.append(_sort_resources(resources[id])[0])
        else:
            resource_to_return.append(_sort_resources(resource)[0])

    if gem5_version:
        for resource in resource_to_return:
            _check_resource_version_compatibility(
                resource, gem5_version=gem5_version
            )

    return resource_to_return


def _get_all_resources_by_id(
    resource_info: List[Dict[str, str]],
    clients: Optional[List[str]] = None,
) -> Dict[str, Any]:
    """
    This function returns all the resources with the given id from all the
    sources.

    :param resource_info: List of dictionaries containing information about the
                            resources to fetch from datasources.
    :param clients: A list of clients to search through. If ``None``, all
                    clients are searched.
    :return: A list of resources as Python dictionaries.
    """
    global clientwrapper

    # creating a dictionary with the resource id as the key and an empty
    # list as the value, the list will be populated with different versions
    # of the resource
    resources = {}
    for resource in resource_info:
        resources[resource["id"]] = []

    if not clients:
        clients = list(clientwrapper.keys())
    for client in clients:
        if client not in clientwrapper:
            raise Exception(f"Client: {client} does not exist")
        try:
            filtered_resources = clientwrapper[client].get_resources_by_id(
                resource_info
            )
            for k in resources.keys():
                if k in filtered_resources.keys():
                    resources[k].extend(filtered_resources[k])

        except Exception as e:
            print(
                f"Exception thrown while getting resources '{resource_info}' "
                f"from client '{client}'\n",
                file=sys.stderr,
            )
            raise e
    # check if no 2 resources have the same id and version
    for resource_id, different_version_of_resource in resources.items():
        for res1, res2 in itertools.combinations(
            different_version_of_resource, 2
        ):
            if res1["resource_version"] == res2["resource_version"]:
                raise Exception(
                    f"Resource {resource_id} has multiple resources with "
                    f"the same version: {res1['resource_version']}"
                )

    return resources


def _get_resources_compatible_with_gem5_version(
    resources: Dict[str, Any],
    resource_info: List[Dict[str, str]],
    gem5_version: str = core.gem5Version,
) -> Dict[str, Any]:
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

    :param resources: A dictionary of resources to filter.
    :param resource_info: List of dictionaries containing information about the
                            resources to fetch from datasources.

    :return: A list of compatible resources as Python dictionaries.

    .. note::

        This is a big duplication of code. This functionality already
        exists in the `AbstractClient` class. This code should be refactored
        to avoid this duplication.
    """
    compatible_resources = {}

    for resource_id_ver in resource_info:
        # If resource version is specified then we just add the specific
        # resource version to the compatible resources as we don't need to
        # check for compatibility which will be done later
        if "resource_version" not in resource_id_ver.keys():
            compatible_resource_versions = []

            for resource in resources[resource_id_ver["id"]]:
                for version in resource["gem5_versions"]:
                    if gem5_version.startswith(version):
                        compatible_resource_versions.append(resource)

            compatible_resources[
                resource_id_ver["id"]
            ] = compatible_resource_versions
        else:
            compatible_resources[resource_id_ver["id"]] = resources[
                resource_id_ver["id"]
            ]

    return compatible_resources


def _sort_resources(resources: List) -> List:
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
    resource: dict, gem5_version: Optional[str] = core.gem5Version
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
        and not _get_resources_compatible_with_gem5_version(
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
