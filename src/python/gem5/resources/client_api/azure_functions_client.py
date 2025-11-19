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
import time
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Tuple,
    Type,
    Union,
)
from urllib import (
    error,
    parse,
    request,
)

from m5.util import warn

from ...utils.socks_ssl_context import get_proxy_context
from .abstract_client import AbstractClient
from .client_query import ClientQuery


class AzureFunctionsAPIClientHttpJsonRequestError(Exception):
    def __init__(
        self,
        client: "AzureFunctionsAPIClient",
        data: Dict[str, Any],
        purpose_of_request: Optional[str],
        response: Optional[str] = None,
    ):
        """An exception raised when an HTTP request to the Azure Functions API fails.
        :param client: The AzureFunctionsAPI instance that raised the exception.
        :param purpose_of_request: A string describing the purpose of the
        request.
        """
        error_str = (
            f"Http Request to Azure Functions API failed.\n"
            f"Azure Functions API URL: {client.url}\n"
            f"Data sent:\n\n{json.dumps(data,indent=4)}\n"
            f"Response: {str(response)}\n"
        )

        if purpose_of_request:
            error_str += f"Purpose of Request: {purpose_of_request}\n"
        error_str += "\n"
        super().__init__(error_str)


class AzureFunctionsAPIClient(AbstractClient):
    def __init__(self, config: Dict[str, str]):
        """
        Initializes a connection to the gem5 resources database via azure functions API.

        :param url: The base url for the azure functions API.
        """
        self.url = config["url"]

    def _functions_http_json_req(
        self,
        url: str,
        data_json: List[Dict[str, str]],
        purpose_of_request: Optional[str],
        max_failed_attempts: int = 3,
        reattempt_pause_base: int = 2,
    ) -> Dict[str, Any]:
        """Sends a JSON object over HTTP to a given Azure functions API and
        returns the response. This function will attempt to reconnect to the
        server if the connection fails a set number of times before raising an
        exception.

        :param url: The URL to open the connection.
        :param data_json: The JSON object to send.
        :param headers: The headers to send with the request.
        :param purpose_of_request: A string describing the purpose of the
        request. This is optional. It's used to give context to the user if an
        exception is raised.
        :param max_failed_attempts: The maximum number of times to an attempt
        at making a request should be done before throwing an exception.
        :param reattempt_pause_base: The base of the exponential backoff -- the
        time between each attempt.

        **Warning**: This function assumes a JSON response.
        """

        for resource in data_json:
            params = parse.urlencode(resource)
            url += ("&" if parse.urlparse(url).query else "?") + params

        req = request.Request(url)

        for attempt in itertools.count(start=1):
            try:
                response = request.urlopen(req, context=get_proxy_context())
                break
            except Exception as e:
                error_response = None
                if isinstance(e, error.HTTPError):
                    try:
                        error_content = e.read().decode("utf-8")
                        error_json = json.loads(error_content)
                        if "error" in error_json:
                            error_response = error_json["error"]
                        else:
                            error_response = error_json
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        # If the content isn't valid JSON or can't be decoded as UTF-8
                        error_response = f"Non-JSON error response: {str(e)}"
                if attempt >= max_failed_attempts:
                    raise AzureFunctionsAPIClientHttpJsonRequestError(
                        client=self,
                        data=data_json,
                        purpose_of_request=purpose_of_request,
                        response=error_response,
                    )
                pause = reattempt_pause_base**attempt
                warn(
                    f"Attempt {attempt} of Azure functions HTTP Request failed.\n"
                    f"Purpose of Request: {purpose_of_request}.\n"
                    f"Failed with Exception: {e}\n"
                    f"Response: {str(error_response)}\n\n"
                    f"Retrying after {pause} seconds...\n\n"
                )
                time.sleep(pause)

        return json.loads(response.read().decode("utf-8"))

    def get_resources(
        self,
        client_queries: List[ClientQuery],
    ) -> Dict[str, Any]:
        url = self.url
        url += "/find-resources-in-batch"

        search_conditions = []
        for resource in client_queries:
            condition = {
                "id": resource.get_resource_id(),
            }

            # If the resource has a resource_version, add it to the search
            # conditions.
            if resource.get_resource_version():
                condition["resource_version"] = resource.get_resource_version()
            else:
                condition["resource_version"] = "None"

            search_conditions.append(condition)

        resources = self._functions_http_json_req(
            url,
            data_json=search_conditions,
            purpose_of_request="Get Resources",
        )
        resources_by_id = {}
        for resource in resources:
            if resource["id"] in resources_by_id.keys():
                resources_by_id[resource["id"]].append(resource)
            else:
                resources_by_id[resource["id"]] = [resource]

        # Sort the resources by version and return the latest version.
        for id, resource_list in resources_by_id.items():
            resources_by_id[id] = self.sort_resources(resource_list)[0]

        # Check if the resource is compatible with the gem5version
        for resource in client_queries:
            if resource.get_resource_id() not in resources_by_id:
                continue
            if not resource.get_gem5_version().startswith("DEVELOP"):
                if not any(
                    resource.get_gem5_version().startswith(gem5_version)
                    for gem5_version in resources_by_id[
                        resource.get_resource_id()
                    ]["gem5_versions"]
                ):
                    warn(
                        f"Resource {resource.get_resource_id()} is not compatible with gem5 version {resource.get_gem5_version()}."
                    )
        return resources_by_id
