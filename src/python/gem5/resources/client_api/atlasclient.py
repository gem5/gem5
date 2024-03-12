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
import ssl
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
    parse,
    request,
)

from m5.util import warn

from ...utils.socks_ssl_context import get_proxy_context
from .abstract_client import AbstractClient


class AtlasClientHttpJsonRequestError(Exception):
    def __init__(
        self,
        client: "AtlasClient",
        data: Dict[str, Any],
        purpose_of_request: Optional[str],
    ):
        """An exception raised when an HTTP request to Atlas MongoDB fails.
        :param client: The AtlasClient instance that raised the exception.
        :param purpose_of_request: A string describing the purpose of the
        request.
        """
        error_str = (
            f"Http Request to Atlas MongoDB failed.\n"
            f"Atlas URL: {client.url}\n"
            f"Auth URL: {client.authUrl}\n"
            f"Database: {client.database}\n"
            f"Collection: {client.collection}\n\n"
            f"Data sent:\n\n{json.dumps(data,indent=4)}\n\n"
        )

        if purpose_of_request:
            error_str += f"Purpose of Request: {purpose_of_request}\n\n"
        super().__init__(error_str)


class AtlasClient(AbstractClient):
    def __init__(self, config: Dict[str, str]):
        """
        Initializes a connection to a MongoDB Atlas database.

        :param uri: The URI for connecting to the MongoDB server.
        :param db: The name of the database to connect to.
        :param collection: The name of the collection within the database.
        """
        self.apiKey = config["apiKey"]
        self.url = config["url"]
        self.collection = config["collection"]
        self.database = config["database"]
        self.dataSource = config["dataSource"]
        self.authUrl = config["authUrl"]

    def get_token(self):
        return self._atlas_http_json_req(
            self.authUrl,
            data_json={"key": self.apiKey},
            headers={"Content-Type": "application/json"},
            purpose_of_request="Get Access Token with API key",
        )["access_token"]

    def _atlas_http_json_req(
        self,
        url: str,
        data_json: Dict[str, Any],
        headers: Dict[str, str],
        purpose_of_request: Optional[str],
        max_failed_attempts: int = 4,
        reattempt_pause_base: int = 2,
    ) -> Dict[str, Any]:
        """Sends a JSON object over HTTP to a given Atlas MongoDB server and
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
        proxy_context = get_proxy_context()
        data = json.dumps(data_json).encode("utf-8")
        req = request.Request(
            url,
            data=data,
            headers=headers,
        )

        for attempt in itertools.count(start=1):
            try:
                response = request.urlopen(req, context=proxy_context)
                break
            except Exception as e:
                if attempt >= max_failed_attempts:
                    raise AtlasClientHttpJsonRequestError(
                        client=self,
                        data=data_json,
                        purpose_of_request=purpose_of_request,
                    )
                pause = reattempt_pause_base**attempt
                warn(
                    f"Attempt {attempt} of Atlas HTTP Request failed.\n"
                    f"Purpose of Request: {purpose_of_request}.\n\n"
                    f"Failed with Exception:\n{e}\n\n"
                    f"Retrying after {pause} seconds..."
                )
                time.sleep(pause)

        return json.loads(response.read().decode("utf-8"))

    def get_resources(
        self,
        resource_id: Optional[str] = None,
        resource_version: Optional[str] = None,
        gem5_version: Optional[str] = None,
        proxy_context: Optional[ssl.SSLContext] = None,
    ) -> List[Dict[str, Any]]:
        url = f"{self.url}/action/find"
        data = {
            "dataSource": self.dataSource,
            "collection": self.collection,
            "database": self.database,
        }
        filter = {}
        if resource_id:
            filter["id"] = resource_id
            if resource_version is not None:
                filter["resource_version"] = resource_version

        if filter:
            data["filter"] = filter

        headers = {
            "Authorization": f"Bearer {self.get_token()}",
            "Content-Type": "application/json",
        }

        resources = self._atlas_http_json_req(
            url,
            data_json=data,
            headers=headers,
            purpose_of_request="Get Resources",
        )["documents"]

        # I do this as a lazy post-processing step because I can't figure out
        # how to do this via an Atlas query, which may be more efficient.
        return self.filter_incompatible_resources(
            resources_to_filter=resources, gem5_version=gem5_version
        )
