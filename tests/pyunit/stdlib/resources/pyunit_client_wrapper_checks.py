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

import unittest
from gem5.isas import ISA
from gem5.resources.client import get_resource_json_obj
import gem5.resources.client
from gem5.resources.client_api.client_wrapper import ClientWrapper
from typing import Dict
from unittest.mock import patch
from unittest import mock
import json
from urllib.error import HTTPError
import io
import contextlib
from pathlib import Path

mock_json_path = Path(__file__).parent / "refs/resources.json"
mock_config_json = {
    "sources": {
        "baba": {
            "url": mock_json_path,
            "isMongo": False,
        }
    },
}

mock_config_mongo = {
    "sources": {
        "gem5-resources": {
            "dataSource": "gem5-vision",
            "database": "gem5-vision",
            "collection": "versions_test",
            "url": "https://data.mongodb-api.com/app/data-ejhjf/endpoint/data/v1",
            "authUrl": "https://realm.mongodb.com/api/client/v2.0/app/data-ejhjf/auth/providers/api-key/login",
            "apiKey": "OIi5bAP7xxIGK782t8ZoiD2BkBGEzMdX3upChf9zdCxHSnMoiTnjI22Yw5kOSgy9",
            "isMongo": True,
        }
    },
}

mock_config_combined = {
    "sources": {
        "gem5-resources": {
            "dataSource": "gem5-vision",
            "database": "gem5-vision",
            "collection": "versions_test",
            "url": "https://data.mongodb-api.com/app/data-ejhjf/endpoint/data/v1",
            "authUrl": "https://realm.mongodb.com/api/client/v2.0/app/data-ejhjf/auth/providers/api-key/login",
            "apiKey": "OIi5bAP7xxIGK782t8ZoiD2BkBGEzMdX3upChf9zdCxHSnMoiTnjI22Yw5kOSgy9",
            "isMongo": True,
        },
        "baba": {
            "url": mock_json_path,
            "isMongo": False,
        },
    },
}

mock_json = {}

with open(Path(__file__).parent / "refs/mongo-mock.json", "r") as f:
    mock_json = json.load(f)

duplicate_mock_json = {}

with open(Path(__file__).parent / "refs/mongo-dup-mock.json", "r") as f:
    duplicate_mock_json = json.load(f)


def mocked_requests_post(*args):
    # mokcing urllib.request.urlopen
    class MockResponse:
        def __init__(self, json_data, status_code):
            self.json_data = json_data
            self.status = status_code

        def read(self):
            return json.dumps(self.json_data).encode("utf-8")

    data = json.loads(args[0].data)
    if "/api-key/login" in args[0].full_url:
        return MockResponse({"access_token": "test-token"}, 200)
    if "/endpoint/data/v1/action/find" in args[0].full_url:
        if data:
            if data["filter"]["id"] == "x86-ubuntu-18.04-img":
                return MockResponse(
                    {
                        "documents": mock_json,
                    },
                    200,
                )
            if data["filter"]["id"] == "test-duplicate":
                return MockResponse(
                    {
                        "documents": duplicate_mock_json,
                    },
                    200,
                )
            if data["filter"]["id"] == "test-too-many":
                error_file = io.BytesIO()
                error_file.status = 429
                raise HTTPError(
                    args[0].full_url, 429, "Too Many Requests", {}, error_file
                )
        return MockResponse(
            {
                "documents": [],
            },
            200,
        )
    error_file = io.BytesIO()
    error_file.status = 404
    raise HTTPError(args[0].full_url, 404, "Not Found", {}, error_file)


class ClientWrapperTestSuite(unittest.TestCase):
    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_json),
    )
    def test_get_resource_json_obj(self):
        # Test that the resource object is correctly returned
        resource = "this-is-a-test-resource"
        resource = get_resource_json_obj(resource)
        self.assertEqual(resource["id"], "this-is-a-test-resource")
        self.assertEqual(resource["resource_version"], "2.0.0")
        self.assertEqual(resource["category"], "binary")
        self.assertEqual(
            resource["description"], "This is a test resource but double newer"
        )
        self.assertEqual(
            resource["source_url"],
            "https://github.com/gem5/gem5-resources/tree/develop/src/asmtest",
        )
        self.assertEqual(resource["architecture"], "X86")

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_json),
    )
    def test_get_resource_json_obj_invalid_client(self):
        # Test that an exception is raised when an invalid client is passed
        resource_id = "test-id"
        client = "invalid"
        with self.assertRaises(Exception) as context:
            get_resource_json_obj(resource_id, clients=[client])
        self.assertTrue(
            f"Client: {client} does not exist" in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_json),
    )
    def test_get_resource_json_obj_with_version(self):
        # Test that the resource object is correctly returned
        resource_id = "this-is-a-test-resource"
        resource_version = "1.0.0"
        resource = get_resource_json_obj(
            resource_id, resource_version=resource_version
        )
        self.assertEqual(resource["id"], "this-is-a-test-resource")
        self.assertEqual(resource["resource_version"], "1.0.0")
        self.assertEqual(resource["category"], "binary")
        self.assertEqual(resource["description"], "This is a test resource")
        self.assertEqual(
            resource["source_url"],
            "https://github.com/gem5/gem5-resources/tree/develop/src/asmtest",
        )
        self.assertEqual(resource["architecture"], "X86")

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_mongo),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_obj_1(self, mock_get):
        resource = "x86-ubuntu-18.04-img"
        resource = get_resource_json_obj(resource)
        self.assertEqual(resource["id"], "x86-ubuntu-18.04-img")
        self.assertEqual(resource["resource_version"], "1.1.0")
        self.assertEqual(resource["category"], "disk-image")
        self.assertEqual(
            resource["description"],
            "A disk image containing Ubuntu 18.04 for x86. This image will run an `m5 readfile` instruction after booting. If no script file is specified an `m5 exit` instruction will be executed.",
        )
        self.assertEqual(
            resource["source_url"],
            "https://github.com/gem5/gem5-resources/tree/develop/src/x86-ubuntu",
        )
        self.assertEqual(resource["architecture"], "X86")

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_mongo),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_obj_with_version_mongodb(self, mock_get):
        # Test that the resource object is correctly returned
        resource_id = "x86-ubuntu-18.04-img"
        resource_version = "1.0.0"
        resource = get_resource_json_obj(
            resource_id,
            resource_version=resource_version,
            clients=["gem5-resources"],
        )
        self.assertEqual(resource["id"], "x86-ubuntu-18.04-img")
        self.assertEqual(resource["resource_version"], "1.0.0")
        self.assertEqual(resource["category"], "disk-image")
        self.assertEqual(resource["description"], "This is a test resource")
        self.assertEqual(
            resource["source_url"],
            "https://github.com/gem5/gem5-resources/tree/develop/src/x86-ubuntu",
        )
        self.assertEqual(resource["architecture"], "X86")

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_mongo),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_obj_with_id_invalid_mongodb(self, mock_get):
        resource_id = "invalid-id"
        with self.assertRaises(Exception) as context:
            get_resource_json_obj(resource_id, clients=["gem5-resources"])
        self.assertTrue(
            "Resource with ID 'invalid-id' not found."
            in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_mongo),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_obj_with_version_invalid_mongodb(
        self, mock_get
    ):
        resource_id = "x86-ubuntu-18.04-img"
        resource_version = "2.5.0"
        with self.assertRaises(Exception) as context:
            get_resource_json_obj(
                resource_id,
                resource_version=resource_version,
                clients=["gem5-resources"],
            )
        self.assertTrue(
            f"Resource x86-ubuntu-18.04-img with version '2.5.0'"
            " not found.\nResource versions can be found at: "
            f"https://resources.gem5.org/resources/x86-ubuntu-18.04-img/versions"
            in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_json),
    )
    def test_get_resource_json_obj_with_version_invalid_json(self):
        resource_id = "this-is-a-test-resource"
        resource_version = "2.5.0"
        with self.assertRaises(Exception) as context:
            get_resource_json_obj(
                resource_id,
                resource_version=resource_version,
            )
        self.assertTrue(
            f"Resource this-is-a-test-resource with version '2.5.0'"
            " not found.\nResource versions can be found at: "
            f"https://resources.gem5.org/resources/this-is-a-test-resource/versions"
            in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_combined),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_obj_combine(self, mock_get):
        resource_id_mongo = "x86-ubuntu-18.04-img"
        resource_version_mongo = "1.0.0"
        resource_id_json = "this-is-a-test-resource"
        resource_version_json = "1.0.0"
        resource_mongo = get_resource_json_obj(
            resource_id_mongo,
            resource_version=resource_version_mongo,
            clients=["gem5-resources"],
        )
        resource_json = get_resource_json_obj(
            resource_id_json,
            resource_version=resource_version_json,
            clients=["baba"],
        )
        self.assertEqual(resource_mongo["id"], "x86-ubuntu-18.04-img")
        self.assertEqual(resource_mongo["resource_version"], "1.0.0")
        self.assertEqual(resource_mongo["category"], "disk-image")
        self.assertEqual(
            resource_mongo["description"], "This is a test resource"
        )
        self.assertEqual(
            resource_mongo["source_url"],
            "https://github.com/gem5/gem5-resources/tree/develop/src/x86-ubuntu",
        )
        self.assertEqual(resource_mongo["architecture"], "X86")

        self.assertEqual(resource_json["id"], "this-is-a-test-resource")
        self.assertEqual(resource_json["resource_version"], "1.0.0")
        self.assertEqual(resource_json["category"], "binary")
        self.assertEqual(
            resource_json["description"], "This is a test resource"
        )
        self.assertEqual(
            resource_json["source_url"],
            "https://github.com/gem5/gem5-resources/tree/develop/src/asmtest",
        )
        self.assertEqual(resource_json["architecture"], "X86")

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_combined),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_obj_multi_database_second_only(self, mock_get):
        resource_id = "simpoint-resource"
        resource = get_resource_json_obj(
            resource_id,
        )
        self.assertEqual(resource["id"], resource_id)
        self.assertEqual(resource["resource_version"], "0.2.0")
        self.assertEqual(resource["category"], "file")
        self.assertEqual(
            resource["description"],
            (
                "Simpoints for running the 'x86-print-this' resource with"
                ' the parameters `"print this" 15000`. This is encapsulated'
                " in the 'x86-print-this-15000-with-simpoints' workload."
            ),
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_combined),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_json_same_resource_different_versions(
        self, mock_get
    ):
        resource_id = "x86-ubuntu-18.04-img"
        resource_json = get_resource_json_obj(
            resource_id,
        )

        self.assertEqual(resource_json["id"], "x86-ubuntu-18.04-img")
        self.assertEqual(resource_json["resource_version"], "2.0.0")
        self.assertEqual(resource_json["category"], "disk-image")

        resource_json = get_resource_json_obj(
            resource_id,
            resource_version="1.0.0",
        )

        self.assertEqual(resource_json["id"], "x86-ubuntu-18.04-img")
        self.assertEqual(resource_json["resource_version"], "1.0.0")
        self.assertEqual(resource_json["category"], "disk-image")

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_combined),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_get_resource_same_resource_same_version(self, mock_get):
        resource_id = "test-duplicate"
        with self.assertRaises(Exception) as context:
            get_resource_json_obj(
                resource_id,
            )
        self.assertTrue(
            f"Resource {resource_id} has multiple resources with"
            f" the same version: 0.2.0" in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(
            {
                "sources": {
                    "gem5-resources": {
                        "dataSource": "gem5-vision",
                        "database": "gem5-vision",
                        "collection": "versions_test",
                        "url": "https://data.mongodb-api.com/app/data-ejhjf/endpoint/data/v1",
                        "authUrl": "https://realm.mongodb.com/api/client/v2.0/app/data-ejhjf/auth/providers/api-key/logi",
                        "apiKey": "OIi5bAP7xxIGK782t8ZoiD2BkBGEzMdX3upChf9zdCxHSnMoiTnjI22Yw5kOSgy9",
                        "isMongo": True,
                    }
                },
            }
        ),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_invalid_auth_url(self, mock_get):
        resource_id = "test-resource"
        f = io.StringIO()
        with self.assertRaises(Exception) as context:
            with contextlib.redirect_stderr(f):
                get_resource_json_obj(
                    resource_id,
                )
        self.assertTrue(
            "Error getting resources from client gem5-resources:"
            " Panic: Not found" in str(f.getvalue())
        )
        self.assertTrue(
            "Resource with ID 'test-resource' not found."
            in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(
            {
                "sources": {
                    "gem5-resources": {
                        "dataSource": "gem5-vision",
                        "database": "gem5-vision",
                        "collection": "versions_test",
                        "url": "https://data.mongodb-api.com/app/data-ejhjf/endpoint/data/v",
                        "authUrl": "https://realm.mongodb.com/api/client/v2.0/app/data-ejhjf/auth/providers/api-key/login",
                        "apiKey": "OIi5bAP7xxIGK782t8ZoiD2BkBGEzMdX3upChf9zdCxHSnMoiTnjI22Yw5kOSgy9",
                        "isMongo": True,
                    }
                },
            }
        ),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_invalid_url(self, mock_get):
        resource_id = "test-resource"
        f = io.StringIO()
        with self.assertRaises(Exception) as context:
            with contextlib.redirect_stderr(f):
                get_resource_json_obj(
                    resource_id,
                )
        self.assertTrue(
            "Error getting resources from client gem5-resources:"
            " Panic: Not found" in str(f.getvalue())
        )
        self.assertTrue(
            "Resource with ID 'test-resource' not found."
            in str(context.exception)
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(
            {
                "sources": {
                    "gem5-resources": {
                        "dataSource": "gem5-vision",
                        "database": "gem5-vision",
                        "collection": "versions_test",
                        "url": "https://data.mongodb-api.com/app/data-ejhjf/endpoint/data/v1",
                        "authUrl": "https://realm.mongodb.com/api/client/v2.0/app/data-ejhjf/auth/providers/api-key/login",
                        "apiKey": "OIi5bAP7xxIGK782t8ZoiD2BkBGEzMdX3upChf9zdCxHSnMoiTnjI22Yw5kOSgy9",
                        "isMongo": True,
                    }
                },
            }
        ),
    )
    @patch("urllib.request.urlopen", side_effect=mocked_requests_post)
    def test_invalid_url(self, mock_get):
        resource_id = "test-too-many"
        f = io.StringIO()
        with self.assertRaises(Exception) as context:
            with contextlib.redirect_stderr(f):
                get_resource_json_obj(
                    resource_id,
                )
        self.assertTrue(
            "Error getting resources from client gem5-resources:"
            " Panic: Too many requests" in str(f.getvalue())
        )
        self.assertTrue(
            "Resource with ID 'test-too-many' not found."
            in str(context.exception)
        )
