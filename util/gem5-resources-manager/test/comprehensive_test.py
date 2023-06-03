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
from server import app
import json
from bson import json_util
import copy
import mongomock
from unittest.mock import patch
from api.mongo_client import MongoDBClient


class TestComprehensive(unittest.TestCase):
    @patch.object(
        MongoDBClient,
        "_get_database",
        return_value=mongomock.MongoClient().db.collection,
    )
    def setUp(self, mock_get_database):
        """This method sets up the test environment."""
        self.ctx = app.app_context()
        self.ctx.push()
        self.app = app
        self.test_client = app.test_client()
        self.alias = "test"
        objects = []
        with open("./test/refs/resources.json", "rb") as f:
            objects = json.loads(f.read(), object_hook=json_util.object_hook)
        self.collection = mock_get_database()
        for obj in objects:
            self.collection.insert_one(obj)

        self.test_client.post(
            "/validateMongoDB",
            json={
                "uri": "mongodb://localhost:27017",
                "database": "test",
                "collection": "test",
                "alias": self.alias,
            },
        )

    def tearDown(self):
        """This method tears down the test environment."""
        self.collection.drop()
        self.ctx.pop()

    def test_insert_find_update_find(self):
        test_resource = {
            "category": "diskimage",
            "id": "test-resource",
            "author": ["test-author"],
            "description": "test-description",
            "license": "test-license",
            "source_url": (
                "https://github.com/gem5/gem5-resources/"
                "tree/develop/src/x86-ubuntu"
            ),
            "tags": ["test-tag", "test-tag2"],
            "example_usage": " test-usage",
            "gem5_versions": [
                "22.1",
            ],
            "resource_version": "1.0.0",
        }
        original_resource = test_resource.copy()
        test_id = test_resource["id"]
        test_resource_version = test_resource["resource_version"]
        # insert resource
        response = self.test_client.post(
            "/insert", json={"resource": test_resource, "alias": self.alias}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Inserted"})
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

        # update resource
        test_resource["description"] = "test-description-2"
        test_resource["author"].append("test-author-2")
        response = self.test_client.post(
            "/update",
            json={
                "original_resource": original_resource,
                "resource": test_resource,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Updated"})
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

    def test_find_new_insert(self):
        test_resource = {
            "category": "diskimage",
            "id": "test-resource",
            "author": ["test-author"],
            "description": "test-description",
            "license": "test-license",
            "source_url": (
                "https://github.com/gem5/gem5-resources/"
                "tree/develop/src/x86-ubuntu"
            ),
            "tags": ["test-tag", "test-tag2"],
            "example_usage": " test-usage",
            "gem5_versions": [
                "22.1",
            ],
            "resource_version": "1.0.0",
        }
        test_id = test_resource["id"]
        test_resource_version = test_resource["resource_version"]
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"exists": False})
        # insert resource
        response = self.test_client.post(
            "/insert", json={"resource": test_resource, "alias": self.alias}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Inserted"})
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

    def test_insert_find_new_version_find_older(self):
        test_resource = {
            "category": "diskimage",
            "id": "test-resource",
            "author": ["test-author"],
            "description": "test-description",
            "license": "test-license",
            "source_url": (
                "https://github.com/gem5/gem5-resources/"
                "tree/develop/src/x86-ubuntu"
            ),
            "tags": ["test-tag", "test-tag2"],
            "example_usage": " test-usage",
            "gem5_versions": [
                "22.1",
            ],
            "resource_version": "1.0.0",
        }
        test_id = test_resource["id"]
        test_resource_version = test_resource["resource_version"]
        # insert resource
        response = self.test_client.post(
            "/insert", json={"resource": test_resource, "alias": self.alias}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Inserted"})
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

        # add new version
        test_resource_new_version = copy.deepcopy(test_resource)
        test_resource_new_version["description"] = "test-description-2"
        test_resource_new_version["author"].append("test-author-2")
        test_resource_new_version["resource_version"] = "1.0.1"

        response = self.test_client.post(
            "/insert",
            json={"resource": test_resource_new_version, "alias": self.alias},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Inserted"})

        # get resource versions
        response = self.test_client.post(
            "/versions", json={"id": test_id, "alias": self.alias}
        )
        return_json = json.loads(response.data)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            return_json,
            [{"resource_version": "1.0.1"}, {"resource_version": "1.0.0"}],
        )

        resource_version = return_json[1]["resource_version"]
        # find older version
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

    def test_find_add_new_version_delete_older(self):
        test_resource = {
            "category": "binary",
            "id": "binary-example",
            "description": "binary-example documentation.",
            "architecture": "ARM",
            "is_zipped": False,
            "md5sum": "71b2cb004fe2cda4556f0b1a38638af6",
            "url": (
                "http://dist.gem5.org/dist/develop/"
                "test-progs/hello/bin/arm/linux/hello64-static"
            ),
            "source": "src/simple",
            "resource_version": "1.0.0",
            "gem5_versions": ["23.0"],
        }
        test_id = test_resource["id"]
        test_resource_version = test_resource["resource_version"]
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

        # add new version
        test_resource_new_version = copy.deepcopy(test_resource)
        test_resource_new_version["description"] = "test-description-2"
        test_resource_new_version["resource_version"] = "1.0.1"

        response = self.test_client.post(
            "/insert",
            json={"resource": test_resource_new_version, "alias": self.alias},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Inserted"})

        # get resource versions
        response = self.test_client.post(
            "/versions", json={"id": test_id, "alias": self.alias}
        )
        return_json = json.loads(response.data)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            return_json,
            [{"resource_version": "1.0.1"}, {"resource_version": "1.0.0"}],
        )
        # delete older version
        response = self.test_client.post(
            "/delete", json={"resource": test_resource, "alias": self.alias}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Deleted"})

        # get resource versions
        response = self.test_client.post(
            "/versions", json={"id": test_id, "alias": self.alias}
        )
        return_json = json.loads(response.data)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(return_json, [{"resource_version": "1.0.1"}])

    def test_find_add_new_version_update_older(self):
        test_resource = {
            "category": "binary",
            "id": "binary-example",
            "description": "binary-example documentation.",
            "architecture": "ARM",
            "is_zipped": False,
            "md5sum": "71b2cb004fe2cda4556f0b1a38638af6",
            "url": (
                "http://dist.gem5.org/dist/develop/"
                "test-progs/hello/bin/arm/linux/hello64-static"
            ),
            "source": "src/simple",
            "resource_version": "1.0.0",
            "gem5_versions": ["23.0"],
        }
        original_resource = test_resource.copy()
        test_id = test_resource["id"]
        test_resource_version = test_resource["resource_version"]
        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": test_resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)

        # add new version
        test_resource_new_version = copy.deepcopy(test_resource)
        test_resource_new_version["description"] = "test-description-2"
        test_resource_new_version["resource_version"] = "1.0.1"

        response = self.test_client.post(
            "/insert",
            json={"resource": test_resource_new_version, "alias": self.alias},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Inserted"})

        # get resource versions
        response = self.test_client.post(
            "/versions", json={"id": test_id, "alias": self.alias}
        )
        return_json = json.loads(response.data)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            return_json,
            [{"resource_version": "1.0.1"}, {"resource_version": "1.0.0"}],
        )

        resource_version = return_json[1]["resource_version"]

        # update older version
        test_resource["description"] = "test-description-3"

        response = self.test_client.post(
            "/update",
            json={
                "original_resource": original_resource,
                "resource": test_resource,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {"status": "Updated"})

        # find resource
        response = self.test_client.post(
            "/find",
            json={
                "id": test_id,
                "resource_version": resource_version,
                "alias": self.alias,
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.json == test_resource)
