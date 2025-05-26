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
from typing import (
    Dict,
    List,
)

import pymongo
from api.client import Client
from bson import json_util
from pymongo import MongoClient
from pymongo.errors import (
    ConfigurationError,
    ConnectionFailure,
)


class DatabaseConnectionError(Exception):
    "Raised for failure to connect to MongoDB client"

    pass


class MongoDBClient(Client):
    def __init__(self, mongo_uri, database_name, collection_name):
        super().__init__()
        self.mongo_uri = mongo_uri
        self.collection_name = collection_name
        self.database_name = database_name
        self.collection = self._get_database(
            mongo_uri, database_name, collection_name
        )

    def _get_database(
        self,
        mongo_uri: str,
        database_name: str,
        collection_name: str,
    ) -> pymongo.collection.Collection:
        """
        This function returns a MongoDB database object for the specified
        collection.
        It takes three arguments: 'mongo_uri', 'database_name', and
        'collection_name'.

        :param: mongo_uri: URI of the MongoDB instance
        :param: database_name: Name of the database
        :param: collection_name: Name of the collection
        :return: database: MongoDB database object
        """

        try:
            client = MongoClient(mongo_uri)
            client.admin.command("ping")
        except ConnectionFailure:
            client.close()
            raise DatabaseConnectionError(
                "Could not connect to MongoClient with given URI!"
            )
        except ConfigurationError as e:
            raise DatabaseConnectionError(e)

        database = client[database_name]
        if database.name not in client.list_database_names():
            raise DatabaseConnectionError("Database Does not Exist!")

        collection = database[collection_name]
        if collection.name not in database.list_collection_names():
            raise DatabaseConnectionError("Collection Does not Exist!")

        return collection

    def find_resource(self, query: Dict) -> Dict:
        """
        Find a resource in the database

        :param query: JSON object with id and resource_version
        :return: json_resource: JSON object with request resource or
        error message
        """
        if "resource_version" not in query or query["resource_version"] == "":
            resource = (
                self.collection.find({"id": query["id"]}, {"_id": 0})
                .sort("resource_version", -1)
                .limit(1)
            )
        else:
            resource = (
                self.collection.find(
                    {
                        "id": query["id"],
                        "resource_version": query["resource_version"],
                    },
                    {"_id": 0},
                )
                .sort("resource_version", -1)
                .limit(1)
            )
        json_resource = json_util.dumps(resource)
        res = json.loads(json_resource)
        if res == []:
            return {"exists": False}
        return res[0]

    def update_resource(self, query: Dict) -> Dict[str, str]:
        """
        This function updates a resource in the database by first checking if
        the resource version in the request matches the resource version
        stored in the database.
        If they match, the resource is updated in the database. If they do not
        match, the update is rejected.

        :param: query: JSON object with original_resource and the
        updated resource
        :return: json_response: JSON object with status message
        """
        original_resource = query["original_resource"]
        modified_resource = query["resource"]
        try:
            self.collection.replace_one(
                {
                    "id": original_resource["id"],
                    "resource_version": original_resource["resource_version"],
                },
                modified_resource,
            )
        except Exception as e:
            print(e)
            return {"status": "Resource does not exist"}
        return {"status": "Updated"}

    def get_versions(self, query: Dict) -> List[Dict]:
        """
        This function retrieves all versions of a resource with the given ID
        from the database.
        It takes two arguments, the database object and a JSON object
        containing the 'id' key of the resource to be retrieved.

        :param: query: JSON object with id
        :return: json_resource: JSON object with all resource versions
        """
        versions = self.collection.find(
            {"id": query["id"]}, {"resource_version": 1, "_id": 0}
        ).sort("resource_version", -1)
        # convert to json
        res = json_util.dumps(versions)
        return json_util.loads(res)

    def delete_resource(self, query: Dict) -> Dict[str, str]:
        """
        This function deletes a resource from the database by first checking
        if the resource version in the request matches the resource version
        stored in the database.
        If they match, the resource is deleted from the database. If they do
        not match, the delete operation is rejected

        :param: query: JSON object with id and resource_version
        :return: json_response: JSON object with status message
        """
        self.collection.delete_one(
            {"id": query["id"], "resource_version": query["resource_version"]}
        )
        return {"status": "Deleted"}

    def insert_resource(self, query: Dict) -> Dict[str, str]:
        """
        This function inserts a new resource into the database using the
        'insert_one' method of the MongoDB client.
        The function takes two arguments, the database object and the JSON
        object representing the new resource to be inserted.

        :param: json: JSON object representing the new resource to be inserted
        :return: json_response: JSON object with status message
        """
        try:
            self.collection.insert_one(query)
        except Exception as e:
            return {"status": "Resource already exists"}
        return {"status": "Inserted"}

    def check_resource_exists(self, query: Dict) -> Dict:
        """
        This function checks if a resource exists in the database by searching
        for a resource with a matching 'id' and 'resource_version' in
        the database.
        The function takes two arguments, the database object and a JSON object
        containing the 'id' and 'resource_version' keys.

        :param: json: JSON object with id and resource_version
        :return: json_response: JSON object with boolean 'exists' key
        """
        resource = (
            self.collection.find(
                {
                    "id": query["id"],
                    "resource_version": query["resource_version"],
                },
                {"_id": 0},
            )
            .sort("resource_version", -1)
            .limit(1)
        )
        json_resource = json_util.dumps(resource)
        res = json.loads(json_resource)
        if res == []:
            return {"exists": False}
        return {"exists": True}

    def save_session(self) -> Dict:
        """
        This function saves the client session to a dictionary.
        :return: A dictionary containing the client session.
        """
        session = {
            "client": "mongodb",
            "uri": self.mongo_uri,
            "database": self.database_name,
            "collection": self.collection_name,
        }
        return session
