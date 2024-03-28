from .abstract_data_source import AbstractDataSource
from .resource_category import ResourceCategory
from .exception import Gem5DataSourceSchemaViolation, Gem5DataSourceEntryNotFound
from ..schemas.json_validator import JSONValidator
from pymongo.errors import ConnectionFailure, ConfigurationError
from pymongo import MongoClient
from typing import Any, Dict, Optional
from bson import json_util
import json
import os


class AtlasDataSource(AbstractDataSource):
    def __init__(self, mongo_uri, database_name, collection_name) -> None:
        super().__init__()
        self.mongo_uri = mongo_uri
        self.database_name = database_name
        self.collection_name = collection_name
        self.validator = JSONValidator(
            "https://resources.gem5.org/gem5-resources-schema.json"
        )

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return super().__exit__(exc_type, exc_value, traceback)

    def open(self):
        try:
            client = MongoClient(self.mongo_uri)
            client.admin.command("ping")
        except ConnectionFailure:
            client.close()
            raise ConnectionFailure("Failed to connect to MongoDB")
        except ConfigurationError:
            client.close()
            raise ConfigurationError("Invalid MongoDB URI")

        database = client[self.database_name]
        if database.name not in client.list_database_names():
            client.close()
            raise ConnectionFailure("Database does not exist")
        collection = database[self.collection_name]
        if collection.name not in database.list_collection_names():
            client.close()
            raise ConnectionFailure("Collection does not exist")

        self.collection = collection

    def close(self):
        pass

    def resource_exists(
        self, resource_id: str, resource_version: Optional[str] = "1.0.0"
    ) -> bool:
        resource = (
            self.collection.find(
                {
                    "id": resource_id,
                    "resource_version": resource_version,
                },
                {"_id": 0},
            )
            .sort("resource_version", -1)
            .limit(1)
        )
        json_resource = json_util.dumps(resource)
        res = json.loads(json_resource)
        if res == []:
            return False
        return True

    def get_resource_category(
        self, resource_id: str, resource_version: str
    ) -> ResourceCategory:
        resource = (
            self.collection.find(
                {
                    "id": resource_id,
                    "resource_version": resource_version,
                },
                {"_id": 0},
            )
            .sort("resource_version", -1)
            .limit(1)
        )
        json_resource = json_util.dumps(resource)
        res = json.loads(json_resource)
        if res == []:
            raise Gem5DataSourceEntryNotFound("Entry not found")
        return ResourceCategory(res[0]["category"])

    def find_resource(self, resource_id: str, resource_version: str):
        resource = self.collection.find(
            {
                "id": resource_id,
                "resource_version": resource_version,
            },
            {"_id": 0},
        )
        json_resource = json_util.dumps(resource)
        res = json.loads(json_resource)
        if res == []:
            raise Gem5DataSourceEntryNotFound(resource_id, resource_version)
        return res[0]

    def find_latest_resource(self, resource_id: str):
        resource = (
            self.collection.find(
                {
                    "id": resource_id,
                },
                {"_id": 0},
            )
            .sort("resource_version", -1)
            .limit(1)
        )
        json_resource = json_util.dumps(resource)
        res = json.loads(json_resource)
        print("res", res)
        if res == []:
            raise Gem5DataSourceEntryNotFound(resource_id)
        return res[0]

    def get_resource_from_file(self, file):
        # append resource to resources.json
        resources = None
        if os.path.exists(file):
            with open(file) as infile:
                resources = json.load(infile)
        return resources

    def validate_resource(self, resource: Dict[str, Any]):
        return self.validator.validate([resource])

    def get_fields(self, category):
        return self.validator.get_fields(category)

    def get_changed_fields(self, resource: Dict[str, Any]):
        return self.validator.get_changed_fields(resource)

    def get_all_resources_by_category(self, category):
        pipeline = [
            # Match Stage: Filter by category
            {"$match": {"category": category}},
            # Sort Stage: Sort by 'id' and 'resource_version' in descending order
            {"$sort": {"id": 1, "resource_version": -1}},
            # Group Stage: Group by 'id' and take the first document
            {
                "$group": {
                    "_id": "$id",
                    "document": {
                        "$first": "$$ROOT"
                    },  # '$$ROOT' represents the whole document
                }
            },
            # Replace Root Stage: Replace the root to bring the document to the top level
            {"$replaceRoot": {"newRoot": "$document"}},
        ]

        resources = self.collection.aggregate(
            pipeline,
        )
        json_resource = json_util.dumps(resources)
        res = json.loads(json_resource)
        return res

    def save_file(self, resource, output):
        # check if output path is a directory
        if os.path.isdir(output):
            # if it is then create resources.json in that directory
            output = os.path.join(output, "resources.json")
        existing_resources = self.get_resource_from_file(output)
        if existing_resources is not None:
            existing_resources.append(resource)
            with open(output, "w") as outfile:
                json.dump(existing_resources, outfile, indent=4)
        else:
            with open(output, "w") as outfile:
                json.dump([resource], outfile, indent=4)
        print("final resource", json.dumps(resource, indent=4))

    def create_new_entry(self, resource: Dict) -> None:
        self.save_file(resource, "new_resources.json")
