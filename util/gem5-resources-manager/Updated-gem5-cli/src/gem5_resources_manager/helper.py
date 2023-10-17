from pymongo import MongoClient
import urllib.request
import zipfile
import tarfile
from typing import Dict, List
from bson import json_util
import hashlib
import os
import json
import requests
import jsonschema
from dotenv import load_dotenv
import ast

load_dotenv()
MONGO_URI = os.getenv("MONGO_URI")


def get_database(collection="versions_test", uri=MONGO_URI, db="gem5-vision"):
    """
    Retrieves the MongoDB database for gem5-vision.
    """
    CONNECTION_STRING = uri
    try:
        client = MongoClient(CONNECTION_STRING)
        client.server_info()
    except:
        print("\nCould not connect to MongoDB")
        exit(1)
    return client[db][collection]


collection = get_database()


def validate_resources(resources: List) -> bool:
    schema = json.loads(
        requests.get("https://resources.gem5.org/gem5-resources-schema.json").content
    )
    for resource in resources:
        validator = jsonschema.Draft7Validator(schema)

        is_resource_valid = validator.is_valid(resource)
        if not is_resource_valid:
            print(
                f"\nResource with 'id': '{resource['id']}' and 'resource_version': '{resource['resource_version']}' is invalid."
            )
            for error in validator.iter_errors(resource):
                print(f"\n- {error.path}: {error.message}")

            return False
    return True


def check_resource_exists(resource):
    global collection
    result = (
        collection.find(
            {
                "id": resource["id"],
                "resource_version": resource["resource_version"],
            },
            {"_id": 0},
        )
        .sort("resource_version", -1)
        .limit(1)
    )
    json_resource = json_util.dumps(result)
    res = json.loads(json_resource)
    if res == []:
        return False
    return True


def enter_fields(fields, resource, populated_fields, args, is_optional=False):
    for field in fields.keys():
        if field in populated_fields:
            resource[field] = populated_fields[field]
            print(f"{field}: {resource[field]}")
            if field == "url" and args.handle_url:
                handle_url(resource[field], populated_fields)
            continue
        if is_optional:
            choice = input(
                f"'{field}' is an optional field. Enter Y to enter a value for this field, or N to skip this field."
            )
            if choice.lower() == "n":
                continue
            elif choice.lower() != "y":
                raise Exception("Invalid input")
        if args.verbose:
            print(
                f"The {field} takes the following input: {json.dumps(fields[field], indent=4)}"
            )
        user_input = input(f"{field}: ")
        if (
            fields[field]["type"] == "integer"
            or fields[field]["type"] == "array"
            or fields[field]["type"] == "object"
        ):
            resource[field] = ast.literal_eval(user_input)
        else:
            resource[field] = user_input
        if field == "url" and args.handle_url:
            handle_url(resource[field], populated_fields)


def handle_url(url, populated_fields):
    urllib.request.urlretrieve(url, "temp")
    is_zipped = zipfile.is_zipfile("temp")
    is_tarred = tarfile.is_tarfile("temp")
    # if url ends with .tar or .tar.gz, then it is tarred
    if url.endswith(".tar"):
        is_tarred = True
    if url.endswith(".tar.gz"):
        is_tarred = True
        is_zipped = True

    def file_as_bytes(file):
        with file:
            return file.read()

    populated_fields["is_zipped"] = is_zipped
    populated_fields["is_tar_archive"] = is_tarred
    populated_fields["md5sum"] = hashlib.md5(
        file_as_bytes(open("temp", "rb"))
    ).hexdigest()


def save_file(resource, output):
    # check if output path is a directory
    if os.path.isdir(output):
        # if it is then create resources.json in that directory
        output = os.path.join(output, "resources.json")
    existing_resources = get_resource_from_file(output)
    if existing_resources is not None:
        existing_resources.append(resource)
        with open(output, "w") as outfile:
            json.dump(existing_resources, outfile, indent=4)
    else:
        with open(output, "w") as outfile:
            json.dump([resource], outfile, indent=4)
    print("final resource", json.dumps(resource, indent=4))


def get_resource_from_file(file):
    # append resource to resources.json
    resources = None
    if os.path.exists(file):
        with open(file, "r") as infile:
            resources = json.load(infile)
    return resources


def get_fields(category, schema):
    optional = {}
    required = {}
    for field in schema["properties"]:

        default = schema["properties"][field]
        if field in schema["required"]:
            required[field] = default
        else:
            optional[field] = default

    definitions = []
    for definition in schema["definitions"][category]["allOf"]:
        definitions.append(definition["$ref"].split("/")[-1])

    for definition in definitions:
        for field in schema["definitions"][definition]["properties"]:
            default = schema["definitions"][definition]["properties"][field]
            if field in optional.keys():
                required[field] = optional[field]
                del optional[field]
            elif (
                "required" not in schema["definitions"][definition]
                or field in schema["definitions"][definition]["required"]
            ):
                required[field] = default
            else:
                optional[field] = default
    if "architecture" in required:
        required["architecture"] = schema["definitions"]["architecture"]
    if "architecture" in optional:
        optional["architecture"] = schema["definitions"]["architecture"]

    return required, optional


def get_latest_resource(id):
    resource = (
        collection.find(
            {
                "id": id,
            },
            {"_id": 0},
        )
        .sort("resource_version", -1)
        .limit(1)
    )

    json_resource = json_util.dumps(resource)
    res = json.loads(json_resource)
    return res


def get_updated_rsource_version(current_resource_version):
    is_update = input("Is the resource version being updated? (Y/N): ")
    if is_update.lower() == "y":
        version_subparts = current_resource_version.split(".")
        version_subparts[0] = str(int(version_subparts[0]) + 1)
        new_version = (
            version_subparts[0] + "." + version_subparts[1] + "." + version_subparts[2]
        )
        return new_version
    return current_resource_version


def update_resource(resource, new_resource_version, update_non_existing_fields):
    updated_resource = {}
    print(resource)
    for field_key, field_value in resource.items():
        if field_key == "resource_version":
            updated_resource[field_key] = new_resource_version
            print(f"{field_key}: {updated_resource[field_key]}")
            continue
        print(f"\n\nCurrent value of {field_key}")
        print(f"{field_key}: {field_value}")
        is_updated = input("Do you want to update this field? (Y/N): ")
        if is_updated.lower() == "y":
            updated_resource[field_key] = input(f"Enter new value for {field_key}: ")
        else:
            updated_resource[field_key] = field_value

    if update_non_existing_fields:
        print("\n\nUpdating non-existing fields\n\n")
        _, optional_fields = get_fields(
            resource["category"],
            json.loads(
                requests.get(
                    "https://resources.gem5.org/gem5-resources-schema.json"
                ).content
            ),
        )
        for optional_field in optional_fields.keys():
            if optional_field not in resource.keys():
                print(f"Update the following optional field: {optional_field}? (Y/N):")
                if input().lower() != "y":
                    continue
                print(
                    f"The {optional_field} takes the following input: {json.dumps(optional_fields[optional_field], indent=4)}"
                )
                updated_resource[optional_field] = input(
                    f"Enter value for {optional_field}: "
                )

    return updated_resource
