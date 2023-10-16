import json
import argparse
import requests
import ast
from loader import Loader
from helper import (
    validate_resources,
    check_resource_exists,
    enter_fields,
    get_resource_from_file,
    get_database,
    save_file,
    get_fields,
    get_latest_resource,
    get_updated_rsource_version,
    update_resource,
)
import helper


def cli():
    parser = argparse.ArgumentParser(
        description="CLI for gem5-resources.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    global collection
    # add subparsers
    subparsers = parser.add_subparsers(
        help="sub-command help", dest="subparser_name"
    )
    resource_creator_parser = subparsers.add_parser("createResources")
    resource_creator_parser.add_argument(
        "category", type=str, help="category of resource to create"
    )
    resource_creator_parser.add_argument(
        "--output",
        "-o",
        type=str,
        help="output file",
        default="resources.json",
    )
    resource_creator_parser.add_argument(
        "--ignore-schema-validation",
        "-i",
        action="store_true",
        help="ignore validation of resource against schema",
    )
    resource_creator_parser.add_argument(
        "--ignore-db-check",
        "-d",
        action="store_true",
        help="ignore the check that the entered resource does not exist in the database.",
    )
    resource_creator_parser.add_argument(
        "--handle-url",
        "-u",
        action="store_true",
        help="automatically fill in the is_zipped, is_tarred, and md5sum fields if the url field has a link to the source repository.",
    )
    resource_creator_parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="show more information about the fields",
    )
    resource_creator_parser.add_argument(
        "--required-fields-only",
        "-r",
        action="store_false",
        help="only show required fields when entering data",
    )
    resource_creator_parser.add_argument(
        "--field-entries",
        "-f",
        type=str,
        help="json file dictionary containing field entries",
        default="{}",
    )
    resource_creator_parser.set_defaults(func=create_resources)

    resource_validator_parser = subparsers.add_parser("validateResources")
    resource_validator_parser.add_argument(
        "--input", "-i", type=str, help="input file", default="resources.json"
    )
    resource_validator_parser.set_defaults(func=validate_resources)

    resource_updater_parser = subparsers.add_parser("updateResources")
    resource_updater_parser.add_argument(
        "id", type=str, help="id of resource to that was just updated"
    )
    resource_updater_parser.add_argument(
        "--update-non-existing-fields",
        "-u",
        action="store_true",
        help="update fields that are not defined in the JSON object",
    )
    resource_updater_parser.add_argument(
        "--output",
        "-o",
        type=str,
        help="output file",
        default="resources.json",
    )
    resource_updater_parser.set_defaults(func=update_resources)
    args = parser.parse_args()
    args.func(args)


def validate_resources(args):
    resources = get_resource_from_file(args.input)
    if validate_resources(resources):
        print("Resources are valid")


def create_resources(args):
    print("args", args)
    with Loader("Connecting to MongoDB...", end="Connected to MongoDB"):
        helper.collection = get_database()
    schema = json.loads(
        requests.get(
            "https://resources.gem5.org/gem5-resources-schema.json"
        ).content
    )
    resource = {}
    required, optional = get_fields(args.category, schema)
    # print("required", json.dumps(required, indent=4))
    # print("optional", json.dumps(optional, indent=4))
    populated_fields = ast.literal_eval(args.field_entries)
    # print("populated_fields", json.dumps(populated_fields, indent=4))
    resource["category"] = args.category
    del required["category"]
    enter_fields(required, resource, populated_fields, args)
    if args.required_fields_only:
        enter_fields(
            optional, resource, populated_fields, args, is_optional=True
        )
    if not args.ignore_db_check and check_resource_exists(resource):
        # throw runtime exception
        raise Exception("Resource already exists in database")
    if not args.ignore_schema_validation:
        validate_resources([resource])
    save_file(resource, args.output)


def update_resources(args):
    resource = get_latest_resource(args.id)[0]
    new_resource_version = get_updated_rsource_version(
        resource["resource_version"]
    )
    updated_resource = update_resource(
        resource, new_resource_version, args.update_non_existing_fields
    )
    save_file(updated_resource, args.output)


if __name__ == "__main__":
    cli()
