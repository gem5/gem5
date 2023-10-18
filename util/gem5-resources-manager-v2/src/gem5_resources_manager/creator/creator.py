from argparse import ArgumentParser, Namespace
import json
import requests

from ..abstract_subtool import AbstractSubtool

from argparse import ArgumentParser

from ..loader import Loader

from ..helper import (
    enter_fields,
    get_database,
    validate_resources,
    get_fields,
    save_file,
    check_resource_exists,
)


class Creator(AbstractSubtool):
    # Implement the two abstract classes declared in AbstractSubtool.
    def get_arg_parser() -> ArgumentParser:
        parser = ArgumentParser("gem5-resources-creator")
        parser.add_argument("category", type=str, help="category of resource to create")
        parser.add_argument(
            "--output",
            "-o",
            type=str,
            help="output file",
            default="resources.json",
        )
        parser.add_argument(
            "--ignore-schema-validation",
            "-i",
            action="store_true",
            help="ignore validation of resource against schema",
        )
        parser.add_argument(
            "--ignore-db-check",
            "-d",
            action="store_true",
            help="ignore the check that the entered resource does not exist in the database.",
        )
        parser.add_argument(
            "--handle-url",
            "-u",
            action="store_true",
            help="automatically fill in the is_zipped, is_tarred, and md5sum fields if the url field has a link to the source repository.",
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="show more information about the fields",
        )
        parser.add_argument(
            "--required-fields-only",
            "-r",
            action="store_false",
            help="only show required fields when entering data",
        )
        parser.add_argument(
            "--field-entries",
            "-f",
            type=str,
            help="json file dictionary containing field entries",
            default="{}",
        )

        return parser

    def execute(args: Namespace):
        print("args", args)
        with Loader("Connecting to MongoDB...", end="Connected to MongoDB"):
            collection = get_database()
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
            enter_fields(optional, resource, populated_fields, args, is_optional=True)
        if not args.ignore_db_check and check_resource_exists(resource):
            # throw runtime exception
            raise Exception("Resource already exists in database")
        if not args.ignore_schema_validation:
            validate_resources([resource])
        save_file(resource, args.output)


if __name__ == "__main__":
    Creator.execute(Creator.get_arg_parser().parse_args())
