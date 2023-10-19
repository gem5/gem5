from argparse import ArgumentParser, Namespace
import json
import requests

from ..abstract_subtool import AbstractSubtool

from argparse import ArgumentParser

# from ..loader import Loader
from ..data_source.abstract_data_source import AbstractDataSource
import ast
import hashlib
import tarfile
import urllib.request
import zipfile


class Creator(AbstractSubtool):
    def __init__(self) -> None:
        self.data_source = None

    # Implement the two abstract classes declared in AbstractSubtool.
    def get_arg_parser(self, subparser):
        parser = subparser.add_parser("gem5-resources-creator")
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

    def execute(self, args: Namespace, data_source: AbstractDataSource):
        self.data_source = data_source
        print("Creator.execute()", args.category)
        self.create_resource_json(args)

    def create_resource_json(self, args: Namespace):
        resource = {}
        required, optional = self.data_source.get_fields(args.category)
        populated_fields = ast.literal_eval(args.field_entries)
        resource["category"] = args.category
        del required["category"]

        self.enter_fields(required, resource, populated_fields, args)
        if args.required_fields_only:
            self.enter_fields(
                optional, resource, populated_fields, args, is_optional=True
            )

        if not args.ignore_db_check and self.data_source.resource_exists(
            resource_id=resource["id"]
        ):
            # throw runtime exception
            raise Exception("Resource already exists in database")

        if not args.ignore_schema_validation:
            self.data_source.validate_resource(resource)
        self.data_source.create_new_entry(resource)

    def enter_fields(self, fields, resource, populated_fields, args, is_optional=False):
        for field in fields.keys():
            if field in populated_fields:
                resource[field] = populated_fields[field]
                print(f"{field}: {resource[field]}")
                if field == "url" and args.handle_url:
                    self.handle_url(resource[field], populated_fields)
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
                self.handle_url(resource[field], populated_fields)

    def handle_url(self, url, populated_fields):
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


if __name__ == "__main__":
    Creator.execute(Creator.get_arg_parser().parse_args())
