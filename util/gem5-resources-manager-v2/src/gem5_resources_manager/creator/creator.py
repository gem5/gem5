from argparse import ArgumentParser, Namespace
import json
import requests

from ..abstract_subtool import AbstractSubtool

from argparse import ArgumentParser

from ..loader import Loader
from ..data_source.abstract_data_source import AbstractDataSource


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

    def execute(args: Namespace, data_source: AbstractDataSource):
        data_source.create_new_entry("bla", "bla", "bla")

    def create_resource_json():
        pass


if __name__ == "__main__":
    Creator.execute(Creator.get_arg_parser().parse_args())
