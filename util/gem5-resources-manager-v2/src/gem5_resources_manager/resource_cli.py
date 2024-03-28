import argparse

from .creator import Creator
from .updater import Updater

from .data_source import JSONDatabase, AtlasDataSource

subtools = [Creator()]


def cli():
    parser = argparse.ArgumentParser(
        description="CLI for gem5-resources.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    subparser = parser.add_subparsers()

    for subtool in subtools:
        subtool.get_arg_parser(subparser)

    args = parser.parse_args()

    datasource = AtlasDataSource(
        mongo_uri="", database_name="gem5-vision", collection_name="versions_test"
    )

    datasource.open()
    subtool.execute(args=args, data_source=datasource)
    datasource.close()
