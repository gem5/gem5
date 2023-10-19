import argparse

from .creator import Creator

from .data_source import JSONDatabase

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

    datasource = JSONDatabase("resources.json")
    datasource.open()
    subtool.execute(args=args, data_source=datasource)
    datasource.close()
