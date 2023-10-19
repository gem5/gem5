import argparse

from .creator import Creator

from .data_source import DummyDatabase

subtools = [Creator()]


def cli():
    parser = argparse.ArgumentParser(
        description="CLI for gem5-resources.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    subparserer = parser.add_subparsers()

    parser.add_argument("obj")
    for subtool in subtools:
        subtool_parser = subtool.__class__.get_arg_parser()
        subtool_parser.set_defaults(obj=subtool)
        subparserer.add_parser(subtool.__class__.get_arg_parser())

    args = parser.parse_args()

    subtool = args.obj

    with DummyDatabase() as data_source:
        subtool.execute(args, data_source)
