import argparse

from .creator import Creator

subtools = [Creator()]


def cli():
    parser = argparse.ArgumentParser(
        description="CLI for gem5-resources.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    subparserer = parser.add_subparsers()

    for subtool in subtools:
        subtool_parser = subtool.get_arg_parser()
        subtool_parser.set_defaults(subtool=subtool)
        subparserer.add_parser(subtool.get_arg_parser())

    args = parser.parse_args()

    subtool = args.subtool

    subtool.execute(args)
