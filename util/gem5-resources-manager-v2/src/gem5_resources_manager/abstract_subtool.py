from abc import ABC
from argparse import ArgumentParser, Namespace


class AbstractSubtool(ABC):
    def get_arg_parser() -> ArgumentParser:
        raise NotImplementedError

    def execute(args: Namespace):
        raise NotImplementedError
