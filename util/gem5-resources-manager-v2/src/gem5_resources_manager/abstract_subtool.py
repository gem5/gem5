from abc import ABC
from argparse import ArgumentParser, Namespace
from .data_source.abstract_data_source import AbstractDataSource


class AbstractSubtool(ABC):
    def get_arg_parser() -> ArgumentParser:
        raise NotImplementedError

    def execute(args: Namespace, data_source: AbstractDataSource):
        raise NotImplementedError
