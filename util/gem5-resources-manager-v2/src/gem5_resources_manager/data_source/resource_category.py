from enum import Enum


class ResourceCategory(Enum):
    BINARY = "BinaryResource"
    DIRECTORY = "DirectoryResource"
    SIMPOINT = "SimpointResource"

    def __str__(self):
        return self.value
