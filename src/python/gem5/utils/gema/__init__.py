# gema/__init__.py
# gema: gem5 External Modules API

__version__ = "1.0"
__all__ = ["Gema"]

# Import the gEMA class from the gEMA.py file
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.info(f"initialized. version: {__version__}")

from gem5.utils.gema.config import (
    GemaConfigGenerator,
    GemaConfigRetreiver,
)
from gem5.utils.gema.manager import GemaSimulationManager
from gem5.utils.gema.server import GemaServer
from gem5.utils.multiprocessing import Process


class Gema:
    """Main application class."""

    def __init__(self, port: int):
        """Initialize the gEMA super class."""
        self.configurator = GemaConfigGenerator(self)
        self.retriever = GemaConfigRetreiver(self)
        self.manager = GemaSimulationManager(self)
        self.server = GemaServer(self, port)
        self.sims = {}

    def run(self):
        """Run a gEMA instance."""
        # main = Process(target=self.server.run(), name="gema")
        # main.run()
        self.server.run()
