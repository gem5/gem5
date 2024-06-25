# configs/example/gem5_library/gema/host.py
# Example host to run gEMA instance.

import argparse

from gem5.utils.gema import *

parser = argparse.ArgumentParser("gEMA host")
parser.add_argument("port", help="Port to use for API", type=int)
args = parser.parse_args()

if __name__ == "__m5_main__":
    app = Gema(port=args.port)
    app.run()
