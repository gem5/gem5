import json

from m5.stats.gem5stats import SimStat

from gem5.utils.multisim.multisim import (
    module_run,
    run,
)


def main():

    # ----- Multisim entrypoint ----- #
    import argparse
    from pathlib import Path

    global module_run
    module_run = True

    multisim_parser = argparse.ArgumentParser(
        description="The geof parser for geof args",
    )

    multisim_parser.add_argument(
        "config",
        help="Path to the config file to run",
    )

    multisim_args = multisim_parser.parse_args()

    results = run(module_path=Path(multisim_args.config))
    # ----- End of multisim ----- #

    print(
        "##################### END OF SIMULATIONS ############################"
    )

    for sim, stats in results:
        print(
            f"Host simulation time for {sim}: {stats['hostSeconds']['value']}"
        )


if __name__ == "__m5_main__":
    main()
