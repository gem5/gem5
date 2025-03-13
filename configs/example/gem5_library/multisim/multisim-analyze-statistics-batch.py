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

    # Check if we have cached results
    success = False
    cached = []
    cached_filename = Path("m5out/.cached_results")
    try:
        with open(cached_filename) as cached_file:
            cached = json.load(cached_file)

        print("Found cached files:")
        for i in cached:
            print(f"  - {i[0]}")

        success = True
    except OSError:
        print("No cached results found, running multisim...")

    # print(multisim_args)

    # Run the multisim
    if not success:
        results = run(module_path=Path(multisim_args.config))
        with open(cached_filename, "w") as cache:
            json.dump(results, cache)
    else:
        # pass
        results = cached
    # ----- End of multisim ----- #

    print(
        "##################### END OF SIMULATIONS ############################"
    )

    print(f"Host simulation time: {results[0][1]['hostSeconds']['value']}")


if __name__ == "__m5_main__":
    main()
