from .updater import Updater


def main():
    Updater.execute(Updater.get_arg_parser().parse_args())


if __name__ == "__main__":
    main()
