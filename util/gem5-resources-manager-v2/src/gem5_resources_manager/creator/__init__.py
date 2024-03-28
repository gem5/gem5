from .creator import Creator


def main():
    Creator.execute(Creator.get_arg_parser().parse_args())


if __name__ == "__main__":
    main()
