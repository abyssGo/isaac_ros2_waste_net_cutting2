"""Vision 2 removal debug popup entry point."""

from .popup_common import run_popup


def main(args=None) -> None:
    run_popup("vision2", args=args)


if __name__ == "__main__":
    main()

