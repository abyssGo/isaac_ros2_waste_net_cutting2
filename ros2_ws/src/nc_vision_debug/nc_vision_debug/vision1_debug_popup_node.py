"""Vision 1 cutting debug popup entry point."""

from .popup_common import run_popup


def main(args=None) -> None:
    run_popup("vision1", args=args)


if __name__ == "__main__":
    main()

