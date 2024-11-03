from typing import Optional

import click

from flight.flight import Flight


@click.command(context_settings={"show_default": True})
@click.option("log_file",
              type=Optional[str],
              default=None,
              help="The path to which to write the log file")
def fly(log_file: Optional[str]):
    """Run ACS flight software."""

    flight = Flight(log_file)
    flight.run()


if __name__ == "__main__":
    fly()
