from typing import Optional

import click


@click.command(context_settings={"show_default": True})
@click.option("-l",
              "--log_file",
              type=Optional[str],
              default=None,
              help="The path to which to write the log file")
def fly(log_file: Optional[str]):
    """Run ACS flight software."""
    from flight.flight import Flight

    flight = Flight(log_file)
    flight.run()


if __name__ == "__main__":
    fly()
