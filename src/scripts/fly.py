import click

from flight.flight import Flight


@click.command(context_settings={"show_default": True})
def fly():
    """Run ACS flight software."""


if __name__ == "__main__":
    fly()
