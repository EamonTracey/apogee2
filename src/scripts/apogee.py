import click

from scripts.simulate import simulate


@click.group()
def apogee():
    """A suite of software tools for the 2024-25 Apogee Control System.

    Authors: Eamon Tracey"""
    pass


apogee.add_command(simulate)

if __name__ == "__main__":
    apogee()
