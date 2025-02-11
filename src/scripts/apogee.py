import click

from scripts.cfd import cfd
from scripts.fly import fly
from scripts.results import results
from scripts.simulate import simulate


@click.group()
def apogee():
    """A suite of software tools for the 2024-25 Apogee Control System.

    Authors: Eamon Tracey, Zach Ebner"""
    pass


apogee.add_command(cfd)
apogee.add_command(fly)
apogee.add_command(results)
apogee.add_command(simulate)

if __name__ == "__main__":
    apogee()
