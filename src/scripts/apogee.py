import click

from scripts.cfd import cfd
from scripts.fly import fly
from scripts.plot import plot
from scripts.simulate import simulate
from scripts.replay import replay


@click.group()
def apogee():
    """A suite of software tools for the 2024-25 Apogee Control System.

    Authors: Eamon Tracey, Zach Ebner"""
    pass


apogee.add_command(cfd)
apogee.add_command(fly)
apogee.add_command(plot)
apogee.add_command(simulate)
apogee.add_command(replay)

if __name__ == "__main__":
    apogee()
