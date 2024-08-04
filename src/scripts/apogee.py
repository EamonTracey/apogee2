import click

from scripts.simulate import simulate

@click.group()
def apogee():
    """A suite of commands for NDRT's 2024-25 Apogee Control System."""
    pass
apogee.add_command(simulate)

if __name__ == "__main__":
    apogee()
