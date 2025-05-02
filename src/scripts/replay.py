import datetime
import logging
from typing import Optional

import click

logger = logging.getLogger(__name__)


@click.command(context_settings={"show_default": True})
@click.argument("path", type=click.STRING, nargs=-1, required=True)
@click.option("-n",
              "--name",
              type=str,
              default=None,
              help="The name of the flight, used to name log files.")
@click.option("-v",
              "--vehicle",
              default="fullscale25",
              help="The vehicle being flown.")
@click.option("-m",
              "--motor",
              default="aerotech_l1940x",
              help="The motor being flown.")
@click.option("-e",
              "--environment",
              default="threeoaks_basic",
              help="The launch environment.")
def replay(name: Optional[str], path: str, vehicle: str, motor: str,
           environment: str):
    """Run ACS replay software."""
    from ereplay.replay import Replay
    from simulation.environment import Environment
    from simulation.motor import Motor
    from simulation.vehicle import Vehicle

    # Naming is hard.
    if name is None:
        utc_date = datetime.datetime.now(datetime.timezone.utc)
        utc_date_string = utc_date.strftime("%Y%m%d%H%M%S")
        name = f"ACS_Replay_{utc_date_string}"

    # File to replay from.
    path = " ".join(path)

    # Initialize logging.
    logging.basicConfig(
        filename=f"{name}.log",
        format="%(asctime)s:%(name)s:%(levelname)s:%(message)s",
        datefmt="%Y%m%d%H%M%S",
        level=logging.INFO)
    logger.info("Apogee Control System Replay. \u0391\u039a\u03a3.")
    logger.info("Developed by the Notre Dame Rocketry Team.")
    logger.info(f"{name=}")

    # Load launch condition parameters.
    vehicle_file = f"data/vehicles/{vehicle}.json"
    vehicle = Vehicle.from_json(vehicle_file)
    motor_file = f"data/motors/{motor}.json"
    motor = Motor.from_json(motor_file)
    environment_file = f"data/environments/{environment}.json"
    environment = Environment.from_json(environment_file)

    replay = Replay(name, vehicle, motor, environment, path)
    replay.run()


if __name__ == "__main__":
    replay()
