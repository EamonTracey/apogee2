import datetime
import logging
from typing import Optional

import click

logger = logging.getLogger(__name__)


@click.command(context_settings={"show_default": True})
@click.option("-n",
              "--name",
              type=str,
              default=None,
              help="The name of the simulation, used to name log files.")
@click.option("-f",
              "--frequency",
              default=100,
              help="The speed at which to run the software loop.")
@click.option("-v",
              "--vehicle",
              default="drjoe",
              help="The vehicle to simulate.")
@click.option("-m",
              "--motor",
              default="aerotech_l1940x",
              help="The motor that provides thrust to the vehicle.")
@click.option("-e",
              "--environment",
              default="threeoaks_basic",
              help="The environment (e.g., temperature, pressure, wind).")
@click.option("-z",
              "--zenith",
              default=0.0,
              help="The zenith angle of the launch rail.")
@click.option("-a",
              "--azimuth",
              default=0.0,
              help="The azimuth angle of the launch rail.")
def simulate(name: Optional[str], vehicle: str, motor: str, environment: str,
             frequency: int, zenith: float, azimuth: float):
    """Run a complete software-in-the-loop rocket flight simulation."""
    from base.stage import Stage
    from simulation.dynamics import DynamicsComponent
    from simulation.environment import Environment
    from simulation.motor import Motor
    from simulation.simulation import Simulation
    from simulation.vehicle import Vehicle

    # Naming is hard.
    if name is None:
        utc_date = datetime.datetime.now(datetime.timezone.utc)
        utc_date_string = utc_date.strftime("%Y%m%d%H%M%S")
        name = f"ACS_simulation_{utc_date_string}"

    logging.basicConfig(
        filename=f"{name}.log",
        format="%(asctime)s:%(name)s:%(levelname)s:%(message)s",
        datefmt="%Y%m%d%H%M%S",
        level=logging.INFO)
    logger.info(
        "Apogee Control System. Alpha Kappa Sigma. \u0391\u039a\u03a3.")
    logger.info("Developed by the Notre Dame Rocketry Team.")
    logger.info(f"{name=}")

    vehicle_file = f"data/vehicles/{vehicle}.json"
    vehicle = Vehicle.from_json(vehicle_file)

    motor_file = f"data/motors/{motor}.json"
    motor = Motor.from_json(motor_file)

    environment_file = f"data/environments/{environment}.json"
    environment = Environment.from_json(environment_file)

    simulation = Simulation(name, frequency, vehicle, motor, environment,
                            zenith, azimuth)
    simulation.run()


if __name__ == "__main__":
    simulate()
