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
def fly(name: Optional[str], vehicle: str, motor: str, environment: str):
    """Run ACS flight software."""
    from flight.flight import Flight
    from simulation.environment import Environment
    from simulation.motor import Motor
    from simulation.vehicle import Vehicle

    # Naming is hard.
    if name is None:
        utc_date = datetime.datetime.now(datetime.timezone.utc)
        utc_date_string = utc_date.strftime("%Y%m%d%H%M%S")
        name = f"ACS_{utc_date_string}"

    # Initialize logging.
    logging.basicConfig(
        filename=f"{name}.log",
        format="%(asctime)s:%(name)s:%(levelname)s:%(message)s",
        datefmt="%Y%m%d%H%M%S",
        level=logging.INFO)
    logger.info(
        "Apogee Control System. Alpha Kappa Sigma. \u0391\u039a\u03a3.")
    logger.info("Developed by the Notre Dame Rocketry Team.")
    logger.info(f"{name=}")
    logger.info("Jokes.")
    logger.info("What did one flap say to the other flap? Smells like apogee.")
    logger.info(
        "ACS needed ballast, so we put Will's mom in the module. Problem was, "
        "we CATO'd.")
    logger.info(
        "The rocket asked, \"What's your goal in life?\" ACS replied, \"Just "
        "trying to get as far away from here as possible.\"")
    logger.info(
        "Knock knock. Who's there? Apogee. Apogee who? Apogee-z, Will's mom "
        "smells bad.")
    logger.info("Skibidi")
    logger.info("It's beginning to look a gyatt like Rizzmas"
                "Everywhere online"
                "My sigma level is maxed"
                "Your food has fanum taxed"
                "With skibidis and rizzlers in Ohio"
                "It's beginning to look a gyatt like Rizzmas"
                "Baby Gronk rizzed up Livvy"
                "But the prettiest sight to see"
                "Is how skibidi they will be"
                "Did you pray today?")
    logger.info("ACS operates on a TeasleysMom model of the gravity.")

    logger.info("The way he's actuating is so tuff.")

    # Load launch condition parameters.
    vehicle_file = f"data/vehicles/{vehicle}.json"
    vehicle = Vehicle.from_json(vehicle_file)
    motor_file = f"data/motors/{motor}.json"
    motor = Motor.from_json(motor_file)
    environment_file = f"data/environments/{environment}.json"
    environment = Environment.from_json(environment_file)

    flight = Flight(name, vehicle, motor, environment)
    flight.run()


if __name__ == "__main__":
    fly()
