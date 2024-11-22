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
def fly(name: Optional[str]):
    """Run ACS flight software."""
    from flight.flight import Flight

    # Naming is hard.
    if name is None:
        utc_date = datetime.datetime.now(datetime.UTC)
        utc_date_string = utc_date.strftime("%Y%m%d%H%M%S")
        name = f"ACS {utc_date_string}"

    # Initialize logging.
    logging.basicConfig(
        filename=f"{name}.log",
        format="%(asctime)s:%(name)s:%(levelname)s:%(message)s",
        datefmt="%Y%m%d%H%M%S",
        level=logging.INFO)
    logger.info("Apogee Control System. Alpha Kappa Sigma. \u0391\u039a\u03a3.")
    logger.info("Developed by the Notre Dame Rocketry Team.")
    logger.info(" ")
    logger.info(f"{name=}")
    logger.info(" ")
    logger.info("Jokes.")
    logger.info("What did one flap say to the other flap? Smells like apogee")
    logger.info("ACS needed a ballast, so we put Will's mom in the module. Problem was, we CATO'd.")
    logger.info(" ")

    flight = Flight(name)
    flight.run()


if __name__ == "__main__":
    fly()
