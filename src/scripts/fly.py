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
        "we CATO'd."
    )
    logger.info(
        "The rocket asked, \"What's your goal in life?\" ACS replied, \"Just "
        "trying to get as far away from here as possible.\""
    )
    logger.info(
        "Knock knock. Who's there? Apogee. Apogee who? Apogee-z, Will's mom "
        "smells bad."
    )
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

    flight = Flight(name)
    flight.run()


if __name__ == "__main__":
    fly()
