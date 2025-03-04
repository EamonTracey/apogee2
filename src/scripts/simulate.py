import logging

import click

@click.command(context_settings={"show_default": True})
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
def simulate(vehicle: str, motor: str, environment: str, frequency: int):
    """Run a complete software-in-the-loop rocket flight simulation."""
    from base.stage import Stage
    from simulation.dynamics import DynamicsComponent
    from simulation.environment import Environment
    from simulation.motor import Motor
    from simulation.simulation import Simulation
    from simulation.vehicle import Vehicle

    logging.basicConfig(
        handlers=[logging.StreamHandler()],
        format="%(asctime)s:%(name)s:%(levelname)s:%(message)s",
        datefmt="%Y%m%d%H%M%S",
        level=logging.INFO)

    vehicle_file = f"data/vehicles/{vehicle}.json"
    vehicle = Vehicle.from_json(vehicle_file)

    motor_file = f"data/motors/{motor}.json"
    motor = Motor.from_json(motor_file)

    environment_file = f"data/environments/{environment}.json"
    environment = Environment.from_json(environment_file)

    simulation = Simulation(frequency, vehicle, motor, environment)
    simulation.run()


if __name__ == "__main__":
    simulate()
