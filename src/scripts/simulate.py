import click

from simulation.dynamics import DynamicsSimulation
from simulation.motor import Motor
from simulation.vehicle import Vehicle


@click.command(context_settings={"show_default": True})
@click.option("--vehicle", default="drjoe", help="The vehicle to simulate.")
@click.option("--motor",
              default="aerotech_l1940x",
              help="The motor that provides thrust to the vehicle.")
@click.option("--hertz",
              default=100,
              help="The speed with which to run the software loop.")
def simulate(vehicle: str, motor: str, hertz: int):
    """Run a complete software-in-the-loop rocket flight simulation."""
    vehicle_file_path = f"data/vehicles/{vehicle}.json"
    vehicle = Vehicle.from_json(vehicle_file_path)

    motor_file_path = f"data/motors/{motor}.json"
    motor = Motor.from_json(motor_file_path)

    dynamics_simulation = DynamicsSimulation(vehicle, motor)


if __name__ == "__main__":
    simulate()
