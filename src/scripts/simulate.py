import click
import matplotlib.pyplot as plt

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

    dynamics = DynamicsSimulation(vehicle, motor)

    # TODO: Run the simulation.
    time_delta = 1 / hertz

    times = []
    positions = []

    while dynamics.time < 1 or dynamics.vehicle_position >= 0:
        times.append(dynamics.time)
        positions.append(dynamics.vehicle_position)
        dynamics.step(time_delta)

    plt.xlabel("Time (seconds)")
    plt.ylabel("Altitude (meters)")
    plt.title("ACS Simulation")
    plt.plot(times, positions)
    plt.show()

if __name__ == "__main__":
    simulate()
