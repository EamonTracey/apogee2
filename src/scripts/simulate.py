import click

@click.command(context_settings={"show_default": True})
@click.option("--vehicle", default="drjoe", help="The vehicle to simulate.")
@click.option("--motor",
              default="aerotech_l1940x",
              help="The motor that provides thrust to the vehicle.")
@click.option("--environment",
              default="threeoaks_basic",
              help="The environment (e.g., temperature, pressure, wind).")
@click.option("--hertz",
              default=100,
              help="The speed at which to run the software loop.")
def simulate(vehicle: str, motor: str, environment: str, hertz: int):
    """Run a complete software-in-the-loop rocket flight simulation."""
    from base.stage import Stage
    from simulation.dynamics import DynamicsSimulation
    from simulation.environment import Environment
    from simulation.motor import Motor
    from simulation.vehicle import Vehicle

    vehicle_file = f"data/vehicles/{vehicle}.json"
    vehicle = Vehicle.from_json(vehicle_file)

    motor_file = f"data/motors/{motor}.json"
    motor = Motor.from_json(motor_file)

    environment_file = f"data/environments/{environment}.json"
    environment = Environment.from_json(environment_file)

    dynamics = DynamicsSimulation(vehicle, motor, environment)

    time_delta = 1 / hertz
    times = []
    positions = []
    state = dynamics.state
    while state.stage != Stage.DESCENT:
        times.append(state.time)
        positions.append(state.position)
        dynamics.step(time_delta)


if __name__ == "__main__":
    simulate()
