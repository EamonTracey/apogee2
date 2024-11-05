from datetime import datetime
import logging
from math import cos, radians, sin
import os

import click

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s:%(levelname)s:%(message)s",
                    datefmt="%Y%m%d%H%M%S")


@click.command(context_settings={"show_default": True})
@click.argument("cases", nargs=-1, required=True)
@click.option("--attacks",
              "-a",
              multiple=True,
              default=(0.0, 5.0, 10.0, 15.0, 20.0),
              help="The angles of attack to parameterize.")
@click.option("--machs",
              "-m",
              multiple=True,
              default=(0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
              help="The Mach numbers to parameterize.")
@click.option("--iterations",
              "-i",
              default=500,
              help="The number of iterations to execute the CFD run.")
@click.option("--port",
              "-p",
              default=9340,
              help="The port on which the TaskVine manager listens.")
def cfd(cases: tuple[str, ...], attacks: tuple[int, ...],
        machs: tuple[int, ...], iterations: int, port: int):
    """Launch parameterized Ansys Fluent CFD jobs via TaskVine.

    The case file(s) must contain the vehicle mesh and four boundaries:
    farfield, inlet, outlet, and launchvehicle. The vehicle's roll axis must
    coincide with the x axis such that the drag force acts in the positive x
    direction.
    """
    import ndcctools.taskvine as vine

    # Create the TaskVine manager.
    manager = vine.Manager(port)
    logging.info(f"TaskVine manager listening on port {port}")

    # Load the journal template.
    journal_file = f"data/journal.jou"
    journal_template: str
    with open(journal_file, "r") as file:
        journal_template = file.read()

    # Make the output directory.
    now = datetime.now().strftime("%Y%m%d%H%M%S")
    output_directory = f"data/cfd/{now}"
    os.makedirs(output_directory, exist_ok=True)
    logger.info(f"Created output directory {output_directory}")

    for case in cases:
        # Retrieve the basename of the case file.
        case_name = os.path.basename(case)
        case_name = case_name.split(".", 2)[0]

        # Expose the case file to TaskVine.
        case_vine_file = manager.declare_file(case)

        # Perform CFD on a matrix of attack angles and Mach numbers.
        for attack in attacks:
            for mach in machs:
                name = f"{case_name}_{attack}_{mach}_{iterations}"

                # To induce an angle of attack, split the x and y components of
                # the flow velocity vector. Note that we assume the vehicle's
                # roll axis coincides with the x axis.
                angle_of_attack = radians(attack)
                flow_vector_x = cos(angle_of_attack)
                flow_vector_y = sin(angle_of_attack)

                # Paramterize the journal template.
                journal_paramaterized = journal_template.format(
                    mach=mach,
                    flow_vector_x=flow_vector_x,
                    flow_vector_y=flow_vector_y,
                    iterations=iterations)

                # Create the task with inputs and outputs.
                task = vine.Task(
                    "module load ansys/2024R1; "
                    "/opt/crc/a/ansys/2024R1/v241/fluent/bin/fluent "
                    "3ddp -t1 -g < journal.jou > log 2>&1")
                journal_vine_buffer = manager.declare_buffer(
                    journal_paramaterized)
                axial_vine_file = manager.declare_file(
                    f"{output_directory}/{name}.axial")
                normal_vine_file = manager.declare_file(
                    f"{output_directory}/{name}.normal")
                log_vine_file = manager.declare_file(
                    f"{output_directory}/{name}.log")
                task.add_input(case_vine_file, "case.cas.h5")
                task.add_input(journal_vine_buffer, "journal.jou")
                task.add_output(axial_vine_file, "axial.out")
                task.add_output(normal_vine_file, "normal.out")
                task.add_output(log_vine_file, "log")

                # Submit the task to TaskVine.
                manager.submit(task)
                logger.info(f"Submitted task {task.id} with "
                            f"{case_name=} {attack=} {mach=} {iterations=}")

    while not manager.empty():
        task = manager.wait(10)
        if task is not None:
            logger.info(
                f"Completed task {task.id} with exit code {task.exit_code}")
        else:
            workers_connected = manager.stats.workers_connected
            tasks_submitted = manager.stats.tasks_submitted
            tasks_running = manager.stats.tasks_running
            tasks_done = manager.stats.tasks_done
            logger.info(
                f"{tasks_done / tasks_submitted * 100:.1f}% complete "
                f"{workers_connected=} {tasks_running=}"
            )


if __name__ == "__main__":
    cfd()
