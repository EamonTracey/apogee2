from math import cos, radians, sin
import os

import click
import ndcctools.taskvine as vine

@click.command(context_settings={"show_default": True})
@click.argument("case")
@click.option("--attacks",
              "-a",
              multiple=True,
              default=(0.0, 5.0, 10.0, 15.0, 20.0),
              help="The angles of attack to parameterize.")
@click.option("--machs",
              "-m",
              multiple=True,
              default=(0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
              help="The mach numbers to parameterize.")
@click.option("--iterations",
              default=500,
              help="The number of iterations to execute the CFD run.")
@click.option("--port",
              default=14418,
              help="The port on which the TaskVine manager listens.")
def cfd(case: str, attacks: tuple[int, ...], machs: tuple[int, ...],
        iterations: int, port: int, email: str):
    """Launch parameterized Ansys Fluent CFD jobs via TaskVine.

    The case file must contain the vehicle mesh. The vehicle's roll axis must
    coincide with the x axis such that the drag force acts in the positive x
    direction.

    The journal must contain a specific set of tokens delimited by
    `{` and `}`. These tokens are mach, ..., iterations, input_case_file,
    output_axial_file, output_normal_file, output_case_data_file.
    """
    # Create the TaskVine manager.
    manager = vine.Manager(port)
    case_vine_file = manager.declare_file(case)

    # Retrieve the basename of the case file.
    case_name = os.path.basename(case)
    case_name = case_name.split(".", 2)[0]

    # Load the journal template.
    journal_file = f"data/journal.jou"
    journal_template: str
    with open(journal_file, "r") as file:
        journal_template = file.read()

    for attack in attacks:
        for mach in machs:
            name = f"{case_name}_{attack}_{mach}_{iterations}"

            # To induce an angle of attack, split the x and y components of the
            # flow velocity vector. Note that we assume the vehicle's roll axis
            # coincides with the x axis.
            angle_of_attack = radians(attack)
            flow_vector_x = cos(angle_of_attack)
            flow_vector_y = sin(angle_of_attack)

            # Paramterize the journal template.
            journal_paramaterized = journal_template.format(
                mach=mach,
                # ...,
                iterations=iterations,
                input_case_file=case)

            # Create the task with inputs and outputs.
            task = vine.Task(f"module load ansys; fluent 3ddp -t1 -gr -gu -i journal.jou")
            journal_vine_buffer = manager.declare_buffer(journal_paramaterized)
            axial_vine_file = manager.declare_file(f"{name}_axial.out")
            normal_vine_file = manager.declare_file(f"{name}_normal.out")
            task.add_input(case_vine_file, "case.cas.h5")
            task.add_input(journal_vine_buffer, "journal.jou")
            task.add_output(axial_vine_file, "axial.out")
            task.add_output(normal_vine_file, "normal.out")

            # Submit the task to TaskVine.
            manager.submit(task)

    while not manager.empty():
        task = manager.wait()

if __name__ == "__main__":
    cfd()
