from math import cos, radians, sin
import os

import click


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
@click.option("--journal",
              default="simple",
              help="The journal file to parameterize.")
@click.option("--email",
              default="etracey@nd.edu",
              help="The email associated with the CRC account.")
def cfd(case: str, attacks: tuple[int, ...], machs: tuple[int, ...],
        iterations: int, journal: str, email: str):
    """Generate parameterized CFD jobs to be run on the CRC machines.

    The case file must contain the vehicle mesh. The vehicle's roll axis must
    coincide with the x axis such that the drag force acts in the positive x
    direction.

    The journal must contain a specific set of tokens delimited by
    `{` and `}`. These tokens are mach, ..., iterations, input_case_file,
    output_axial_file, output_normal_file, output_case_data_file.
    """
    # Retrieve the basename of the case file.
    case_name = os.path.basename(case)
    case_name = case_name.split(".", 2)[0]

    # Load the journal template.
    journal_file_path = f"data/journals/{journal}.jou"
    journal_template: str
    with open(journal_file_path, "r") as file:
        journal_template = file.read()

    # Create the output directories.
    output_directory = f"data/cfd/{case_name}_{journal}"
    journal_directory = f"{output_directory}/journal"
    job_directory = f"{output_directory}/job"
    axial_directory = f"{output_directory}/axial"
    normal_directory = f"{output_directory}/normal"
    case_data_directory = f"{output_directory}/case_data"
    log_directory = f"{output_directory}/log"
    os.makedirs(journal_directory, exist_ok=True)
    os.makedirs(job_directory, exist_ok=True)
    os.makedirs(axial_directory, exist_ok=True)
    os.makedirs(normal_directory, exist_ok=True)
    os.makedirs(case_data_directory, exist_ok=True)
    os.makedirs(log_directory, exist_ok=True)

    for attack in attacks:
        for mach in machs:
            name = f"{case_name}_{journal}_{attack}_{mach}_{iterations}"

            # To induce an angle of attack, split the x and y components of the
            # flow velocity vector. Note that we assume the vehicle's roll axis
            # coincides with the x axis.
            angle_of_attack = radians(attack)
            flow_vector_x = cos(angle_of_attack)
            flow_vector_y = sin(angle_of_attack)

            # Paramterize the journal template.
            output_axial_file = f"{axial_directory}/{name}_axial.out"
            output_normal_file = f"{normal_directory}/{name}_normal.out"
            output_case_data_file = f"{case_data_directory}/{name}_case_data.cas.dat.h5"
            journal_paramaterized = journal_template.format(
                mach=mach,
                # ...,
                iterations=iterations,
                input_case_file=case,
                output_axial_file=output_axial_file,
                output_normal_file=output_normal_file,
                output_case_data_file=output_case_data_file)

            # Write the journal.
            output_journal_file = f"{journal_directory}/{name}_journal.jou"
            with open(output_journal_file, "w") as file:
                file.write(journal_paramaterized)

            # Write the job.
            ...


if __name__ == "__main__":
    cfd()
