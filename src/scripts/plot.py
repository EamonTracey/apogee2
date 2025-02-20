import click


@click.command(context_settings={"show_default": True})
@click.argument("filepath", type=str, nargs=1)
@click.option("-s",
              "--significant",
              is_flag=True,
              help="Display significant flight data.")
def plot(filepath, significant: bool):
    """Plot and review ACS flight data."""
    import pandas as pd

    # Displays a few important data points about the flight.
    if significant:
        df = pd.read_csv(filepath)
        timeon = df["Time"].iloc[-1]
        apogeeval = df["Filtered_Altitude"].max()
        slipcount = df["Loop_Slip_Count"].iloc[-1]
        print(f"How long as ACS on: {timeon} (s)")
        print(f"Apogee: {apogeeval} (ft)")
        print(f"Loop Slip Count: {slipcount}")
