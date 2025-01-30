import click

@click.command(context_settings={"show_default": True})
@click.argument("filepath", type=str, nargs=1)
@click.option("-s",
              "--significant",
              is_flag=True,
              help="Display significant flight data.")
@click.option("-af",
              "--altitudefull",
              is_flag=True,
              help="Full-flight altitude vs. time")
def plot(filepath, significant: bool, altitudefull: bool):
    """Plot and review ACS flight data."""
    import pandas as pd
    import matplotlib.pyplot as plt

    # Displays a few important data points about the flight.
    if significant:
        df = pd.read_csv(filepath)
        timeon = df["Time"].iloc[-1]
        apogeeval = df["Altitude_BMP390"].max()
        slipcount = df["Loop_Slip_Count"].iloc[-1]
        print(f"How long as ACS on: {timeon} (s)")
        print(f"Apogee: {apogeeval} (ft)")
        print(f"Loop Slip Count: {slipcount}")

    # Displays a full plot of alititude vs time for the entire time ACS is on.
    if altitudefull:
        print("Not Yet Implemented...")
