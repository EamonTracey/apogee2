import click


@click.command(context_settings={"show_default": True})
@click.argument("path", type=click.STRING, nargs=-1, required=True)
@click.option("-s",
              "--significant",
              is_flag=True,
              help="Display significant flight data.")
@click.option(
    "-a",
    "--alldata",
    is_flag=True,
    help="Toggle to display data for entire duration ACS is on vs. only flight."
)
@click.option("-p",
              "--plot_choice",
              type=click.Choice(
                  ["Altitude", "Velocity", "Acceleration", "Servo"]),
              help="Plot data vs. time.")
def plot(path: str, significant: bool, alldata: bool, plot_choice: str):
    """A suite of tools to plot analyze flight data post-launch."""
    import matplotlib.pyplot as plt
    import pandas as pd
    from base.stage import Stage

    path = ' '.join(path)
    df = pd.read_csv(path)

    keys = {
        "Altitude": "Altitude",
        "Velocity": "Velocity_Z",
        "Acceleration": "Acceleration_Z",
        "Servo": "Servo_Angle"
    }

    # Displays a few important data points about the flight.
    if significant:

        timeon = df["Time"].iloc[-1]
        apogeeval = df["Filtered_Altitude"].max()
        slipcount = df["Loop_Slip_Count"].iloc[-1]
        maxservo = df["Servo_Angle"].max()
        overshoottime = (df.loc[df["Stage"] == Stage.OVERSHOOT, "Time"].max() -
                         df.loc[df["Stage"] == Stage.OVERSHOOT, "Time"].min())

        print(f"How long as ACS on: {timeon} (s)")
        print(f"Apogee: {apogeeval} (ft)")
        print(f"Loop Slip Count: {slipcount}")
        print(f"Max Servo Angle: {maxservo} (degrees)")
        print(f"Number of seconds in OVERSHOOT: {overshoottime} (s)")

    if alldata:

        indstart = df[df['Stage'] == Stage.BURN].index[0] - 3000
        indend = df[(df['Altitude'] < df.at[indstart + 3000, 'Altitude'])
                    & (df['Stage'] == Stage.DESCENT)].index[0] + 3000

        rangevals = range(indstart, indend)

    else:
        rangevals = range(0, df.index[-1])

    if plot_choice == "Altitude":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals, df.columns.get_loc("Altitude")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (ft)")
        plt.title("Filtered Altitude vs Time")
        plt.show()

    elif plot_choice == "Velocity":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals,
                         df.columns.get_loc("Velocity_Z")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Vertical Velocity (ft/s)")
        plt.title("Filtered Vertical Velocity vs Time")
        plt.show()

    elif plot_choice == "Acceleration":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals,
                         df.columns.get_loc("Acceleration_Z")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Vertical Acceleration (ft/s^2)")
        plt.title("Filtered Vertical Acceleration vs Time")
        plt.show()

    elif plot_choice == "Servo":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals,
                         df.columns.get_loc("Servo_Angle")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Servo Angle (degrees)")
        plt.title("Servo Angle vs Time")
        plt.show()
