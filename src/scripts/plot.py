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
    help="Include to plot full duration ACS is on. Default is flight only.")
@click.option("-p",
              "--plot_choice",
              type=click.Choice(
                  ["Alt", "Velz", "Accelz", "Servo"]),
              help="Plot selected data vs. time.")

## TODO: Add functionality to plot multiple datasets on same plot
## TODO: More data options as ACS algorithm improves (prediction, target, zenith, etc.)

def plot(path: str, significant: bool, alldata: bool, plot_choice: str):
    """A suite of tools to plot analyze flight data post-launch."""
    import matplotlib.pyplot as plt
    import pandas as pd

    path = ' '.join(path)
    df = pd.read_csv(path)

    # Prints to console a few important data points about the flight.
    if significant:
        
        time_on = df["Time"].iloc[-1]
        apogee_val = df["Altitude"].max()
        max_servo = df["Servo_Angle"].max()
        if "OVERSHOOT" not in df['Stage'].values:
            overshoot_time = 0.0
        else:
            overshoot_time = (df.loc[df["Stage"] == "OVERSHOOT", "Time"].max() -
                            df.loc[df["Stage"] == "OVERSHOOT", "Time"].min())

        print(f"Apogee: {apogee_val} ft")
        print(f"How long as ACS on: {time_on} sec")
        print(f"Max Servo Angle: {max_servo} degrees")
        print(f"Number of seconds in OVERSHOOT: {overshoot_time} s")

    # Plot entire duration of ACS being on vs. just flight
    if not alldata:

        indstart = df[df['Stage'] == "BURN"].index[0] - 300
        indend = df[(df['Altitude'] < df.at[indstart + 300, 'Altitude'])
                    & (df['Stage'] == "DESCENT")].index[0] + 300
        rangevals = range(indstart, indend)

    else:
        rangevals = range(0, df.index[-1])

    # Altitude Plot 
    if plot_choice == "Alt":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals, df.columns.get_loc("Altitude")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (ft)")
        plt.title("Filtered Altitude vs Time")

    # z-Velocity Plot
    elif plot_choice == "Velz":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals,
                         df.columns.get_loc("Velocity_Z")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Vertical Velocity (ft/s)")
        plt.title("Filtered Vertical Velocity vs Time")

    # z-Acceleration Plot
    elif plot_choice == "Accelz":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals,
                         df.columns.get_loc("Acceleration_Z")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Vertical Acceleration (ft/s^2)")
        plt.title("Filtered Vertical Acceleration vs Time")

    # Servo Angle Plot
    elif plot_choice == "Servo":

        plt.plot(df.iloc[rangevals, df.columns.get_loc("Time")],
                 df.iloc[rangevals,
                         df.columns.get_loc("Servo_Angle")],
                 color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Servo Angle (degrees)")
        plt.title("Servo Angle vs Time")

    if plot_choice is not None:
        plt.show()
