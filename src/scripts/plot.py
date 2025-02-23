import click


@click.command(context_settings={"show_default": True})
@click.argument("path", type=click.STRING, nargs=-1, required=True)

@click.option("-s",
              "--significant",
              is_flag=True,
              help="Display significant flight data.")
@click.option("-a",
              "--alldata",
              is_flag=True,
              help="Toggle to display data for entire duration ACS is on vs. only flight.")

@click.option("-p",
              "--plot_choice",
              type=click.Choice(["Altitude", "Velocity", "Acceleration", "Servo"]),
              help="Plot data vs. time.")

def plot(path: str, significant: bool, alldata: bool, plot_choice: str):

    import matplotlib.pyplot as plt
    import pandas as pd
    from base.stage import Stage

    path = ' '.join(path)
    df = pd.read_csv(path)

    keys = {"Altitude": "Altitude", 
            "Velocity": "Velocity_Z", 
            "Acceleration": "Acceleration_Z"
            "Servo": "Servo_Angle"}

    # Displays a few important data points about the flight.
    if significant:
        df = pd.read_csv(path)
        timeon = df["Time"].iloc[-1]
        apogeeval = df["Filtered_Altitude"].max()
        slipcount = df["Loop_Slip_Count"].iloc[-1]
        maxservo = df["Servo_Angle"].max()
        print(f"How long as ACS on: {timeon} (s)")
        print(f"Apogee: {apogeeval} (ft)")
        print(f"Loop Slip Count: {slipcount}")
        print(f"Max Servo Angle: {maxservo} degrees")
    
    if alldata:
        indstart = df[df['Stage'] == Stage.BURN].index[0]
        indend = df[(df['Altitude'] < df.at[indstart, 'Altitude']) & (df['Stage'] == Stage.DESCENT)].index[0]
        indstart = indstart - 3000
        indend = indend + 3000
        rangevals = indstart:indend
    else:   
        rangevals = 0:df.index[-1]

    
    if altitude:
            
        plt.plot(df["Time"], df["Altitude_BMP390"], color="red")
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (ft)")
        plt.title("Altitude BMP390")
        plt.show()


