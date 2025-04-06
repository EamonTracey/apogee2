import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV file
def plot_position_and_velocity(csv_file):
    # Read the CSV file
    df = pd.read_csv(csv_file)
    
    # Check if required columns exist
    required_columns = {"Time", "Position_Z", "Mass", "Linear_Momentum_Z"}
    if not required_columns.issubset(df.columns):
        print("Error: CSV file must contain 'Time', 'Position_Z', 'Mass', and 'Linear_Momentum_Z' columns.")
        return
    
    # Compute Velocity_Z
    df["Velocity_Z"] = df["Linear_Momentum_Z"] / df["Mass"]
    df["Motor Mass"] = df["Mass"] - 1.0660447709
    
    # Plot Position_Z and Velocity_Z vs Time
    plt.figure(figsize=(10, 5))
    
    plt.plot(df["Time"], df["Position_Z"])
#    plt.plot(df["Time"], df["Force_Axial_X"])
#    plt.plot(df["Time"], df["Force_Axial_Y"])
#    plt.plot(df["Time"], df["Force_Axial_Z"])
    plt.xlabel("Time")
    plt.ylabel("Position Z")
    plt.title("Position Z vs Time")
    plt.legend()
    plt.grid()
    
    plt.tight_layout()
    plt.show()

# Example usage
csv_file = sys.argv[1]
plot_position_and_velocity(csv_file)

