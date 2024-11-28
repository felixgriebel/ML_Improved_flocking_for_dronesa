import matplotlib.pyplot as plt
import numpy as np

def plot_columns_adjacent_no_space(max_value, data_array):
    """
    Plots vertical columns based on the provided data_array and max_value.
    - The upper part is red with height (max-current).
    - The lower part is blue with height (current).
    - If the value is -1, the column is green.
    - Following a -1 column, all subsequent columns are gray.
    Each column corresponds directly to one step in the array, with no space between them.
    """
    num_columns = len(data_array)
    x_positions = np.arange(num_columns)  # One column per step
    width = 1.0  # Full width of each column for no space between
    got_minusone=False
    for i, (x, value) in enumerate(zip(x_positions, data_array)):
        if value == -1:
            got_minusone=True
            # Entire column is green
            plt.bar(x, max_value, color='green', width=width, align='edge')
        elif got_minusone:
            # Entire column is gray if preceded by -1
            plt.bar(x, max_value, color='gray', width=width, align='edge')
        else:
            # Split column: lower blue (current) and upper red (max-current)
            plt.bar(x, value, color='blue', width=width, align='edge')
            plt.bar(x, max_value - value, bottom=value, color='red', width=width, align='edge')

    # Customize the plot appearance
    plt.axhline(0, color='black', linewidth=0.8)  # Add a baseline
    plt.gca().spines['left'].set_position(('data', 0))  # Ensure y-axis is at x=0
    plt.gca().spines['bottom'].set_position(('data', 0))  # Ensure x-axis is at y=0
    plt.gca().spines['top'].set_visible(False)  # Hide top spine
    plt.gca().spines['right'].set_visible(False)  # Hide right spine
    plt.gca().spines['left'].set_bounds(0, max_value)  # Limit the y-axis line to positive values
    plt.gca().spines['bottom'].set_bounds(0, num_columns)  # Limit the x-axis line to valid range
    plt.xticks([0, num_columns//4, num_columns//2, 3*num_columns//4, num_columns], 
               labels=["0", "1/4", "1/2", "3/4", "1"])
    plt.xlabel("Array Index")
    plt.ylabel("Values")
    plt.title("Custom Column Plot (No Space, No Negative Axis Continuation)")
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()

from scipy.ndimage import gaussian_filter1d
def plot_time_series_with_avg(time_array, sigma=20):
    """
    Plots the original time series and its Gaussian smoothed version.
    - Original time series is plotted in blue.
    - Smoothed version is plotted in red, overlaying the original graph.
    """
    x_positions = np.arange(len(time_array))
    avg_time = np.mean(time_array)

    # Apply Gaussian smoothing
    smoothed_time_array = gaussian_filter1d(time_array, sigma=sigma)

    # Plot the original time series
    plt.plot(x_positions, time_array, label="Original Time (ms)", color="blue", alpha=0.7)

    # Plot the smoothed time series
    plt.plot(x_positions, smoothed_time_array, label="Smoothed Time (ms)", color="red", linewidth=2)

    # Annotate average on the smoothed plot
    plt.text(len(time_array) * 0.75, max(smoothed_time_array) * 0.9, f"Avg: {avg_time:.2f} ms", fontsize=10, color='black',
             bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

    # Customize the plot appearance
    plt.axhline(0, color='black', linewidth=0.8)  # Add a baseline
    plt.gca().spines['left'].set_position(('data', 0))  # Ensure y-axis is at x=0
    plt.gca().spines['bottom'].set_position(('data', 0))  # Ensure x-axis is at y=0
    plt.gca().spines['top'].set_visible(False)  # Hide top spine
    plt.gca().spines['right'].set_visible(False)  # Hide right spine
    plt.gca().spines['left'].set_bounds(0, max(smoothed_time_array))  # Limit y-axis to positive values only
    plt.gca().spines['bottom'].set_bounds(0, len(time_array)-1)  # Limit x-axis to the valid range
    plt.xticks([0, len(time_array)//4, len(time_array)//2, 3*len(time_array)//4, len(time_array)-1], 
               labels=["0", "1/4", "1/2", "3/4", "1"])
    plt.xlabel("Time Index")
    plt.ylabel("Milliseconds")
    plt.title("Time Series Plot with Gaussian Smoothing")
    plt.legend()
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()




def plot_multiple_smoothed_time_series_with_averages(time_arrays, sigma=20):
    """
    Plots multiple smoothed time series on the same graph.
    - Each time array is smoothed with a Gaussian filter with sigma=20.
    - Each smoothed time array is plotted as a separate graph.
    - The legend includes the average value of each array.
    """
    colors = plt.cm.tab10(np.linspace(0, 1, len(time_arrays)))  # Generate distinct colors for up to 10 arrays

    for i, time_array in enumerate(time_arrays):
        # Smooth the time array
        smoothed_time_array = gaussian_filter1d(time_array, sigma=sigma)

        # Compute the average
        avg_time = np.mean(time_array)

        # Plot the smoothed time series
        x_positions = np.arange(len(smoothed_time_array))
        plt.plot(x_positions, smoothed_time_array, label=f"Series {i+1} (Avg: {avg_time:.2f} ms)", color=colors[i % len(colors)])

    # Customize the plot appearance
    plt.axhline(0, color='black', linewidth=0.8)  # Add a baseline
    plt.gca().spines['left'].set_position(('data', 0))  # Ensure y-axis is at x=0
    plt.gca().spines['bottom'].set_position(('data', 0))  # Ensure x-axis is at y=0
    plt.gca().spines['top'].set_visible(False)  # Hide top spine
    plt.gca().spines['right'].set_visible(False)  # Hide right spine
    max_y_value = max([max(gaussian_filter1d(arr, sigma=sigma)) for arr in time_arrays])
    plt.gca().spines['left'].set_bounds(0, max_y_value)  # Limit y-axis to positive values only
    max_x_length = max(len(arr) for arr in time_arrays)
    plt.gca().spines['bottom'].set_bounds(0, max_x_length-1)  # Limit x-axis to the valid range
    plt.xticks([0, max_x_length//4, max_x_length//2, 3*max_x_length//4, max_x_length-1], 
               labels=["0", "1/4", "1/2", "3/4", "1"])
    plt.xlabel("Time Index")
    plt.ylabel("Milliseconds")
    plt.title("Multiple Smoothed Time Series Plot with Averages")
    plt.legend()
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()




# max_value = 100
# data = [80,80, 78, 74,72,66,61,55,43,33,23,21,19,18,15, -1, 60, 80]
# plot_columns_adjacent_no_space(max_value, data)

# # Example for Method 2
# times = [120, 150, 130, 170, 160, 140, 155,200]
# plot_time_series_with_avg(times)
