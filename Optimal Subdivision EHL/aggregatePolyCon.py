import os
import numpy as np

# Set the directory where the .txt files are located
directory = "tempRes"

# Open the output file for writing
output_file = open("aggregatePolyConOut.txt", "w")
output_file.write("maptype: [timeMean(microseconds) " +
"expansionsMean numVisibleMean] [stds]\n")

# Loop over all .txt files in the directory
for filename in os.listdir(directory):
    if filename.endswith(".txt"):
        # Open the file for reading
        with open(os.path.join(directory, filename), "r") as file:
            # Read the data from the file
            data = np.loadtxt(file, delimiter=",")
            # Calculate the mean and standard deviation of each column
            means = np.mean(data, axis=0)
            stds = np.std(data, axis=0)
            # Write the filename, means, and stds to the output file
            output_file.write(f"{filename}: {means} {stds}\n")

# Close the output file
output_file.close()