import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np

# Filepath to the log file
log_file_path = './eda/addresses.log'

# Initialize lists to store time and addresses
times = []
addresses = []
requester_ids = []

# Read the log file
with open(log_file_path, 'r') as file:
    for i, line in enumerate(file):
        if line.startswith('Addr:'):
            parts = line.split()
            address = int(parts[1], 10)  # Convert hex address to integer
            requester_id = int(parts[7], 10)  # Convert requesterId to integer
            addresses.append(address)
            requester_ids.append(requester_id)
            times.append(i)  # Use the line number as the time

# Create a colormap with bright colors
cmap = cm.get_cmap('tab10', 4)  # 10 distinct bright colors

# Plot the addresses as a time series with different colors for each requester ID
plt.figure(figsize=(10, 6))
for requester_id in [0, 13, 14, 15]:
    indices = [i for i, rid in enumerate(requester_ids) if rid == requester_id]
    plt.plot([times[i] for i in indices], [addresses[i] for i in indices], 
             color=cmap(requester_id % 13), label=f'Requester ID {requester_id}', marker='o', markersize=5)

plt.xlabel('Time')
plt.ylabel('Address')
plt.title('Addresses as a Time Series')
plt.legend()
plt.grid(True)
plt.show()