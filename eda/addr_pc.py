import matplotlib.pyplot as plt

# Filepath to the log file
log_file_path = './eda/addr_pc.log'

# Initialize lists to store addresses and PCs
addresses = []
pcs = []

# Read the log file
with open(log_file_path, 'r') as file:
    for line in file:
        if line.startswith('Addr:'):
            parts = line.split()
            addr = int(parts[1], 16)  # Convert hex address to integer
            pc = int(parts[3], 16)    # Convert hex PC to integer
            addresses.append(addr)
            pcs.append(pc)

# Plot the addresses and PCs as different data streams
plt.figure(figsize=(12, 6))

# Plot addresses
plt.plot(addresses, label='Addresses', color='blue')

# Plot PCs
plt.plot(pcs, label='PCs', color='red')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Addresses and PCs as Time Series')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()