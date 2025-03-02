import matplotlib.pyplot as plt

# Path to the log file
log_file_path = "/home/abishek/Desktop/SHELLY/gem5/eda/addr.log"

# Initialize lists to store the data
indices_addr = []
addresses_addr = []
indices_prefetch = []
addresses_prefetch = []

# Read the log file
with open(log_file_path, "r") as log_file:
    for line in log_file:
        parts = line.split()
        if len(parts) >= 3:
            index = int(parts[0])
            addr = int(parts[2], 16)  # Convert hexadecimal address to integer
            if parts[1] == "ADDR:":
                indices_addr.append(index)
                addresses_addr.append(addr)
            elif parts[1] == "PREFETCH:":
                indices_prefetch.append(index)
                addresses_prefetch.append(addr)

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(indices_addr, addresses_addr, marker='o', linestyle='-', color='b', label='ADDR')
plt.plot(indices_prefetch, addresses_prefetch, marker='x', linestyle='-', color='r', label='PREFETCH')
plt.xlabel("Index")
plt.ylabel("Address")
plt.title("Prefetch Addresses Over Time")
plt.legend()
plt.grid(True)
plt.show()