import re
from collections import Counter

# Filepath to the address stream file
file_path = './eda/addr_pc.log'

# Initialize a list to store addresses
addresses = []

# Read the address stream file
with open(file_path, 'r') as file:
    for line in file:
        match = re.match(r'Addr: ([0-9a-fA-F]+) PC: ([0-9a-fA-F]+)', line)
        if match:
            addr = match.group(1)  # Extract the address
            addresses.append(addr)

# Count the occurrences of each address
address_counts = Counter(addresses)

# Find the most repeated addresses
most_repeated_addresses = address_counts.most_common(10)  # Change the number to get more or fewer results

# Print the most repeated addresses
print("Most Repeated Addresses:")
for addr, count in most_repeated_addresses:
    print(f'Address: {addr}, Count: {count}')