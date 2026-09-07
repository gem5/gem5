import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

print("Loading data...")
# Read the 3 columns: Tick (Time), Program Counter (Instruction), Virtual Address (Data)
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])

# Remove any corrupted rows
df = df.dropna()

if df.empty:
    print("CSV is empty after removing invalid rows!")
    exit()

# Safely convert the VAddr column to integers (handling hex strings and floats)
df['VAddr'] = df['VAddr'].astype(str).apply(lambda x: int(x, 16) if x.startswith('0x') else int(float(x)))

# Convert the gem5 Ticks to milliseconds, starting from T=0
min_tick = df['Tick'].min()
df['Time_ms'] = (df['Tick'] - min_tick) / 1e9

# Find the boundaries of your address space to zoom the Y-axis properly
min_vaddr = df['VAddr'].min()
max_vaddr = df['VAddr'].max()

print(f"Total data points: {len(df)}")
print(f"Time Range: 0 ms to {df['Time_ms'].max():.2f} ms")
print(f"Address Range: 0x{min_vaddr:X} to 0x{max_vaddr:X}")

# Create the plot
plt.figure(figsize=(16, 9))

# Scatter plot: X = Time, Y = Address
# Using a small point size (s=1) and low opacity (alpha=0.1) to show density
plt.scatter(df['Time_ms'], df['VAddr'], alpha=0.1, s=1, color='darkblue')

# Add a 5% margin to the top and bottom of the Y-axis so points don't hit the border
margin = (max_vaddr - min_vaddr) * 0.05
plt.ylim(max(0, min_vaddr - margin), max_vaddr + margin)

# Format the Y-axis labels to display as Hexadecimal (e.g., 0xffff...)
formatter = ticker.FuncFormatter(lambda x, pos: f'0x{int(x):X}' if x >= 0 else '')
plt.gca().yaxis.set_major_formatter(formatter)

# Add titles and labels
plt.title('Memory Access: Address vs. Time', fontsize=16)
plt.xlabel('Time (ms)', fontsize=14, fontweight='bold')
plt.ylabel('Address (VAddr)', fontsize=14, fontweight='bold')

plt.grid(True, linestyle='--', alpha=0.4)
plt.tight_layout()

# Save and display
filename = "address_vs_time.png"
plt.savefig(filename, dpi=300, facecolor='white')
print(f"Plot successfully saved as '{filename}'")
