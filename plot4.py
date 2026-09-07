import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

print("Loading data...")
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])
df = df.dropna()

if df.empty:
    print("CSV is empty!")
    exit()

# Safely convert addresses to integers
df['VAddr'] = df['VAddr'].astype(str).apply(lambda x: int(x, 16) if x.startswith('0x') else int(float(x)))

# 1. Identify distinct context switches
gap_threshold = 10_000_000 
df['Tick_Diff'] = df['Tick'].diff().fillna(0)
df['Window_ID'] = (df['Tick_Diff'] > gap_threshold).cumsum()

num_switches = df['Window_ID'].nunique()
print(f"Overlaying {num_switches} distinct context switches onto one plot...")

# 2. Calculate Relative Time (in nanoseconds) for EACH switch
# This resets the clock to 0 for every single context switch window
df['Relative_Time_ns'] = df.groupby('Window_ID')['Tick'].transform(lambda x: (x - x.min()) / 1000.0)

# 3. Collapse the Y-axis to remove empty space
unique_vaddrs = sorted(df['VAddr'].unique())
addr_to_y = {addr: i for i, addr in enumerate(unique_vaddrs)}
df['Y_Pos'] = df['VAddr'].map(addr_to_y)

# Create the overlaid plot
plt.figure(figsize=(16, 9))

# Plot all data. 
# We use a very low alpha (transparency) and small dot size so overlapping accesses become darker.
plt.scatter(df['Relative_Time_ns'], df['Y_Pos'], alpha=0.1, s=15, color='crimson', marker='o')

# Format the Y-axis to show hex addresses without overlapping text
max_labels = 40
if len(unique_vaddrs) > max_labels:
    y_ticks = np.linspace(0, len(unique_vaddrs) - 1, max_labels, dtype=int)
else:
    y_ticks = np.arange(len(unique_vaddrs))
    
y_labels = [f"0x{unique_vaddrs[i]:X}" for i in y_ticks]
plt.yticks(y_ticks, y_labels, fontsize=8)

plt.title(f'Memory Access Signature: {num_switches} Context Switches Overlaid', fontsize=16)
plt.xlabel('Relative Time from Switch Start (Nanoseconds)', fontsize=14, fontweight='bold')
plt.ylabel('Accessed Data Addresses (Empty Space Removed)', fontsize=14, fontweight='bold')

plt.grid(True, linestyle='--', alpha=0.4)
plt.tight_layout()

# Save the figure
filename = "all_switches_overlaid.png"
plt.savefig(filename, dpi=300, facecolor='white')
print(f"Plot successfully saved as '{filename}'")
