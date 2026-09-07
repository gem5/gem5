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
print(f"Stitching {num_switches} distinct context switches back-to-back...")

# 2. Calculate Relative Time (in nanoseconds) for EACH switch
df['Relative_Time_ns'] = df.groupby('Window_ID')['Tick'].transform(lambda x: (x - x.min()) / 1000.0)

# 3. Create a "Stitched" timeline
# We add a small 2000ns gap between switches so they don't visually crash into each other
padding_ns = 2000
window_max_times = df.groupby('Window_ID')['Relative_Time_ns'].max() + padding_ns

# Calculate the starting X-offset for each context switch
offsets = window_max_times.cumsum().shift(1).fillna(0)

# Apply the offset to create a continuous sequence
df['Stitched_Time_ns'] = df['Relative_Time_ns'] + df['Window_ID'].map(offsets)

# 4. Collapse the Y-axis to remove empty space
unique_vaddrs = sorted(df['VAddr'].unique())
addr_to_y = {addr: i for i, addr in enumerate(unique_vaddrs)}
df['Y_Pos'] = df['VAddr'].map(addr_to_y)

# Create the plot
plt.figure(figsize=(18, 8))

# Plot all data sequentially
plt.scatter(df['Stitched_Time_ns'], df['Y_Pos'], alpha=0.6, s=15, color='darkblue', marker='o')

# Draw vertical dashed lines to separate the context switches
for offset in offsets[1:]:
    plt.axvline(x=offset - (padding_ns / 2), color='red', linestyle='--', alpha=0.5)

# Format the Y-axis to show hex addresses
max_labels = 40
if len(unique_vaddrs) > max_labels:
    y_ticks = np.linspace(0, len(unique_vaddrs) - 1, max_labels, dtype=int)
else:
    y_ticks = np.arange(len(unique_vaddrs))
    
y_labels = [f"0x{unique_vaddrs[i]:X}" for i in y_ticks]
plt.yticks(y_ticks, y_labels, fontsize=8)

plt.title(f'Sequential Memory Access: {num_switches} Context Switches (Idle Time Removed)', fontsize=16)
plt.xlabel('Continuous Time [Nanoseconds]', fontsize=14, fontweight='bold')
plt.ylabel('Accessed Data Addresses', fontsize=14, fontweight='bold')

plt.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()

# Save the figure
filename = "all_switches_sequential.png"
plt.savefig(filename, dpi=300, facecolor='white')
print(f"Plot successfully saved as '{filename}'")
