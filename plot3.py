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

df['VAddr'] = df['VAddr'].astype(str).apply(lambda x: int(x, 16) if x.startswith('0x') else int(float(x)))

# 1. Separate the 13 context switches by looking for time gaps
gap_threshold = 10_000_000 
df['Tick_Diff'] = df['Tick'].diff().fillna(0)
df['Window_ID'] = (df['Tick_Diff'] > gap_threshold).cumsum()

# 2. Extract ONLY the first Context Switch (Zooming in on the first "dot")
target_window = 0
df_zoom = df[df['Window_ID'] == target_window].copy()

# Convert Ticks to Nanoseconds relative to the start of this specific context switch
df_zoom['Time_ns'] = (df_zoom['Tick'] - df_zoom['Tick'].min()) / 1000.0

# 3. Collapse the Y-axis: Map unique addresses to sequential rows to remove empty space
unique_vaddrs = sorted(df_zoom['VAddr'].unique())
addr_to_y = {addr: i for i, addr in enumerate(unique_vaddrs)}
df_zoom['Y_Pos'] = df_zoom['VAddr'].map(addr_to_y)

print(f"Zooming in on Context Switch #{target_window + 1}")
print(f"Found {len(df_zoom)} memory accesses touching {len(unique_vaddrs)} unique addresses.")

plt.figure(figsize=(16, 9))

# Plot the zoomed-in data with larger dots
plt.scatter(df_zoom['Time_ns'], df_zoom['Y_Pos'], alpha=0.7, s=30, color='crimson', marker='o')

# Format the Y-axis. If there are too many addresses, we only show a subset so they don't overlap
y_ticks = np.linspace(0, len(unique_vaddrs) - 1, min(30, len(unique_vaddrs)), dtype=int)
y_labels = [f"0x{unique_vaddrs[i]:X}" for i in y_ticks]
plt.yticks(y_ticks, y_labels, fontsize=9)

plt.title(f'Zoomed Memory Access (Context Switch #{target_window + 1})', fontsize=16)
plt.xlabel('Relative Time (Nanoseconds)', fontsize=14, fontweight='bold')
plt.ylabel('Accessed Data Addresses (Empty Space Removed)', fontsize=14, fontweight='bold')

plt.grid(True, linestyle='--', alpha=0.4)
plt.tight_layout()

filename = "zoomed_address_vs_time.png"
plt.savefig(filename, dpi=300, facecolor='white')
print(f"Plot successfully saved as '{filename}'")
