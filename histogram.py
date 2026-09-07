import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data
print("Loading data...")
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])

if df.empty:
    print("CSV is empty! No data captured.")
    exit()

print(f"Loaded {len(df)} memory accesses. Processing...")

# 1. Identify separate context switches
# In gem5, time between context switches is huge (milliseconds).
# Time between memory accesses within a switch is tiny.
# We look for a gap of > 10,000,000 Ticks (10 microseconds) to identify a new context switch.
gap_threshold_ticks = 10_000_000 
df['Tick_Diff'] = df['Tick'].diff().fillna(0)
df['New_Window'] = df['Tick_Diff'] > gap_threshold_ticks
df['Window_ID'] = df['New_Window'].cumsum()

# 2. Normalize time relative to the start of EACH context switch
# This makes every context switch start at Tick 0
df['Relative_Tick'] = df.groupby('Window_ID')['Tick'].transform(lambda x: x - x.min())
df['Relative_Time_ns'] = df['Relative_Tick'] / 1000.0  # Convert to nanoseconds

# 3. Define the bins for the histogram
# Now the max time is just the length of the longest single context switch!
max_duration_ns = df['Relative_Time_ns'].max()
bins = np.arange(0, max_duration_ns + 5, 5) # 5ns steps

print(f"Found {df['Window_ID'].max() + 1} context switches.")
print(f"Max context switch duration: {max_duration_ns} ns. Plotting...")

# 4. Create the histogram
plt.figure(figsize=(12, 6))

# We plot the aggregated relative times. 
# This stacks all context switches on top of each other to show the overall behavior pattern.
plt.hist(df['Relative_Time_ns'], bins=bins, color='skyblue', edgecolor='black')

# Formatting the plot
plt.title('Aggregated Memory Access Profile of a Context Switch', fontsize=14)
plt.xlabel('Time since Context Switch Started (Nanoseconds) [5ns steps]', fontsize=12)
plt.ylabel('Total Memory Accesses', fontsize=12)
plt.grid(axis='y', alpha=0.75)

plt.tight_layout()
plt.savefig('context_switch_histogram.png', dpi=300)
print("Plot saved as context_switch_histogram.png")
