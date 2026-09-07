import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

print("Loading table data...")
df = pd.read_csv('m5out/instruction_usage_table_exact.csv', index_col=0)

# Drop the '-1' total row 
if -1 in df.index:
    df = df.drop(-1)

if df.empty:
    print("Table has no data!")
    exit()

# ==========================================
# FILTER 1: Keep only the Top 30 most used instructions
# ==========================================
TOP_N = 200
instruction_totals = df.sum(axis=0).sort_values(ascending=False)
top_instructions = instruction_totals.head(TOP_N).index
df_top = df[top_instructions]

# ==========================================
# FILTER 2: Get the LAST 150 Context Switches
# ==========================================
MAX_SWITCHES = 200
if len(df_top) > MAX_SWITCHES:
    df_top = df_top.tail(MAX_SWITCHES)

print(f"Plotting Top {TOP_N} instructions over the Last {len(df_top)} switches...")

# ==========================================
# PLOT: High-Contrast, Robust Heatmap
# ==========================================
plt.figure(figsize=(18, 10))

# Transpose data so Instructions are on Y-axis
data_to_plot = df_top.T

# Create a mask so that any count of exactly '0' is completely transparent (white background)
mask_zeros = (data_to_plot == 0)

# Configure the heatmap
ax = sns.heatmap(
    data_to_plot, 
    cmap='rocket_r',             # High contrast: Light orange to deep purple/black
    mask=mask_zeros,             # Hide all 0s to remove visual noise
    robust=True,                 # Ignore extreme outliers when calculating color scale
    linewidths=0.5,              # Thin white lines between boxes
    linecolor='white',
    cbar_kws={'label': 'Number of Executions'}
)

# Set a light gray background color for the plot area so white boxes stand out slightly
ax.set_facecolor("#f8f9fa")

plt.title(f'Instruction Signature Heatmap (Top {TOP_N} Insts, Last {len(df_top)} Switches)', fontsize=16, fontweight='bold', pad=15)
plt.xlabel('Context Switch Number', fontsize=14, fontweight='bold', labelpad=10)
plt.ylabel('Instruction Name / PC', fontsize=14, fontweight='bold', labelpad=10)

# Angle the X-axis labels slightly if they are squished
plt.xticks(rotation=45, ha='right', fontsize=9)
plt.yticks(fontsize=10)

filename = "optimized_heatmap_better_colors.png"
plt.savefig(filename, dpi=200, facecolor='white', bbox_inches='tight')
print(f"Plot saved successfully as '{filename}'")