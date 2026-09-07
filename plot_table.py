import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.ticker import MaxNLocator

print("Loading table data...")
# Read the table from the previous step
df = pd.read_csv('instruction_usage_table.csv', index_col=0)

# Drop the '-1' total row so it doesn't ruin the scale of the graph
if -1 in df.index:
    df = df.drop(-1)

if df.empty:
    print("Table has no context switch data to plot!")
    exit()

print(f"Plotting data for {len(df)} context switches...")

# ==========================================
# PLOT 1: Line Graph (Trends over time)
# X = Context Switch Number, Y = Count
# ==========================================
plt.figure(figsize=(16, 8))

# Plot each instruction as a separate line
for column in df.columns:
    plt.plot(df.index, df[column], marker='o', label=column, linewidth=2, markersize=4)

plt.title('Instruction Usage Trends Across Context Switches', fontsize=16, fontweight='bold')
plt.xlabel('Context Switch Number', fontsize=14, fontweight='bold')
plt.ylabel('Number of Times Executed', fontsize=14, fontweight='bold')

# Force X-axis to only show whole numbers (1, 2, 3...)
plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))

# Move legend completely outside the plot so it doesn't cover your data
plt.legend(title='Instruction', bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0.)

plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout()

file_lines = "instruction_plot_lines.png"
plt.savefig(file_lines, dpi=300, facecolor='white')
print(f"Line plot saved as '{file_lines}'")


# ==========================================
# PLOT 2: Heatmap (Exact X/Y match to your request)
# X = Context Switch Number, Y = Instruction Name, Color = Count
# ==========================================
plt.figure(figsize=(16, max(6, len(df.columns) * 0.4))) # Dynamically scale height based on instruction count

# Transpose the dataframe (df.T) so Instructions are on the Y-axis and Switches are on the X-axis
sns.heatmap(df.T, cmap='YlGnBu', annot=False, linewidths=.5, cbar_kws={'label': 'Number of Executions'})

plt.title('Instruction Signature Heatmap per Context Switch', fontsize=16, fontweight='bold')
plt.xlabel('Context Switch Number', fontsize=14, fontweight='bold')
plt.ylabel('Instruction Name', fontsize=14, fontweight='bold')

plt.tight_layout()

file_heatmap = "instruction_plot_heatmap.png"
plt.savefig(file_heatmap, dpi=300, facecolor='white')
print(f"Heatmap plot saved as '{file_heatmap}'")
