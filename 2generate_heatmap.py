import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.colors import LogNorm

print("[-] در حال رسم Heatmap با مقیاس لگاریتمی...")
df_exact = pd.read_csv("m5out/instruction_usage_table_exact.csv")

TOTAL_SWITCHES = 200
last_switches = sorted(df_exact["switch_id"].unique())[-TOTAL_SWITCHES:]
df_filtered = df_exact[df_exact["switch_id"].isin(last_switches)]

TOP_INSTS = 200
top_insts = (
    df_filtered.groupby("inst_label")["execution_count"]
    .sum()
    .sort_values(ascending=False)
    .head(TOP_INSTS)
    .index
)
df_filtered = df_filtered[df_filtered["inst_label"].isin(top_insts)]

pivot_matrix = df_filtered.pivot_table(
    index="inst_label", 
    columns="switch_id", 
    values="execution_count", 
    fill_value=0
)
pivot_matrix = pivot_matrix.loc[top_insts]

plt.figure(figsize=(18, 10))

# استفاده از مقیاس لگاریتمی برای نمایش همزمان جزییات کوچک و قله‌های بزرگ
sns.heatmap(
    pivot_matrix + 1,  # افزودن ۱ برای جلوگیری از log(0)
    norm=LogNorm(vmin=1, vmax=pivot_matrix.values.max()),
    cmap="rocket_r",
    cbar_kws={'label': 'Number of Executions (Log Scale)'},
    xticklabels=3,
    yticklabels=4,
    rasterized=True
)

plt.title(f"Instruction Signature Heatmap (Top {TOP_INSTS} Insts, Last {TOTAL_SWITCHES} Switches)", fontsize=16, weight="bold", pad=15)
plt.xlabel("Context Switch Number", fontsize=14, weight="bold")
plt.ylabel("Instruction Name / PC", fontsize=14, weight="bold")
plt.xticks(rotation=45, ha='right', fontsize=9)
plt.yticks(fontsize=8)

plt.savefig("analysis_plots/instruction_signature_heatmap_log.png", dpi=300, bbox_inches="tight")
plt.close()
print("[✓] نمودار لگاریتمی ذخیره شد: analysis_plots/instruction_signature_heatmap_log.png")
