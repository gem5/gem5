import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# 1. بارگذاری داده‌ها
print("Loading data...")
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])

if df.empty:
    print("CSV is empty! No data captured.")
    exit()

print(f"Loaded {len(df)} memory accesses. Generating plots...")

# 2. پیدا کردن پرکاربردترین‌ها (Top 15)
top_n = 15
top_vaddrs = df['VAddr'].value_counts().head(top_n)
top_pcs = df['PC'].value_counts().head(top_n)

# ==========================================
# رسم نمودارها (در یک تصویر شامل ۳ بخش)
# ==========================================
fig = plt.figure(figsize=(18, 12))

# --- نمودار اول: پرکاربردترین آدرس‌های داده (Data Addresses) ---
ax1 = plt.subplot(2, 2, 1)
sns.barplot(x=top_vaddrs.values, y=top_vaddrs.index, palette='Blues_r', ax=ax1)
ax1.set_title(f'Top {top_n} Most Accessed Data Addresses (VAddr)')
ax1.set_xlabel('Number of Accesses')
ax1.set_ylabel('Data Address')

# --- نمودار دوم: پرکاربردترین آدرس‌های دستور (Instruction PCs) ---
ax2 = plt.subplot(2, 2, 2)
sns.barplot(x=top_pcs.values, y=top_pcs.index, palette='Oranges_r', ax=ax2)
ax2.set_title(f'Top {top_n} Most Memory-Intensive Instructions (PC)')
ax2.set_xlabel('Number of Accesses')
ax2.set_ylabel('Instruction Address (PC)')

# --- نمودار سوم: هیت‌مپ ارتباط PC و VAddr ---
ax3 = plt.subplot(2, 1, 2)

# فیلتر کردن داده‌ها فقط برای Top PCها و Top VAddrها تا هیت‌مپ شلوغ نشود
heatmap_data = df[df['PC'].isin(top_pcs.index) & df['VAddr'].isin(top_vaddrs.index)]

# ایجاد ماتریس تقاطع (Cross Tabulation)
pivot_table = pd.crosstab(heatmap_data['PC'], heatmap_data['VAddr'])

# رسم هیت‌مپ
sns.heatmap(pivot_table, annot=True, fmt="d", cmap="YlGnBu", linewidths=.5, ax=ax3)
ax3.set_title('Heatmap: Top Instructions (PC) vs. Top Data Addresses (VAddr)')
ax3.set_xlabel('Data Address (VAddr)')
ax3.set_ylabel('Instruction Address (PC)')

# تنظیم فاصله‌ها و ذخیره تصویر
plt.tight_layout()
plt.savefig('context_switch_address_analysis.png', dpi=300)
print("Analysis complete! Plot saved as 'context_switch_address_analysis.png'")
