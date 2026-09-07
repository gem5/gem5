import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

print("Loading data...")
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])

df = df.dropna()

if df.empty:
    print("CSV is empty after removing invalid rows!")
    exit()

# پاکسازی و تبدیل آدرس‌ها به عدد صحیح
df['VAddr'] = df['VAddr'].astype(str).apply(lambda x: int(x, 16) if x.startswith('0x') else int(float(x)))

# تبدیل Tick به میلی‌ثانیه (ms)
min_tick = df['Tick'].min()
df['Time_ms'] = (df['Tick'] - min_tick) / 1e9

# پیدا کردن پایین‌ترین و بالاترین آدرس
min_vaddr = df['VAddr'].min()
max_vaddr = df['VAddr'].max()

print(f"Total memory accesses to plot: {len(df)}")
print(f"Lowest Address (Min): 0x{min_vaddr:X}")
print(f"Highest Address (Max): 0x{max_vaddr:X}")

plt.figure(figsize=(16, 9))

# رسم نقاط
plt.scatter(df['Time_ms'], df['VAddr'], alpha=0.1, s=1, color='darkblue')

# ایجاد یک حاشیه (Margin) برای بالا و پایین نمودار (۵ درصدِ کل بازه)
address_range = max_vaddr - min_vaddr
margin = address_range * 0.05

# محدود کردن محور عمودی بین پایین‌ترین و بالاترین آدرس (به علاوه حاشیه)
plt.ylim(max(0, min_vaddr - margin), max_vaddr + margin)

# تنظیم برچسب‌های محور Y به صورت هگزادسیمال صحیح
formatter = ticker.FuncFormatter(lambda x, pos: f'0x{int(x):X}' if x >= 0 else '')
plt.gca().yaxis.set_major_formatter(formatter)

plt.title('Global Memory Access Timeline (Bounded Address Space)', fontsize=16)
plt.xlabel('Absolute Time (ms)', fontsize=14)
plt.ylabel('Data Address (VAddr)', fontsize=14)

plt.grid(True, linestyle='--', alpha=0.3)
plt.tight_layout()

# ذخیره فایل
filename = "timeline_bounded_address_space.png"
plt.savefig(filename, dpi=300, facecolor='white')
print(f"Success! Plot saved as '{filename}'")
