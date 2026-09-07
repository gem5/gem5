import pandas as pd
import matplotlib.pyplot as plt
import math

# تنظیمات نمودار
MAX_ADDR_PER_PLOT = 50  # حداکثر تعداد آدرس در هر نمودار

# بارگذاری داده‌ها
print("Loading data...")
# چون فایل C++ شما تک‌هسته‌ای است، فقط ۳ ستون داریم:
df = pd.read_csv('m5out/ctx_mem_plot.csv', names=['Tick', 'PC', 'VAddr'])

# حذف هرگونه ردیف خالی یا ناقص برای جلوگیری از خطای NaN
df = df.dropna()

if df.empty:
    print("CSV is empty after removing invalid rows!")
    exit()

# پاکسازی و تبدیل آدرس‌ها به عدد صحیح (Int) برای جلوگیری از خطای اعشار
df['VAddr'] = df['VAddr'].astype(str).apply(lambda x: int(x, 16) if x.startswith('0x') else int(float(x)))

# تبدیل Tick به میلی‌ثانیه (ms)
min_tick = df['Tick'].min()
df['Time_ms'] = (df['Tick'] - min_tick) / 1e9

# پیدا کردن تمام آدرس‌های یکتا (Unique) و مرتب‌سازی آن‌ها
unique_vaddrs = sorted(df['VAddr'].unique())
total_unique = len(unique_vaddrs)
num_plots = math.ceil(total_unique / MAX_ADDR_PER_PLOT)

print(f"Total unique addresses: {total_unique}")
print(f"Generating {num_plots} separate plots (Max {MAX_ADDR_PER_PLOT} addresses per plot)...")

# تولید نمودارها
for i in range(num_plots):
    start_idx = i * MAX_ADDR_PER_PLOT
    end_idx = start_idx + MAX_ADDR_PER_PLOT
    chunk_vaddrs = unique_vaddrs[start_idx:end_idx]
    
    # فیلتر کردن داده‌ها فقط برای آدرس‌های این بخش
    chunk_df = df[df['VAddr'].isin(chunk_vaddrs)].copy()
    
    if chunk_df.empty:
        continue
        
    # نگاشت آدرس‌ها به موقعیت‌های خطی برای محور Y
    addr_to_y = {addr: y for y, addr in enumerate(chunk_vaddrs)}
    chunk_df['Y_Pos'] = chunk_df['VAddr'].map(addr_to_y)
    
    # رسم نمودار
    plt.figure(figsize=(16, 9))
    plt.scatter(chunk_df['Time_ms'], chunk_df['Y_Pos'], 
                alpha=0.6, s=15, color='darkblue', marker='|')
    
    # برچسب‌های محور Y به صورت هگزادسیمال صحیح
    plt.yticks(range(len(chunk_vaddrs)), [f"0x{addr:X}" for addr in chunk_vaddrs], fontsize=8)
    
    plt.title(f'Memory Access Timeline (Plot {i+1} of {num_plots})', fontsize=14)
    plt.xlabel('Absolute Time (ms)', fontsize=12)
    plt.ylabel('Data Address (VAddr)', fontsize=12)
    
    plt.grid(True, linestyle='--', alpha=0.4, axis='x')
    plt.tight_layout()
    
    # ذخیره فایل
    filename = f"timeline_plot_{i+1:02d}.png"
    plt.savefig(filename, dpi=200)
    plt.close()
    
    print(f"Saved {filename} with {len(chunk_vaddrs)} addresses.")

print("All plots generated successfully!")
